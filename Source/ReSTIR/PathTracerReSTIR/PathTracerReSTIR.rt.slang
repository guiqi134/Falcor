/***************************************************************************
 # Copyright (c) 2015-21, NVIDIA CORPORATION. All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions
 # are met:
 #  * Redistributions of source code must retain the above copyright
 #    notice, this list of conditions and the following disclaimer.
 #  * Redistributions in binary form must reproduce the above copyright
 #    notice, this list of conditions and the following disclaimer in the
 #    documentation and/or other materials provided with the distribution.
 #  * Neither the name of NVIDIA CORPORATION nor the names of its
 #    contributors may be used to endorse or promote products derived
 #    from this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS "AS IS" AND ANY
 # EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 # IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 # PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 # CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 # PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 # PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 # OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 # OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **************************************************************************/

#include "Utils/Math/MathConstants.slangh"

import Scene.Raytracing;
import Utils.Math.MathHelpers;
import Utils.Sampling.SampleGenerator;
import Experimental.Scene.Material.StandardMaterial;
import Experimental.Scene.Material.MaterialHelpers;
import Experimental.Scene.Lights.LightHelpers;
import Utils.Color.ColorHelpers;
import Experimental.Scene.Lights.EmissiveLightSampler; // need to set _EMISSIVE_LIGHT_SAMPLER_TYPE define
import Experimental.Scene.Lights.EmissiveLightSamplerHelpers;
import Utils.Debug.PixelDebug;

import RenderPasses.Shared.ReSTIR.ReSTIRUtils;
import RenderPasses.Shared.ReSTIR.GBufferUtils;

cbuffer CB
{
    uint gFrameCount;        // Frame count since scene was loaded.
    uint gPRNGDimension;     // First available PRNG dimension.
    bool gTemporalReuse;
    bool gSpatialReuse;
    bool gUnbiased;
}

static struct Data
{
    // Materials
    StandardMaterial standardMaterial;
} gData;

// Outputs
RWTexture2D<float4> gOutputColor;

// Temporal data
RWStructuredBuffer<Reservoir> gLastFrameReservoirs;
RWStructuredBuffer<GBuffer> gLastFrameGBuffer;

// Static configuration based on defines set from the host.
#define is_valid(name) (is_valid_##name != 0)
static const uint kMaxBounces = MAX_BOUNCES;
static const bool kComputeDirect = COMPUTE_DIRECT;
static const bool kUseReSTIR = USE_RESTIR;
static const bool kUseAnalyticLights = USE_ANALYTIC_LIGHTS;
static const bool kUseEmissiveLights = USE_EMISSIVE_LIGHTS;
static const bool kUseEnvLight = USE_ENV_LIGHT;
static const bool kUseEnvBackground = USE_ENV_BACKGROUND;
static const float3 kDefaultBackgroundColor = float3(0, 0, 0);
static const float kRayTMax = FLT_MAX;

/** Payload for shadow ray.
*/
struct ShadowRayData
{
    bool visible;
};

/** Payload for scatter ray (80B).
*/
struct ScatterRayData
{
    float3  radiance;       ///< Accumulated outgoing radiance from path.
    bool    terminated;     ///< Set to true when path is terminated.
    float3  thp;            ///< Current path throughput. This is updated at each path vertex.
    uint    pathLength;     ///< Path length in number of path segments (0 at origin, 1 at first secondary hit, etc.). Max 2^31.
    float3  origin;         ///< Next path segment origin.
    uint    _pad0;
    float3  direction;      ///< Next path segment direction.
    uint    _pad1;

    SampleGenerator sg;     ///< Per-ray state for the sample generator (up to 16B).

    /** Create ray payload with default parameters.
    */
    static ScatterRayData create(SampleGenerator sg)
    {
        ScatterRayData d;
        d.terminated = false;
        d.pathLength = 0;
        d.radiance = float3(0, 0, 0);
        d.thp = float3(1, 1, 1);
        d.origin = float3(0, 0, 0);
        d.direction = float3(0, 0, 0);
        d.sg = sg;
        return d;
    }
};

/** Traces a scatter ray based on ray parameters stored in the ray payload.
    \param[in] rayData Describes the ray parameters. The struct is modified based on the result.
*/
void traceScatterRay(inout ScatterRayData rayData)
{
    RayDesc ray;
    ray.Origin = rayData.origin;
    ray.Direction = rayData.direction;
    ray.TMin = 0.f;
    ray.TMax = kRayTMax;

    uint rayFlags = 0;      // TODO: Set cull mode from the app
    TraceRay(gScene.rtAccel, rayFlags, 0xff, 0, rayTypeCount, 0, ray, rayData);
}

/** Traces a shadow ray towards a light source.
    \param[in] origin Ray origin for the shadow ray.
    \param[in] dir Direction from shading point towards the light source (normalized).
    \param[in] distance Distance to the light source.
    \return True if light is visible, false otherwise.
*/
bool traceShadowRay(float3 origin, float3 dir, float distance)
{
    RayDesc ray;
    ray.Origin = origin;
    ray.Direction = dir;
    ray.TMin = 0.f;
    ray.TMax = distance;

    ShadowRayData rayData;
    rayData.visible = false;    // Set to true by miss shader if ray is not terminated before
    TraceRay(gScene.rtAccel, RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH, 0xff, 1, rayTypeCount, 1, ray, rayData);

    return rayData.visible;
}

/** Evaluates the direct illumination from analytic lights.
    This function samples Falcor's light list uniformly with one shadow ray.
    \param[in] sd Shading data.
    \param[in] rayOrigin Ray origin for the shadow ray.
    \param[in] sg SampleGenerator object.
    \return Outgoing radiance in view direction.
*/
float3 evalDirectAnalytic(const ShadingData sd, float3 rayOrigin, inout SampleGenerator sg)
{
    const uint lightCount = gScene.getLightCount();
    if (lightCount == 0) return float3(0);

    // Pick one of the analytic light sources randomly with equal probability.
    const uint lightIndex = min(uint(sampleNext1D(sg) * lightCount), lightCount - 1);
    float invPdf = lightCount; // Light selection pdf = 1.0 / lightCount.

    // Sample local light source.
    AnalyticLightSample ls;
    bool valid = sampleLight(rayOrigin, gScene.getLight(lightIndex), sg, ls);

    // Reject sample if lower hemisphere.
    if (!valid || dot(ls.dir, sd.N) <= kMinCosTheta) return float3(0);

    // Test visibility by tracing a shadow ray.
    bool V = traceShadowRay(rayOrigin, ls.dir, ls.distance);

    float3 radiance = V ? gData.standardMaterial.eval(sd, ls.dir) * ls.Li * invPdf : float3(0);
    
    return radiance;
}

/** Processes a hit point to generate a scatter ray or terminate.
    This function generates a cosine-weighted direction over the hemisphere.
    \param[in] sd Shading data.
    \param[in] rayOrigin Ray origin for the new ray.
    \param[in] rayData Ray payload.
    \return True if the path continues.
*/
bool generateScatterRay(const ShadingData sd, float3 rayOrigin, inout ScatterRayData rayData)
{
    // Generate scatter ray as cosine-weighted direction over the hemisphere.
    float pdf = 0.f;
    float2 u = sampleNext2D(rayData.sg);
    float3 wi = sample_cosine_hemisphere_concentric(u, pdf);
    wi = sd.fromLocal(wi);

    rayData.origin = rayOrigin;
    rayData.direction = wi;
    rayData.thp *= pdf > 0.f ? gData.standardMaterial.eval(sd, wi) / pdf : float3(0);
    return any(rayData.thp > 0.f);
}

/** ********************* Ray index 0: Scatter ray ************************ */

[shader("miss")]
void scatterMiss(inout ScatterRayData rayData : SV_RayPayload)
{
    // Ray missed the scene. Mark the ray as terminated.
    rayData.terminated = true;

    // Add contribution from distant light (env map) in this direction.
    if (kUseEnvLight && (kComputeDirect || rayData.pathLength > 0))
    {
        float3 Le = gScene.envMap.eval(WorldRayDirection());
        rayData.radiance += rayData.thp * Le;
    }
}

[shader("anyhit")]
void scatterAnyHit(inout ScatterRayData rayData : SV_RayPayload, BuiltInTriangleIntersectionAttributes attribs : SV_IntersectionAttributes)
{
    // Alpha test for non-opaque geometry.
    GeometryInstanceID instanceID = getGeometryInstanceID();
    VertexData v = getVertexData(instanceID, PrimitiveIndex(), attribs);
    const uint materialID = gScene.getMaterialID(instanceID);
    if (alphaTest(v, gScene.materials[materialID], gScene.materialResources[materialID], 0.f)) IgnoreHit();
}

// no need for adding emissive light
[shader("closesthit")]
void scatterClosestHit(inout ScatterRayData rayData : SV_RayPayload, BuiltInTriangleIntersectionAttributes attribs : SV_IntersectionAttributes)
{
    // Evaluate Falcor's material parameters at the hit point.
    // TODO: Implement texLOD to enable texture filtering in prepareShadingData().
    GeometryInstanceID instanceID = getGeometryInstanceID();
    VertexData v = getVertexData(instanceID, PrimitiveIndex(), attribs);
    const uint materialID = gScene.getMaterialID(instanceID);
    ShadingData sd = prepareShadingData(v, materialID, gScene.materials[materialID], gScene.materialResources[materialID], -WorldRayDirection(), 0.f);

    // Add emitted light if hit it 
    if (kUseEmissiveLights && (kComputeDirect || rayData.pathLength > 0))
    {
        rayData.radiance += rayData.thp * sd.emissive;
    }

    // Check whether to terminate based on max depth.
    if (rayData.pathLength >= kMaxBounces)
    {
        rayData.terminated = true;
        return;
    }

    // Compute ray origin for new rays spawned from the hit.
    float3 rayOrigin = sd.computeNewRayOrigin();

    // Add contribution of direct light from analytic lights.
    if (kUseAnalyticLights)
    {
        float3 Lr = evalDirectAnalytic(sd, rayOrigin, rayData.sg);
        rayData.radiance += rayData.thp * Lr;
    }

    // Generate scatter ray for the next path segment.
    // The raygen shader will continue the path based on the returned payload.
    if (!generateScatterRay(sd, rayOrigin, rayData))
    {
        rayData.terminated = true;
        return;
    }

    rayData.pathLength++;
}

/** ************************ Ray index 1: Shadow ray ************************ */

[shader("miss")]
void shadowMiss(inout ShadowRayData rayData : SV_RayPayload)
{
    // The miss shader is executed if the ray misses all geometry. Mark as visible.
    rayData.visible = true;
}

[shader("anyhit")]
void shadowAnyHit(inout ShadowRayData rayData : SV_RayPayload, BuiltInTriangleIntersectionAttributes attribs : SV_IntersectionAttributes)
{
    // Alpha test for non-opaque geometry.
    GeometryInstanceID instanceID = getGeometryInstanceID();
    VertexData v = getVertexData(instanceID, PrimitiveIndex(), attribs);
    const uint materialID = gScene.getMaterialID(instanceID);
    if (alphaTest(v, gScene.materials[materialID], gScene.materialResources[materialID], 0.f)) IgnoreHit();
}

// ReSTIR utility functions
// -------------------------------------------------------------------

float computePHat(const ShadingData sd, const Reservoir r, bool visibility)
{
    TriangleLightSample tls;
    bool valid = sampleTriangle(sd.posW, r.y, r.uv, tls);
    if (!valid || !length(tls.Le)) return 0.0f;

    bool V = true;
    // if (visibility) V = traceShadowRay(rayOrigin, currentTls.dir, currentTls.distance);

    float pHat = V ? luminance(gData.standardMaterial.eval(sd, tls.dir) * tls.Le / (tls.pdf)) : 0.0f;

    return pHat;
}

void combineReservoirs(inout Reservoir current, const Reservoir other, const ShadingData sd, inout SampleGenerator sg, inout uint M)
{
    float pHatCurrent = computePHat(sd, other, false);
    float wi = pHatCurrent * other.W * other.M;

    current.update(other.y, other.uv, wi, sg);
    M += other.M;
}

void combineReservoirsUnbiased(inout Reservoir current, const Reservoir other, const ShadingData sd, inout SampleGenerator sg, 
    inout uint M, inout uint Z, uint bufferIndex, const GBuffer pixelGBuffer)
{
    // prepare shading struct for neighbor pixel 
    ShadingData sdNeighbor = prepareShadingDataHelper(bufferIndex, pixelGBuffer);

    // evaluate neighbor sample at current pixel
    float pHatCurrent = computePHat(sd, other, false); 
    float wi = pHatCurrent * other.W * other.M;
    current.update(other.y, other.uv, wi, sg);
    M += other.M;

    // TODO: add visibility reuse
    // evaluate current sample at neighbor pixel
    float pHatNeighbor = computePHat(sdNeighbor, current, true);
    if (pHatNeighbor > 0) Z += other.M;
}


/** Calculate screen-space motion vector.
    \param[in] pixelCrd Sample in current frame expressed in pixel coordinates with origin in the top-left corner.
    \param[in] prevPosH Sample in previous frame expressed in homogeneous clip space coordinates. Note that the definition differs between D3D12 and Vulkan.
    \param[in] renderTargetDim Render target dimension in pixels.
    \return Motion vector pointing from current to previous position expressed in sceen space [0,1] with origin in the top-left corner.
*/
float2 calcMotionVector(float2 pixelCrd, float4 prevPosH, float2 renderTargetDim)
{
    float2 prevCrd = prevPosH.xy / prevPosH.w;
#ifdef FALCOR_VK
    prevCrd *= float2(0.5, 0.5);
#else
    prevCrd *= float2(0.5, -0.5);
#endif
    prevCrd += 0.5f; // screen space 
    float2 normalizedCrd = pixelCrd / renderTargetDim;
    return prevCrd - normalizedCrd; // Q: why in [0, 1] range? not [-1, 1]?
}


// Evalute direct illumination from emissive light (no visibility reuse)
// -------------------------------------------------------------------
float3 evalDirectLightEmissive(const uint2 pixelPos, const uint2 dims, const ShadingData sd, float3 rayOrigin, 
    inout SampleGenerator sg, inout Reservoir s)
{
    float3 radiance;
    uint M = s.M;
    uint Z = 0; 
    float pHat;
    const float EPSILON = 0.0001f;

    uint currBufferIndex = getBufferIndex(pixelPos, dims);
    GBuffer currPixelGBuffer = gGBuffer[currBufferIndex];

    // temporal reuse 
    // -------------------------------------------------------------------
    if (gTemporalReuse)
    {
        float2 motionVec = currPixelGBuffer.mvec; 
        // float2 pixelPosNorm = (((pixelPos.xy + 0.5f) / dims.xy) * 2.f - 1.f);
        float2 pixelPosNorm = (pixelPos.xy + 0.5f) / dims;
        float2 prevFramePosNorm = clamp(pixelPosNorm + motionVec, float2(0.0f), float2(1.0f)); 
        uint2 prevFramePos = uint2(prevFramePosNorm * dims);
        // uint2 prevFramePos = uint2(float2(pixelPos.xy) + motionVec * dims + float2(0.5f));

        uint prevBufferIndex = getBufferIndex(prevFramePos, dims);
        GBuffer prevPixelGBuffer = gLastFrameGBuffer[prevBufferIndex];

        // get previous frame reservoir and update current
        Reservoir prevResevoir = gLastFrameReservoirs[prevBufferIndex];
        if (gUnbiased)
        {
            combineReservoirsUnbiased(s, prevResevoir, sd, sg, M, Z, prevBufferIndex, prevPixelGBuffer);
            s.M = min(M, 20 * s.M); // stops unbounded grow of M
            pHat = computePHat(sd, s, false);
            // Z = Z == 0 ? 1 : Z;
            s.W = pHat == 0 || Z == 0 ? 0.0f : (s.wSum / (pHat * Z));
        }
        else
        {
            combineReservoirs(s, prevResevoir, sd, sg, M);
            s.M = min(M, 20 * s.M); // stops unbounded grow of M
            pHat = computePHat(sd, s, false);
            s.W = pHat == 0.0f ? 0.0f : (s.wSum / (pHat * s.M));
        }
        print("current reservoir M after temporal reuse: ", s.M);
    }


    // spatial reuse
    // -------------------------------------------------------------------
    if (gSpatialReuse)
    {
        // TODO: use UI to adjust these numbers
        const uint n = 1; // number of spatial reuse iteration
        const uint k = 3; // number of spatial neighbors (5 for biased, 3 for unbiased) 

        for (uint i = 1; i <= n; i++)
        {
            M = s.M;
            Z = 0;
            // combine neighbor reservoirs 
            for (uint j = 0; j < k; j++)
            {
                // TODO: chaneg to low discrepancy sample in 30-pixel radius circle
                // pick saptial neighbor pixel
                int2 offset = int2(sampleNext2D(sg) * 60) - 30; // [-30, 30]
                uint2 neighborIndex = uint2(max(uint2(0), pixelPos + offset));

                uint neighborBufferIndex = getBufferIndex(neighborIndex, dims);
                GBuffer neighborPixelGBuffer = gGBuffer[neighborBufferIndex];

                // get neighbor reservoir
                Reservoir neighborReservoir = gReservoirs[neighborBufferIndex];

                // combine neighbor to current pixel
                if (gUnbiased)
                    combineReservoirsUnbiased(s, neighborReservoir, sd, sg, M, Z, neighborBufferIndex, neighborPixelGBuffer);
                else
                    combineReservoirs(s, neighborReservoir, sd, sg, M);
            }

            // update current reservoir M and W
            s.M = M;
            pHat = computePHat(sd, s, false);
            if (gUnbiased)
                s.W = pHat == 0 || Z == 0 ? 0.0f : (s.wSum / (pHat * Z));
            else
                s.W = pHat == 0 ? 0.0f : (s.wSum / (pHat * s.M));
        }
        print("current reservoir M after spatial reuse: ", s.M);
    }


    // compute pixel color
    TriangleLightSample tls;
    bool valid = sampleTriangle(sd.posW, s.y, s.uv, tls);
    const uint triangleLightCount = gScene.lightCollection.getActiveTriangleCount();
    float invPdf = triangleLightCount;

    if (!valid || !length(tls.Le)) return float3(0.0f);

    print("light sample dir = ", tls.dir);
    print("light sample distance = ", tls.distance);
    print("light sample Le = ", tls.Le);
    print("light sample pdf = ", tls.pdf);

    bool V = traceShadowRay(rayOrigin, tls.dir, tls.distance);

    float3 f = V ? gData.standardMaterial.eval(sd, tls.dir) * tls.Le / tls.pdf : float3(0.0f);
    radiance = f * s.W;
    return radiance;
}

/** ******************************** RayGen ******************************** */
[shader("raygeneration")]
void rayGen()
{
    uint2 launchIndex = DispatchRaysIndex().xy; // current pixel
    uint2 launchDim = DispatchRaysDimensions().xy;

    printSetPixel(launchIndex);

    float3 outColor = float3(0, 0, 0);

    // get current pixel's GBuffer. 
    uint bufferIndex = getBufferIndex(launchIndex, launchDim);
    GBuffer currentGBuffer = gGBuffer[bufferIndex];

    const float3 rayDir = -currentGBuffer.viewW.xyz;
    const float4 worldPos = currentGBuffer.posW;

    // current pixel reservoir
    Reservoir r = gReservoirs[bufferIndex];

    if (worldPos.w != 0.f)  // Using w to indicate valid geometry for now
    {
        // Prepare ShadingData struct.
        ShadingData sd = prepareShadingDataHelper(bufferIndex, currentGBuffer);

        // Create sample generator.
        SampleGenerator sg = SampleGenerator.create(launchIndex, gFrameCount);

        // Advance the generator to the first available dimension.
        // TODO: This is potentially expensive. We may want to store/restore the state from memory if it becomes a problem.
        for (uint i = 0; i < gPRNGDimension; i++) sampleNext1D(sg);

        // Compute ray origin for new rays spawned from the G-buffer(posW: hit position).
        const float3 rayOrigin = sd.computeNewRayOrigin();

        if (kComputeDirect)
        {   
            // Always output directly emitted light from the primary hit.
            // This is independent of whether emissive materials are treated as light sources or not. (must)
            outColor += sd.emissive;

            if (kUseEmissiveLights)
            {
                if (kUseReSTIR && !length(sd.emissive)) 
                    outColor += evalDirectLightEmissive(launchIndex, launchDim, sd, rayOrigin, sg, r);
            }

            // Add contribution of direct light from analytic lights.
            // Light probe and mesh lights are handled by the scatter ray hit/miss shaders.
            if (kUseAnalyticLights)
            {
                outColor += evalDirectAnalytic(sd, rayOrigin, sg);  // change to evalDirectLightAnalytic()
            }
        }

        // // Prepare ray payload.
        // ScatterRayData rayData = ScatterRayData.create(sg);

        // // Generate scatter ray.
        // if (!generateScatterRay(sd, rayOrigin, rayData)) rayData.terminated = true;

        // // Follow path into the scene and compute its total contribution.
        // for (uint depth = 0; depth <= kMaxBounces && !rayData.terminated; depth++)
        // {
        //     // Trace scatter ray. If it hits geometry, the closest hit shader samples
        //     // direct illumination and generates the next scatter ray.
        //     traceScatterRay(rayData);
        // }

        // // Store contribution from scatter ray.
        // outColor += rayData.radiance;
    }
    else
    {
        // Background pixel.
        outColor = kUseEnvBackground ? gScene.envMap.eval(rayDir) : kDefaultBackgroundColor;
    }

    // Update temporal data
    gLastFrameReservoirs[bufferIndex] = r;
    gLastFrameGBuffer[bufferIndex] = gGBuffer[bufferIndex];

    print("Color = ", outColor);
    
    gOutputColor[launchIndex] = float4(outColor, 1);
}
