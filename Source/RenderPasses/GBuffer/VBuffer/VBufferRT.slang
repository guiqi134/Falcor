/***************************************************************************
 # Copyright (c) 2015-22, NVIDIA CORPORATION. All rights reserved.
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
__exported import Scene.Shading;
__exported import Utils.Timing.GpuTimer;
__exported import Utils.Math.Ray;
import Utils.Math.MathHelpers;
import Utils.Sampling.SampleGenerator;
import Utils.Debug.PixelDebug;
import Scene.Material.ShadingUtils;


RWTexture2D<PackedHitInfo> gVBuffer;
RWTexture2D<float>  gDepth;
RWTexture2D<float2> gMotionVector;
RWTexture2D<float4> gViewW;
RWTexture2D<uint> gTime;

// For denoising
RWTexture2D<float4> gPosW;
RWTexture2D<float4> gNormW;
RWTexture2D<float2> gLinearZ;
RWTexture2D<float4> gEmissive;
RWTexture2D<float4> gAlbedo;
RWTexture2D<float2> gPosNormalFwidth;

#define is_valid(name) (is_valid_##name != 0)

#if !defined(COMPUTE_DEPTH_OF_FIELD) || !defined(USE_ALPHA_TEST) || !defined(RAY_FLAGS)
#error "Not all defines are set!"
#endif

struct GBufferData
{
    float4 posW;
    float4 normW;
    float4 tangentW;
    float4 faceNormalW;
    float2 texC;
    uint4 mtlData;

    // Legacy channels
    float4 diffuseOpacity;
    float4 specRough;
    float4 emissive;
};

/** Helper function to prepare G-buffer data.
*/
GBufferData prepareGBufferData(const ShadingData sd, const IBSDF bsdf)
{
    GBufferData gbuf;

    // We store the final normal and tangent in the G-buffer.
    // In order to reconstruct the bitangent later, we also need to store its handedness (sign).
    float3 B = cross(sd.N, sd.T);
    float bitangentSign = dot(sd.B, B) >= 0.f ? 1.f : -1.f;

    gbuf.posW = float4(sd.posW, 1.f);
    gbuf.normW = float4(sd.N, 0.f);
    gbuf.tangentW = float4(sd.T, bitangentSign);
    gbuf.faceNormalW = float4(sd.faceN, 0.f);
    gbuf.texC = sd.uv;
    gbuf.mtlData = uint4(sd.materialID, sd.mtl.packedData.x, sd.mtl.packedData.y, 0);

    // Get BSDF properties.
    let bsdfProperties = bsdf.getProperties(sd);

    // Setup legacy material channels.
    gbuf.diffuseOpacity = float4(bsdfProperties.diffuseReflectionAlbedo, sd.opacity);
    gbuf.specRough = float4(bsdfProperties.specularReflectance, bsdfProperties.roughness);
    gbuf.emissive = float4(bsdfProperties.emission, 0.f);

    return gbuf;
}

struct VBufferRT
{
    static const bool kComputeDepthOfField = COMPUTE_DEPTH_OF_FIELD;
    static const bool kUseAlphaTest = USE_ALPHA_TEST;
    static const uint kRayFlags = RAY_FLAGS;

    uint2 frameDim;
    uint frameCount;
    float2 invFrameDim;

    /** Ray differentials for primary hit. Code from RayTracingGems, Chapter 20.
    */
    void computeRayDifferentials(const TriangleHit hit, float3 rayDir, float hitT, const Camera camera, float2 invFrameDim, out float2 ddx, out float2 ddy)
    {
        // TODO: Is this code correct for instance transforms that flip the handedness of the coordinate system?

        // Ray differentials
        float3 P[3];
        gScene.getVertexPositionsW(hit.instanceID, hit.primitiveIndex, P);
        float3 e1 = P[1] - P[0];
        float3 e2 = P[2] - P[0];
        float3 d = rayDir;
        float k = dot(cross(e1, e2), d);
        k = abs(k) > 1e-20f ? rcp(k) : 0.0f;
        float3 cu = cross(e2, d);
        float3 cv = cross(d, e1);
        // Assumes a normalized ray direction
        float3 dx = camera.data.cameraU * 2.f * invFrameDim.x / camera.data.focalDistance; // dDdx in ray gen
        float3 dy = camera.data.cameraV * 2.f * invFrameDim.y / camera.data.focalDistance; // dDdy in ray gen
        float3 q = dx * hitT;                                                              // Transfer to primary hit
        float3 r = dy * hitT;
        float dudx = k * dot(cu, q);
        float dudy = k * dot(cu, r);
        float dvdx = k * dot(cv, q);
        float dvdy = k * dot(cv, r);
        float2 T[3];
        gScene.getVertexTexCoords(hit.instanceID, hit.primitiveIndex, T);
        float2 g1 = T[1] - T[0];
        float2 g2 = T[2] - T[0];
        float dsdx = (dudx * g1.x + dvdx * g2.x);
        float dsdy = (dudy * g1.x + dvdy * g2.x);
        float dtdx = (dudx * g1.y + dvdx * g2.y);
        float dtdy = (dudy * g1.y + dvdy * g2.y);
        ddx = float2(dsdx, dtdx);
        ddy = float2(dsdy, dtdy);
    }

    float3 computeDdxPosW(float3 posW, float3 normW, float2 invFrameDim)
    {
        float3 projRight = normalize(cross(normW, cross(normW, gScene.camera.data.cameraV)));
        float distanceToHit = length(posW - gScene.camera.data.posW);
        float2 ddNdc = float2(2.f, -2.f) * invFrameDim;
        float distRight = distanceToHit * ddNdc.x / dot(normalize(gScene.camera.data.cameraV), projRight);
        return distRight * projRight;
    }

    float3 computeDdyPosW(float3 posW, float3 normW, float2 invFrameDim)
    {
        float3 projUp = normalize(cross(normW, cross(normW, gScene.camera.data.cameraU)));
        float distanceToHit = length(posW - gScene.camera.data.posW);
        float2 ddNdc = float2(2.f, -2.f) * invFrameDim;
        float distUp = distanceToHit * ddNdc.y / dot(normalize(gScene.camera.data.cameraU), projUp);
        return distUp * projUp;
    }

    Ray generateRay(uint2 pixel)
    {
        if (kComputeDepthOfField)
        {
            SampleGenerator sg = SampleGenerator(pixel, frameCount);
            return gScene.camera.computeRayThinlens(pixel, frameDim, sampleNext2D(sg));
        }
        else
        {
            return gScene.camera.computeRayPinhole(pixel, frameDim);
        }
    }

    void writeHit(uint2 pixel, float3 rayOrigin, float3 rayDir, const HitInfo hit)
    {
        gVBuffer[pixel] = hit.getData();

        VertexData v;
        float depth = 1.f;
        float3 prevPosW = float3(0.f);
        float2 motionVector = float2(0.f);
        uint materialID = 0;
        ITextureSampler lod = ExplicitLodTextureSampler(0.f); // The default LOD method is replaced below in some cases.

        if (hit.getType() == HitType::Triangle)
        {
            const TriangleHit triangleHit = hit.getTriangleHit();
            materialID = gScene.getMaterialID(triangleHit.instanceID);
            v = gScene.getVertexData(triangleHit);
            prevPosW = gScene.getPrevPosW(triangleHit);
        }
        else if (hit.getType() == HitType::DisplacedTriangle)
        {
            const DisplacedTriangleHit displacedTriangleHit = hit.getDisplacedTriangleHit();
            materialID = gScene.getMaterialID(displacedTriangleHit.instanceID);
            v = gScene.getVertexData(displacedTriangleHit, -rayDir);
            prevPosW = gScene.getPrevPosW(displacedTriangleHit);
        }
        else if (hit.getType() == HitType::Curve)
        {
            const CurveHit curveHit = hit.getCurveHit();
            materialID = gScene.getMaterialID(curveHit.instanceID);
            v = gScene.getVertexDataFromCurve(curveHit);
            prevPosW = gScene.getPrevPosWFromCurve(curveHit);
        }
        else if (hit.getType() == HitType::SDFGrid)
        {
            const SDFGridHit sdfGridHit = hit.getSDFGridHit();
            materialID = gScene.getMaterialID(sdfGridHit.instanceID);
            v = gScene.getVertexDataFromSDFGrid(sdfGridHit, rayOrigin, rayDir);

            prevPosW = gScene.getPrevPosWFromSDFGrid(sdfGridHit, v.posW);
        }


        if (hit.getType() == HitType::Triangle || hit.getType() == HitType::DisplacedTriangle || hit.getType() == HitType::Curve || hit.getType() == HitType::SDFGrid)
        {
            // Compute depth similar to raster (NDC).
            float4 curPosH = mul(float4(v.posW, 1.f), gScene.camera.data.viewProjMatNoJitter);
            depth = curPosH.z / curPosH.w;

            print("prevPosW = ", prevPosW);
            print("posW = ", v.posW);
            const TriangleHit triangleHit = hit.getTriangleHit();
            const GeometryInstanceData instance = gScene.getGeometryInstance(triangleHit.instanceID);
            //const float4x4 prevWorldMat = gScene.loadPrevWorldMatrix(instance.globalMatrixID);
            const float4x4 prevWorldMat = gScene.getPrevWorldMatrix(triangleHit.instanceID);
            //const float4x4 worldMat = gScene.loadWorldMatrix(instance.globalMatrixID);
            const float4x4 worldMat = gScene.getWorldMatrix(triangleHit.instanceID);
            for (uint i = 0; i < 4; i++)
                print("prevWorldMat = ", prevWorldMat[i]);
            for (uint i = 0; i < 4; i++)
                print("worldMat = ", worldMat[i]);

            // Compute motion vector.
            float2 pixelPos = pixel + float2(0.5f, 0.5f);
            float4 prevPosH = mul(float4(prevPosW, 1.f), gScene.camera.data.prevViewProjMatNoJitter);
            motionVector = calcMotionVector(pixelPos, prevPosH, frameDim) + float2(gScene.camera.data.jitterX, -gScene.camera.data.jitterY); // Remove camera jitter from motion vector
        }

        // Prepare shading data.
        ShadingData sd = gScene.materials.prepareShadingData(v, materialID, -rayDir, lod);

        if (true && (hit.getType() == HitType::Triangle || hit.getType() == HitType::DisplacedTriangle))
        {
            adjustShadingNormal(sd, v);
        }

        // Create BSDF instance.
        let bsdf = gScene.materials.getBSDF(sd, lod);
        let bsdfProperties = bsdf.getProperties(sd);

        const GBufferData gbuf = prepareGBufferData(sd, bsdf);

        // For denoising
        if (is_valid(gPosW)) gPosW[pixel] = gbuf.posW;
        if (is_valid(gNormW)) gNormW[pixel] = gbuf.normW;
        if (is_valid(gEmissive)) gEmissive[pixel] = gbuf.emissive;
        if (is_valid(gAlbedo)) gAlbedo[pixel] = float4((bsdfProperties.diffuseReflectionAlbedo + bsdfProperties.diffuseTransmissionAlbedo + bsdfProperties.specularReflectionAlbedo
            + bsdfProperties.specularTransmissionAlbedo), 1.0f);

        if (is_valid(gLinearZ))
        {
            float4 curPosH = mul(float4(sd.posW, 1.f), gScene.camera.data.viewProjMatNoJitter);
            float curLinearZ = curPosH.w;

            // TODO: Improve computation of derivatives:
            float3 ddxPosW = computeDdxPosW(sd.posW, sd.faceN, invFrameDim);
            float3 ddyPosW = computeDdyPosW(sd.posW, sd.faceN, invFrameDim);
            float4 curPosH_dx = mul(float4(sd.posW + ddxPosW, 1.f), gScene.camera.data.viewProjMatNoJitter);
            float4 curPosH_dy = mul(float4(sd.posW + ddxPosW, 1.f), gScene.camera.data.viewProjMatNoJitter);
            float ddxLinearZ = abs(curPosH_dx.w - curLinearZ);
            float ddyLinearZ = abs(curPosH_dy.w - curLinearZ);
            float dLinearZ = max(ddxLinearZ, ddyLinearZ);
            gLinearZ[pixel] = float2(curLinearZ, dLinearZ);
        }

        // Length of derivatives of position and normal
        if (is_valid(gPosNormalFwidth))
        {
            gPosNormalFwidth[pixel] = float2(length(fwidth(sd.posW)), length(fwidth(sd.N)));
        }

        if (is_valid(gDepth))           gDepth[pixel]           = depth;
        if (is_valid(gMotionVector))    gMotionVector[pixel]    = motionVector;
    }

    void writeMiss(uint2 pixel, float3 rayOrigin, float3 rayDir)
    {
        gVBuffer[pixel] = {};

        if (is_valid(gDepth))           gDepth[pixel]           = 1.f;
        if (is_valid(gMotionVector))    gMotionVector[pixel]    = {};
    }

    void writeAux(uint2 pixel, const Ray ray)
    {
        // Write view direction.
        if (is_valid(gViewW)) gViewW[pixel] = float4(-ray.dir, 0.f);
    }

    void beginTime(inout GpuTimer timer)
    {
        if (is_valid(gTime)) timer.start();
    }

    void endTime(uint2 pixel, inout GpuTimer timer)
    {
        if (is_valid(gTime)) gTime[pixel] = timer.getElapsed();
    }
};
