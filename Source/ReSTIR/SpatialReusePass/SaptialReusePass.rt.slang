/**
This pass Is Using input GBuffer data, Do a simple RIS calculation to generate reservoir For
each pixel. Thus, it only has the shaders for raygeneration and shadow. It doesn't need to perform any scatter
ray calculation.
*/

// TODO: change to inline ray tracing

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
    uint gFrameCount;
    uint gPRNGDimension;
}

static struct Data
{
    StandardMaterial standardMaterial;
} gData;

// Payload for shadow ray
struct ShadowRayData
{
    bool visible;
}

// Inputs: GBuffer data
Texture2D<float4> gWorldPosition;
Texture2D<float4> gWorldShadingNormal;
Texture2D<float4> gWorldShadingTangent; // Optional
Texture2D<float4> gWorldFaceNormal;
Texture2D<float4> gWorldView; // Optional
Texture2D<float4> gMaterialDiffuseOpacity;
Texture2D<float4> gMaterialSpecularRoughness;
Texture2D<float4> gMaterialEmissive;
Texture2D<float4> gMaterialExtraParams;
Texture2D<float2> gMotionVectors;

void setGBufferData(uint index, uint2 pixelPos)
{
    gGBuffer[index].posW = gWorldPosition[pixelPos];
    gGBuffer[index].normalW = gWorldShadingNormal[pixelPos];
    gGBuffer[index].tangentW = gWorldShadingTangent[pixelPos];
    gGBuffer[index].faceNormal = gWorldFaceNormal[pixelPos];
    gGBuffer[index].viewW = gWorldView[pixelPos];
    gGBuffer[index].mtlDiffOpacity = gMaterialDiffuseOpacity[pixelPos];
    gGBuffer[index].mtlSpecRough = gMaterialSpecularRoughness[pixelPos];
    gGBuffer[index].mtlEmissive = gMaterialEmissive[pixelPos];
    gGBuffer[index].mtlParams = gMaterialExtraParams[pixelPos];
    gGBuffer[index].mvec = gMotionVectors[pixelPos];
}

bool traceShadowRay(float3 origin, float3 dir, float distance)
{
    RayDesc ray;
    ray.Origin = origin;
    ray.Direction = dir;
    ray.TMin = 0.0f;
    ray.TMax = distance;

    ShadowRayData rayData;
    rayData.visible = false;
    TraceRay(gScene.rtAccel, RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH, 0xff, 0, rayTypeCount, 0, ray, rayData);

    return rayData.visible;
}

// Ray index 0: shadow ray
// ------------------------------------------------
[shader("miss")]
void shadowMiss(inout ShadowRayData rayData : SV_RayPayLoad)
{
    rayData.visible = true;
}

[shader("anyhit")]
void shadowAnyHit(inout ShadowRayData rayData : SV_RayPayLoad, BuiltInTriangleIntersectionAttributes attribs : SV_IntersectionAttributes)
{
    // Alpha test for non-opaque geometry.
    GeometryInstanceID instanceID = getGeometryInstanceID();
    VertexData v = getVertexData(instanceID, PrimitiveIndex(), attribs);
    const uint materialID = gScene.getMaterialID(instanceID);
    if (alphaTest(v, gScene.materials[materialID], gScene.materialResources[materialID], 0.f)) IgnoreHit();
}

void generateCandidates(const ShadingData sd, float3 rayOrigin, inout SampleGenerator sg, inout Reservoir r)
{
    LightCollection lc = gScene.lightCollection;

    if (lc.isEmpty()) return;
    const uint triangleLightCount = lc.getActiveTriangleCount();

    // Use RIS (Algo.3)
    // ------------------------------------------------------------------
    uint M = min(triangleLightCount, 32);
    float invPdf = triangleLightCount;
    float pHat = 0.0f;
    float pHatSelect = 0.0f;

    for (uint i = 0; i < M; i++)
    {
        // Safety precaution as the result of the multiplication may be rounded to triangleCount even if uLight < 1.0 when triangleCount is large.
        uint xi = min(uint(sampleNext1D(sg) * triangleLightCount), triangleLightCount - 1); 
        // uint xi = 10;
        uint triangleIndex = lc.activeTriangles[xi]; // get the actual global triangle index

        // get light sample data
        TriangleLightSample tls;
        float2 uv = sampleNext2D(sg); 
        bool valid = sampleTriangle(sd.posW, triangleIndex, uv, tls);

        // Emitted radiance. This is zero if the light is back-facing or sample is invalid.
        // tls.pdf would be 0, and we will get NaN, so we need to remove invaild sample
        if (!valid || !length(tls.Le))
        {  
            r.update(triangleIndex, uv, 0.0f, sg);
            continue;
        }

        // eval() already contain cosine term
        pHat = luminance(gData.standardMaterial.eval(sd, tls.dir) * tls.Le / tls.pdf); 

        float wi = pHat * invPdf;

        if (r.update(triangleIndex, uv, wi, sg)) 
        {
            pHatSelect = pHat;
        }
    }

    // calculate Equation (6)
    TriangleLightSample currentTls;
    bool vaild = sampleTriangle(sd.posW, r.y, r.uv, currentTls);
    r.W = vaild ? r.wSum / (pHatSelect * r.M) : 0.0f;

    print("light sample dir = ", currentTls.dir);
    print("light sample distance = ", currentTls.distance);
    print("light sample Le = ", currentTls.Le);
    print("light sample pdf = ", currentTls.pdf);

    // Evaluate visibility for initial candidates
    bool V = traceShadowRay(rayOrigin, currentTls.dir, currentTls.distance);
    if (!V) r.W = 0;
}

[shader("raygeneration")]
void rayGen()
{
    uint2 launchIndex = DispatchRaysIndex().xy;
    uint2 launchDim = DispatchRaysDimensions().xy;

    printSetPixel(launchIndex);

    uint bufferIndex = getBufferIndex(launchIndex, launchDim);
    setGBufferData(bufferIndex, launchIndex);
    GBuffer pixelGBuffer = gGBuffer[bufferIndex];

    const float3 rayDir = getPrimaryRayDir(bufferIndex, launchIndex, launchDim, gScene.camera);
    const float4 worldPos = pixelGBuffer.posW;

    Reservoir r = {0, float2(0.0), 0.0f, 0, 0.0f}; // y, uv, wSum, M, W

    // If pixel is not a vaild primary hit, return the default reservoir
    if (worldPos.w != 0.f)
    {
        // load geometry params for preparing shading data
        float3 normal = pixelGBuffer.normalW.xyz;
        float4 tangent = pixelGBuffer.tangentW;
        float3 faceNormal = pixelGBuffer.faceNormal.xyz;
        GeometryParams geoParams = prepareGeometryParams(worldPos.xyz, -rayDir, normal, tangent, faceNormal);

        // load material parameters
        MaterialParams matParams = loadMaterialParams(bufferIndex);

        // prepare shading data
        ShadingData sd = prepareShadingData(geoParams, matParams);

        // create uniform sample generator
        SampleGenerator sg = SampleGenerator.create(launchIndex, gFrameCount);

        for (uint i = 0; i < gPRNGDimension; i++) sampleNext1D(sg);

        const float3 rayOrigin = sd.computeNewRayOrigin();

        // generate candidate(update reservoir) for current pixel
        generateCandidates(sd, rayOrigin, sg, r);
    }

    gReservoirs[bufferIndex] = r;
}
