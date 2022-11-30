#include "HostDeviceSharedDefinitions.h"

import Scene.Shading;
import Scene.Material.ShadingUtils;
import Scene.RaytracingInline;
import Rendering.Lights.LightHelpers;
import Utils.Debug.PixelDebug;
import Utils.Sampling.TinyUniformSampleGenerator;
import ISMHelpers;

typedef TinyUniformSampleGenerator SampleGenerator;

cbuffer ShadingCB
{
    uint gFrameIndex;
    uint2 gScreenSize;
    float2 gLightNearFar;
    SamplerState gLinearSampler;
}

Texture2D<PackedHitInfo> gVbuffer;
Texture2DArray<float> gIsmTextureArray;
StructuredBuffer<LightShadowData> gLightShadowDataBuffer;
RWTexture2D gOutColor;

bool evaluateShadowMaps(float3 posW, float3 normalW, uint lightID, float bias)
{
    // Check the shading is in which hemisphere
    LightShadowData lightShadowData = gLightShadowDataBuffer[lightID];
    float3 lightPosW = lightShadowData.centerPosW;
    int direction = sign(lightPosW.y - posW.y);
    uint ismOffset = direction == 1 ? 0 : 1;

    print("lightPosW = ", lightPosW);
    print("posW = ", posW);
    print("ismOffset = ", ismOffset);

    // Compute depth bias
    float3 L = normalize(lightPosW - posW);
    float NdotL = dot(normalW, L);
    float depthBias = bias * saturate(1.0 - NdotL) + bias;

    print("depthBias = ", depthBias);

    // Paraboloid project shading point
    float4x4 viewMat = lightShadowData.viewMats[ismOffset];
    float3 shadingPosLightView = mul(float4(posW, 1.0f), viewMat).xyz;
    float3 shadingPosLightNdc = paraboloidProjection(shadingPosLightView, gLightNearFar.x, gLightNearFar.y);
    float3 projCoord = float3(shadingPosLightNdc.xy * float2(0.5f, -0.5f) + 0.5f, shadingPosLightNdc.z - depthBias);

    print("projCoord = ", projCoord);

    // Sample the target face's shadow map
    float shadowMapDepth = gIsmTextureArray.SampleLevel(gLinearSampler, float3(projCoord.xy, 2 * lightID + ismOffset), 0);

    return shadowMapDepth < projCoord.z ? false : true;
}

[numthreads(16, 16, 1)]
void main(uint3 threadID : SV_DispatchThreadID)
{
    printSetPixel(threadID.xy);

    uint2 pixelPos = threadID.xy;
    SampleGenerator sg = SampleGenerator(pixelPos, gFrameIndex + 2 * 13);
    float3 rayDir = gScene.camera.computeRayPinhole(pixelPos, gScreenSize).dir;

    ShadingData sd = {};
    bool isVaildSurface = false;
    let lod = ExplicitLodTextureSampler(0.0f);

    // Extract the shading data from vBuffer
    {
        HitInfo hit = HitInfo(gVbuffer[pixelPos]);
        print("hit type = ", (uint)hit.getType());
        if (hit.isValid() && hit.getType() == HitType::Triangle)
        {
            const TriangleHit triangleHit = hit.getTriangleHit();
            const VertexData v = gScene.getVertexData(triangleHit);
            const uint materialID = gScene.getMaterialID(triangleHit.instanceID);
            sd = gScene.materials.prepareShadingData(v, materialID, -rayDir, lod);

            adjustShadingNormal(sd, v);
            isVaildSurface = true;
        }
        sd.V = -rayDir;
    }

    float3 res = 0.0f;
    let bsdf = gScene.materials.getBSDF(sd, lod);

    if (isVaildSurface)
    {
        // Sampling analytic lights
        uint sampleCount = 64;
        for (uint i = 0; i < sampleCount; i++)
        {
            const uint lightCount = gScene.getLightCount();
            uint lightIndex = min(uint(sampleNext1D(sg) * lightCount), lightCount - 1);
            // lightIndex = 0;
            float3 posWOrigin = sd.computeNewRayOrigin();
            float invPdf = lightCount;

            AnalyticLightSample als;
            if (sampleLight(posWOrigin, gScene.getLight(lightIndex), sg, als) && dot(sd.N, als.dir) >= kMinCosTheta)
            {
                Ray ray = Ray(posWOrigin, als.dir, 0.001f, als.distance - 0.001f);
                // float visibility = (float)traceSceneVisibilityRay<1>(ray, RAY_FLAG_NONE, 0xff);
                float visibility = (float)evaluateShadowMaps(posWOrigin, sd.N, lightIndex, 1e-6);
                print("visibility = ", visibility);
                float3 unshadowedContrib = als.Li * bsdf.eval(sd, als.dir, sg) * invPdf;
                res += unshadowedContrib * visibility;
            }
        }

        print("res = ", res);

        res = res / (float)sampleCount + bsdf.getProperties(sd).emission;
    }

    print("res = ", res);

    gOutColor[pixelPos] = float4(res, 1.0f);
}