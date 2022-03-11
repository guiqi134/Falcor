#include "Utils/Math/MathConstants.slangh"

import Scene.Raster;
import Scene.Shading;
import Scene.Material.MaterialData;
import Experimental.Scene.Material.StandardMaterial;
import Experimental.Scene.Lights.LightHelpers;
import Utils.Sampling.SampleGenerator;
import Utils.Debug.PixelDebug;

import PCSS;
import VSM;

cbuffer PerFrameCB
{
    uint gFrameIndex;
    bool gEnableShadow;
    LightParams gLightParams;
    uint gShadowType;
}

Texture2D gShadowMap;
Texture2D gVSM;
Texture2D gSAT;

SamplerComparisonState gSamplerCmp;
SamplerState gLinearSampler;
SamplerState gPointSampler;


float4 psMain(VSOut vsOut, uint triangleIndex : SV_PrimitiveID) : SV_TARGET0
{
    float2 pixelPos = vsOut.posH.xy;
    printSetPixel(pixelPos);

    float4 color = float4(0, 0, 0, 1);

    SampleGenerator sg = SampleGenerator.create(pixelPos, gFrameIndex);
    SampleGenerator sg1 = SampleGenerator.create(pixelPos, 0);

    // Perform alpha test first
    if (alphaTest(vsOut, triangleIndex)) discard;

    // Prepare all shading data
    float3 viewDir = normalize(gScene.camera.getPosition() - vsOut.posW);
    ShadingData sd = prepareShadingData(vsOut, triangleIndex, viewDir);
    StandardMaterial mtl;

    // Self-shadowing bias
    float3 toLight = gLightParams.lightPos - sd.posW;
    float distSqr = max(dot(toLight, toLight), kMinLightDistSqr);
    float surfaceDistToLight = sqrt(distSqr);
    float3 L = toLight / surfaceDistToLight;
    float bias = _DEPTH_BIAS < 0.0f ? max(0.05 * (1.0 - dot(sd.N, L)), 0.001) : _DEPTH_BIAS; // TODO: better bias selecting algorithm

    // TODO: use random number from CPU and store them in a 3D texture. Use pixel world pos to index it.
    float2 poissonOffset = sampleNext2D(sg1) - 0.5f;
    float angle = sampleNext1D(sg1) * M_2PI;
    poissonOffset = float2(cos(angle), sin(angle));

    /*
    1. Since DX uses upper-left corner as the origin in screen space, we need to reverse y-axis
    2. In D3D the normalized device coordinates(x, y, z) form a 3D cube like so :
        x: -1 (left of screen) to 1 (right)
        y : -1 (bottom of screen) to 1 (top)
        z : 0 (near)to 1 (far)
    */
    float4 lightSpacePosH = mul(float4(sd.posW, 1.0f), gLightParams.lightSpaceMat); // clipping space
    float3 lightSpacePos = lightSpacePosH.xyz / lightSpacePosH.w; // NDC
    lightSpacePos.y *= -1;
    float3 projCoords = float3(lightSpacePos.xy * 0.5f + 0.5f, lightSpacePos.z - bias); // screen space
    float3 shadingPosLightView = mul(float4(sd.posW, 1.0f), gLightParams.lightView).xyz;
    float zReceiver = abs(shadingPosLightView.z);

    // numerical integration to sample the sphere light to get outgoing radiance
    float3 unshadowedSum = 0.0f;
    float3 shadowedSum = 0.0f;
    float visibility = 1.0f;
    LightData light = gScene.getLight(0);
    for (uint i = 0; i < _LIGHT_SAMPLES; i++)
    {
        AnalyticLightSample ls;
        sampleLight(sd.posW, light, sg, ls);
        if (dot(sd.N, ls.dir) < kMinCosTheta) continue;
        float3 radiance = mtl.eval(sd, ls.dir) * ls.Li;
        unshadowedSum += radiance;  

        // New PCSS need to integrate over light samples
        if (gShadowType == (uint)ShadowType::NewPCSS)
        {
            visibility = newPCSS(ls.posW, projCoords, shadingPosLightView, zReceiver, poissonOffset,
                gLightParams, gLinearSampler, gSamplerCmp, gShadowMap);
            shadowedSum += radiance * visibility;
        }
    }
    float3 unshadowed = unshadowedSum / _LIGHT_SAMPLES;

    // Which shadow mapping method to use
    switch (gShadowType)
    {
    case ShadowType::NewPCSS:
        float3 shadowed = shadowedSum / _LIGHT_SAMPLES;
        color.rgb = shadowed;
        break;
    case ShadowType::PCSS:
        visibility = PCSS(projCoords, zReceiver, poissonOffset, gLinearSampler, gSamplerCmp, gShadowMap);
        print("visibility = ", visibility);
        color.rgb = unshadowed * visibility;
        break;
    case ShadowType::VSM:
        // Lookup VSM and get two moments using trilinear filtering
        float depthNoBias = projCoords.z + bias; // TODO: change to linear depth
        //float2 moments = gVSM.Sample(gLinearSampler, projCoords.xy).xy;
        //print("moments = ", moments);
        //visibility = ChebyshevUpperBound(moments, depthNoBias);

        // Use SAT method in place of mipmaps
        // First blocker search
        float avgBlockerDepth, numBlockers;
        findBlocker(avgBlockerDepth, numBlockers, projCoords, zReceiver, poissonOffset, gLinearSampler, gShadowMap);

        // Calculate filter size
        color.rgb = unshadowed;
        if (numBlockers != 0)
        {
            float penumbraSize = _LIGHT_WORLD_SIZE * (zReceiver - avgBlockerDepth) / avgBlockerDepth; // depth in light view space
            float2 filterRadiusUV = float2(penumbraSize * _LIGHT_NEAR_PLANE / zReceiver);
            filterRadiusUV /= _LIGHT_FRUSTUM_SIZE;

            float2 shadowMapSize;
            gSAT.GetDimensions(shadowMapSize.x, shadowMapSize.y);
            visibility = shadowContribSAT(projCoords.xy, filterRadiusUV, shadowMapSize, depthNoBias, gSAT, gLinearSampler);
            color.rgb = unshadowed * visibility;
        }
        break;
    case ShadowType::EVSM:
    case ShadowType::MSM:
    }

    color.rgb += sd.emissive;
    color.a = sd.opacity;

    return color;
}


