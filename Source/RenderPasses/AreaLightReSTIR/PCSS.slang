#include "Utils/Math/MathConstants.slangh"
#define _DEVICE_NEED_INCLUDE
#include "HostDeviceSharedDefinitions.h"

import Utils.Debug.PixelDebug;
import Utils.Sampling.SampleGenerator;
import Utils.Math.MathHelpers;
import Scene.Scene;
import Experimental.Scene.Lights.LightHelpers;
import Experimental.Scene.Lights.EmissiveLightSampler;

__exported import PoissonDisk;

float LinearizeDepth(float depth, float n, float f)
{
    return (n * f) / (depth * (f - n) - f);
}

float2 rotatedPoisson(float2 poissonSample, float2 rotation)
{
    return float2(
        poissonSample.x * rotation.x - poissonSample.y * rotation.y,
        poissonSample.x * rotation.y + poissonSample.y * rotation.x
    );
}

float4x4 inverse(float4x4 m)
{
    float n11 = m[0][0], n12 = m[1][0], n13 = m[2][0], n14 = m[3][0];
    float n21 = m[0][1], n22 = m[1][1], n23 = m[2][1], n24 = m[3][1];
    float n31 = m[0][2], n32 = m[1][2], n33 = m[2][2], n34 = m[3][2];
    float n41 = m[0][3], n42 = m[1][3], n43 = m[2][3], n44 = m[3][3];

    float t11 = n23 * n34 * n42 - n24 * n33 * n42 + n24 * n32 * n43 - n22 * n34 * n43 - n23 * n32 * n44 + n22 * n33 * n44;
    float t12 = n14 * n33 * n42 - n13 * n34 * n42 - n14 * n32 * n43 + n12 * n34 * n43 + n13 * n32 * n44 - n12 * n33 * n44;
    float t13 = n13 * n24 * n42 - n14 * n23 * n42 + n14 * n22 * n43 - n12 * n24 * n43 - n13 * n22 * n44 + n12 * n23 * n44;
    float t14 = n14 * n23 * n32 - n13 * n24 * n32 - n14 * n22 * n33 + n12 * n24 * n33 + n13 * n22 * n34 - n12 * n23 * n34;

    float det = n11 * t11 + n21 * t12 + n31 * t13 + n41 * t14;
    float idet = 1.0f / det;

    float4x4 ret;

    ret[0][0] = t11 * idet;
    ret[0][1] = (n24 * n33 * n41 - n23 * n34 * n41 - n24 * n31 * n43 + n21 * n34 * n43 + n23 * n31 * n44 - n21 * n33 * n44) * idet;
    ret[0][2] = (n22 * n34 * n41 - n24 * n32 * n41 + n24 * n31 * n42 - n21 * n34 * n42 - n22 * n31 * n44 + n21 * n32 * n44) * idet;
    ret[0][3] = (n23 * n32 * n41 - n22 * n33 * n41 - n23 * n31 * n42 + n21 * n33 * n42 + n22 * n31 * n43 - n21 * n32 * n43) * idet;

    ret[1][0] = t12 * idet;
    ret[1][1] = (n13 * n34 * n41 - n14 * n33 * n41 + n14 * n31 * n43 - n11 * n34 * n43 - n13 * n31 * n44 + n11 * n33 * n44) * idet;
    ret[1][2] = (n14 * n32 * n41 - n12 * n34 * n41 - n14 * n31 * n42 + n11 * n34 * n42 + n12 * n31 * n44 - n11 * n32 * n44) * idet;
    ret[1][3] = (n12 * n33 * n41 - n13 * n32 * n41 + n13 * n31 * n42 - n11 * n33 * n42 - n12 * n31 * n43 + n11 * n32 * n43) * idet;

    ret[2][0] = t13 * idet;
    ret[2][1] = (n14 * n23 * n41 - n13 * n24 * n41 - n14 * n21 * n43 + n11 * n24 * n43 + n13 * n21 * n44 - n11 * n23 * n44) * idet;
    ret[2][2] = (n12 * n24 * n41 - n14 * n22 * n41 + n14 * n21 * n42 - n11 * n24 * n42 - n12 * n21 * n44 + n11 * n22 * n44) * idet;
    ret[2][3] = (n13 * n22 * n41 - n12 * n23 * n41 - n13 * n21 * n42 + n11 * n23 * n42 + n12 * n21 * n43 - n11 * n22 * n43) * idet;

    ret[3][0] = t14 * idet;
    ret[3][1] = (n13 * n24 * n31 - n14 * n23 * n31 + n14 * n21 * n33 - n11 * n24 * n33 - n13 * n21 * n34 + n11 * n23 * n34) * idet;
    ret[3][2] = (n14 * n22 * n31 - n12 * n24 * n31 - n14 * n21 * n32 + n11 * n24 * n32 + n12 * n21 * n34 - n11 * n22 * n34) * idet;
    ret[3][3] = (n12 * n23 * n31 - n13 * n22 * n31 + n13 * n21 * n32 - n11 * n23 * n32 - n12 * n21 * n33 + n11 * n22 * n33) * idet;

    return ret;
}


// Reference: https://developer.download.nvidia.com/whitepapers/2008/PCSS_Integration.pdf
// All calculation should be performed in light view space!
float shadowMapPCSS(BlockerSearchOutputs bso, float2 poissonOffset, SamplerComparisonState samplerCmp, SamplerState linearSampler, Texture2D shadowMap,
    Texture1D<float2> PCFRectSamples, float2 rotation, LightParams lightParams)
{
    // 1. get blocker search outputs
    float avgBlockerDepth = bso.projCoords.w;
    float3 shadingPosLightView = bso.shadingPosLightView.xyz;
    float3 projCoords = bso.projCoords.xyz;
    float zView = abs(shadingPosLightView.z);

    // 2. penumbra size
    float2 penumbraSize = _LIGHT_WORLD_SIZE * (zView - avgBlockerDepth) / avgBlockerDepth; // depth in light view space

    // Mapping penumbra size to the shadow map (divide by frustum size) using similar triangle to determine the filter size
    // The similar triangle takes sample point in area light as upper vertex and penumbra size as bottom edge
    float2 filterRadiusUV = float2(penumbraSize * lightParams.lightNearPlane / zView);
    filterRadiusUV /= lightParams.lightFrustumSize;

    print(lightParams.lightFrustumSize);
    print("penumbraSize = ", penumbraSize);
    print("filterRadiusUV = ", filterRadiusUV);

    // 3. PCF filtering
    float sum = 0.0f;
    for (int i = 0; i < _PCF_SAMPLES; i++)
    {
        // Poisson Rectangle
        float2 selectedSample = PCFRectSamples[i] - 0.5f;
        float2 offset = selectedSample * filterRadiusUV;

        // Hardware PCF
        sum += shadowMap.SampleCmpLevelZero(samplerCmp, projCoords.xy + offset, projCoords.z); // projCoords.z <= sample value ? for all nearest texel
    }

    return saturate(sum / _PCF_SAMPLES);
}

// New idea for PCSS: instead of using PCF, we just use the mapped light sample
float shadowMapNewPCSS(float3 lightDirView, BlockerSearchOutputs bso, SamplerComparisonState samplerCmp, Texture2D shadowMap,
    LightParams lightParams, float2 lightProjData)
{
    // 1. get data for mapping from blocker search outputs
    float3 projCoords = bso.projCoords.xyz;
    float3 shadingPosLightView = bso.shadingPosLightView.xyz;
    float avgBlockerDepth = bso.projCoords.w;
    float zReceiver = abs(shadingPosLightView.z);

    // 2. map samplePos to shadow map. (this part IS the bottleneck)
    float3 queryPosView = shadingPosLightView + lightDirView * (1.f - avgBlockerDepth / zReceiver);
    float2 samplePosProj = 0.5 * float2(-lightProjData.x, lightProjData.y) * queryPosView.xy / queryPosView.z + 0.5;

    //float3 shadingPosOnBlocker = (avgBlockerDepth / zReceiver) * shadingPosLightView; 
    //float2 samplePosView = mul(float4(samplePos, 1.0f), lightParams.lightView).xy; // samplePosView.z = 0
    //samplePosView = shadingPosOnBlocker.xy + samplePosView * (zReceiver - avgBlockerDepth) / zReceiver;
    //float4 samplePosH = mul(float4(samplePosView, shadingPosOnBlocker.z, 1.0f), lightParams.lightProj);
    //float3 samplePosNDC = samplePosH.xyz / samplePosH.w; // TODO: remove
    //samplePosNDC.y *= -1;
    //float3 samplePosProj = float3(samplePosNDC.xy * 0.5f + 0.5f, samplePosNDC.z); // normalized screen space

    // 3. calculate visibility
    float visibility = shadowMap.SampleCmpLevelZero(samplerCmp, samplePosProj.xy, projCoords.z); // already add depth bias
        
    return visibility;
}

void blockerSearch(inout BlockerSearchOutputs bso, float bias, float3 posW, float2 rotation, SamplerState sampler,
    Texture2D shadowMap, Texture1D<float2> blockSearchDiskSamples, LightParams lightParams)
{
    // Apply light transforms 
    float4 lightSpacePosH = mul(float4(posW, 1.0f), lightParams.lightSpaceMat); // clipping space
    float3 lightSpacePos = lightSpacePosH.xyz / lightSpacePosH.w; // NDC
    lightSpacePos.y *= -1; // DirectX flips the Y axis in viewport scale matrix
    float3 projCoords = float3(lightSpacePos.xy * 0.5f + 0.5f, lightSpacePos.z - bias); // NDC
    float3 shadingPosLightView = mul(float4(posW, 1.0f), lightParams.lightView).xyz;
    float zReceiver = abs(shadingPosLightView.z);

    // Blocker Search
    float2 searchSize = float2(_LIGHT_WORLD_SIZE * (zReceiver - lightParams.lightNearPlane) / zReceiver);
    searchSize /= lightParams.lightFrustumSize; // Measure the searchSize according to the frustum size and clamp search size into [0, 1]

    print(searchSize);

    float blockerSum = 0;
    uint numBlockers = 0;
    for (int i = 0; i < _BLOCKER_SEARCH_SAMPLES; i++)
    {
        float2 selectedSample = blockSearchDiskSamples[i] * 0.5f;
        float2 offset = rotatedPoisson(selectedSample, rotation) * searchSize;

        float shadowMapDepth = shadowMap.SampleLevel(sampler, projCoords.xy + offset, 0).r;
        if (shadowMapDepth < projCoords.z)
        {
            float linearDepth = abs(LinearizeDepth(shadowMapDepth, lightParams.lightNearPlane, lightParams.lightFarPlane));
            blockerSum += linearDepth;
            numBlockers++;
        }
    }
    float avgBlockerDepth = blockerSum / numBlockers;

    // Initialize structure
    bso.projCoords.xyz = projCoords;
    bso.shadingPosLightView.xyz = shadingPosLightView;
    bso.projCoords.w = avgBlockerDepth;
    bso.shadingPosLightView.w = numBlockers < 1 ? false : true;
}

//void blockerSearchNewPCSS(inout BlockerSearchOutputs bso, float bias, float3 posW, SampleGenerator sg, SamplerState sampler,
//    Texture2D shadowMap, LightParams lightParams, float2 lightProjData)
//{
//    float4 lightSpacePosH = mul(float4(posW, 1.0f), lightParams.lightSpaceMat); // clipping space
//    float3 lightSpacePos = lightSpacePosH.xyz / lightSpacePosH.w; // NDC
//    lightSpacePos.y *= -1; // DirectX flips the Y axis in viewport scale matrix
//    float3 projCoords = float3(lightSpacePos.xy * 0.5f + 0.5f, lightSpacePos.z - bias); // screen space
//    float3 shadingPosLightView = mul(float4(posW, 1.0f), lightParams.lightView).xyz;
//    float zReceiver = abs(shadingPosLightView.z);
//
//    // Blocker search
//    float blockerSum = 0;
//    uint numBlockers = 0;
//    for (int i = 0; i < _BLOCKER_SEARCH_SAMPLES; i++)
//    {
//        float3 randoms = sampleNext3D(sg);
//        uint triangleCount = gScene.lightCollection.triangleCount;
//        uint triangleIndex = min((uint)(randoms.x * triangleCount), triangleCount - 1);
//        const EmissiveTriangle tri = gScene.lightCollection.getTriangle(triangleIndex);
//        float3 samplePosW = tri.getPosition(sample_triangle(randoms.yz));
//        float3 lightDirView = mul(samplePosW - posW, float3x3(lightParams.lightView));
//
//        // Map light sample to near plane
//        float projectionPlane = lightParams.lightNearPlane + sampleNext1D(sg) * (zReceiver - lightParams.lightNearPlane);
//        float3 queryPosView = shadingPosLightView + lightDirView * (1.f - projectionPlane / zReceiver);
//        float2 samplePosProj = 0.5 * float2(-gLightProjData.x, gLightProjData.y) * queryPosView.xy / queryPosView.z + 0.5;
//
//        float shadowMapDepth = shadowMap.SampleLevel(sampler, samplePosProj.xy, 0).r;
//        if (shadowMapDepth < projCoords.z)
//        {
//            float linearDepth = abs(LinearizeDepth(shadowMapDepth, lightParams.lightNearPlane, lightParams.lightFarPlane));
//            blockerSum += linearDepth;
//            numBlockers++;
//        }
//    }
//    float avgBlockerDepth = blockerSum / numBlockers;
//
//    // Initialize structure
//    bso.projCoords.xyz = projCoords;
//    bso.shadingPosLightView.xyz = shadingPosLightView;
//    bso.projCoords.w = avgBlockerDepth;
//    bso.shadingPosLightView.w = numBlockers < 1 ? false : true;
//}

// Reference: Adaptive Depth Bias for Shadow Maps, Dou 2014
float adaptiveDepthBias(float3 normalW, float3 wsFragPos, uint shadowMapSize, Texture2D shadowMap, SamplerState sampler, LightParams lightParams)
{
    // Note: Actually the light rays can be precomputed.
    // The light ray can therefore be obtained by looking up shadow map
    // texel center, followed by a texture lookup.

    /** Define tangent plane **/
    // Light space frag normal
    //float4 normalLightView = mul(float4(normalW, 1.0f), inverse(transpose(lightParams.lightView)));
    float4 normalLightView = mul(float4(normalW, 1.0f), lightParams.lightView);
    float3 n = normalize(normalLightView.xyz);

    /** Locate shadow map texel center **/
    // Transform from eye-space to shadow map texture coordinates
    float4 lsFragPos = mul(float4(wsFragPos, 1.0f), lightParams.lightView);
    float4 smTexCoord = mul(lsFragPos, lightParams.lightProj);
    smTexCoord /= smTexCoord.w;
    smTexCoord.y *= -1; // DirectX flips the Y axis in viewport scale matrix
    smTexCoord = float4(0.5 * smTexCoord.xy, smTexCoord.z, 0.0f) + float4(0.5, 0.5, 0.0, 0.0);
    print(smTexCoord);

    // Locate corresponding light space shadow map grid center
    uint2 index = floor(float2(smTexCoord.xy * shadowMapSize));
    float texelSize = 1.0 / shadowMapSize;
    // Normalized coordinate in [0,1]
    float2 nlsGridCenter = texelSize * (index + float2(0.5)); // Normalized eye space grid center --- [0,1]
    // Unnormalized coordinate in [-lightLeft,lightLeft]
    float viewBound = 0.5f * lightParams.lightFrustumSize; // TODO: remove *0.5f ???
    float2 lsGridCenter = viewBound * (2.0 * nlsGridCenter - float2(1.0));

    /** Define light ray **/
    // Light ray direction in light space
    float nearPlane = lightParams.lightNearPlane;
    float farPlane = lightParams.lightFarPlane;
    float3 lsGridLineDir = normalize(float3(lsGridCenter, -nearPlane)); // Light space grid line direction

    /** Plane ray intersection **/
    // Locate the potential occluder for the shading fragment
    float ls_t_hit = dot(n, lsFragPos.xyz) / dot(n, lsGridLineDir);
    float3 ls_hit_p = ls_t_hit * lsGridLineDir;

    /** Compute Adaptive Epsilon **/
    // Normalized depth value in shadow map
    float SMDepth = shadowMap.SampleLevel(sampler, smTexCoord.xy, 0).r;
    // A and B are computed bnased on light near and far planes. 
    // They can be retrieved directly from light projection matrix
    float A = lightParams.lightProj[2][2];
    float B = lightParams.lightProj[3][2];

    // Change projection matrix calculation from DirectX to OpenGL
    A += nearPlane / (nearPlane - farPlane);
    B *= 2;

    // Constant part is (sceneScale * K)
    float adaptiveDepthBias = 0.5 * pow(1.0 - A - 2.0 * SMDepth, 2) * _CONSTANT_BIAS / B;

    // Use the intersection point as new look up point
    float4 lsPotentialoccluder = mul(float4(ls_hit_p, 1.0), lightParams.lightProj);
    lsPotentialoccluder = lsPotentialoccluder / lsPotentialoccluder.w;

    print(lsPotentialoccluder);
    print(SMDepth);

    // So we need to project this actual Depth
    float actualDepth = min(lsPotentialoccluder.z, smTexCoord.z);

    print(adaptiveDepthBias);
    
    return abs(adaptiveDepthBias);
}
