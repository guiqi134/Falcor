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

float LinearizeDepth(float depth)
{
    return (_LIGHT_NEAR_PLANE * _LIGHT_FAR_PLANE) /
        (_LIGHT_FAR_PLANE + depth * (_LIGHT_NEAR_PLANE - _LIGHT_FAR_PLANE));
}

float2 rotatedPoisson(float2 poissonSample, float2 rotation)
{
    return float2(
        poissonSample.x * rotation.x - poissonSample.y * rotation.y,
        poissonSample.x * rotation.y + poissonSample.y * rotation.x
    );
}

// Reference: https://developer.download.nvidia.com/whitepapers/2008/PCSS_Integration.pdf
// All calculation should be performed in light view space!
float shadowMapPCSS(BlockerSearchOutputs bso, float2 poissonOffset, SamplerComparisonState samplerCmp, SamplerState linearSampler, Texture2D shadowMap,
    Texture1D<float2> PCFRectSamples, float2 rotation)
{
    // 1. get blocker search outputs
    float avgBlockerDepth = bso.projCoords.w;
    float3 shadingPosLightView = bso.shadingPosLightView.xyz;
    float3 projCoords = bso.projCoords.xyz;
    float zView = abs(shadingPosLightView.z);

    // 2. penumbra size
    float penumbraSize = _LIGHT_WORLD_SIZE * (zView - avgBlockerDepth) / avgBlockerDepth; // depth in light view space

    // Mapping penumbra size to the shadow map (divide by frustum size) using similar triangle to determine the filter size
    // The similar triangle takes sample point in area light as upper vertex and penumbra size as bottom edge
    float2 filterRadiusUV = float2(penumbraSize * _LIGHT_NEAR_PLANE / zView);
    filterRadiusUV /= _LIGHT_FRUSTUM_SIZE;

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
float shadowMapNewPCSS(float3 samplePos, BlockerSearchOutputs bso, SamplerComparisonState samplerCmp, Texture2D shadowMap,
    LightParams lightParams)
{
    // 1. get data for mapping from blocker search outputs
    float3 projCoords = bso.projCoords.xyz;
    float3 shadingPosLightView = bso.shadingPosLightView.xyz;
    float avgBlockerDepth = bso.projCoords.w;
    float zReceiver = abs(shadingPosLightView.z);

    // 2. map samplePos to shadow map. (this part is not the bottleneck)
    float3 shadingPosOnBlocker = (avgBlockerDepth / zReceiver) * shadingPosLightView; 
    float2 samplePosView = mul(float4(samplePos, 1.0f), lightParams.lightView).xy; // samplePosView.z = 0
    samplePosView = shadingPosOnBlocker.xy + samplePosView * (zReceiver - avgBlockerDepth) / zReceiver; 
    float4 samplePosH = mul(float4(samplePosView, shadingPosOnBlocker.z, 1.0f), lightParams.lightProj);
    float3 samplePosNDC = samplePosH.xyz / samplePosH.w; // TODO: remove
    samplePosNDC.y *= -1;
    float3 samplePosProj = float3(samplePosNDC.xy * 0.5f + 0.5f, samplePosNDC.z); // normalized screen space

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
    lightSpacePos.y *= -1;
    float3 projCoords = float3(lightSpacePos.xy * 0.5f + 0.5f, lightSpacePos.z - bias); // screen space
    float3 shadingPosLightView = mul(float4(posW, 1.0f), lightParams.lightView).xyz;
    float zReceiver = abs(shadingPosLightView.z);

    // Blocker Search
    float2 searchSize = float2(_LIGHT_WORLD_SIZE * (zReceiver - _LIGHT_NEAR_PLANE) / zReceiver);
    searchSize /= _LIGHT_FRUSTUM_SIZE; // Measure the searchSize according to the frustum size and clamp search size into [0, 1]

    float blockerSum = 0;
    uint numBlockers = 0;
    for (int i = 0; i < _BLOCKER_SEARCH_SAMPLES; i++)
    {
        float2 selectedSample = blockSearchDiskSamples[i] * 0.5f;
        float2 offset = rotatedPoisson(selectedSample, rotation) * searchSize;

        float shadowMapDepth = shadowMap.SampleLevel(sampler, projCoords.xy + offset, 0).r;
        if (shadowMapDepth < projCoords.z)
        {
            float linearDepth = abs(LinearizeDepth(shadowMapDepth));
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

void blockerSearchNewPCSS(inout BlockerSearchOutputs bso, float bias, float3 posW, SampleGenerator sg, SamplerState sampler,
    Texture2D shadowMap, LightParams lightParams)
{
    float4 lightSpacePosH = mul(float4(posW, 1.0f), lightParams.lightSpaceMat); // clipping space
    float3 lightSpacePos = lightSpacePosH.xyz / lightSpacePosH.w; // NDC
    lightSpacePos.y *= -1;
    float3 projCoords = float3(lightSpacePos.xy * 0.5f + 0.5f, lightSpacePos.z - bias); // screen space
    float3 shadingPosLightView = mul(float4(posW, 1.0f), lightParams.lightView).xyz;
    float zReceiver = abs(shadingPosLightView.z);

    // Blocker search
    float blockerSum = 0;
    uint numBlockers = 0;
    for (int i = 0; i < _BLOCKER_SEARCH_SAMPLES; i++)
    {
        uint triangleCount = gScene.lightCollection.triangleCount;
        uint triangleIndex = min((uint)(sampleNext1D(sg) * triangleCount), triangleCount - 1);
        const EmissiveTriangle tri = gScene.lightCollection.getTriangle(triangleIndex);
        float2 u = sampleNext2D(sg);
        const float3 barycentrics = sample_triangle(u);
        float3 lightSamplePos = tri.getPosition(barycentrics);

        // Map light sample to near plane
        float3 shadingPosOnNearPlane = (_LIGHT_NEAR_PLANE / zReceiver) * shadingPosLightView;
        float2 samplePosView = mul(float4(lightSamplePos, 1.0f), lightParams.lightView).xy; // sample position on light 
        samplePosView = shadingPosOnNearPlane.xy + samplePosView * (zReceiver - _LIGHT_NEAR_PLANE) / zReceiver; // sample position on near plane
        float4 samplePosH = mul(float4(samplePosView, shadingPosOnNearPlane.z, 1.0f), lightParams.lightProj);
        float3 samplePosNDC = samplePosH.xyz / samplePosH.w; // TODO: remove
        samplePosNDC.y *= -1;
        float3 samplePosProj = float3(samplePosNDC.xy * 0.5f + 0.5f, samplePosNDC.z); // normalized screen space

        float shadowMapDepth = shadowMap.SampleLevel(sampler, samplePosProj.xy, 0).r;
        if (shadowMapDepth < projCoords.z)
        {
            float linearDepth = abs(LinearizeDepth(shadowMapDepth));
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
