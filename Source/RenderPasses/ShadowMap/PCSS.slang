#define _DEVICE_NEED_INCLUDE
#include "HostDeviceSharedDefinitions.h"

import Utils.Debug.PixelDebug;

__exported import Poisson;

// Utility functions
/*
Projection Matrix (row major):
xScale     0          0              0
0        yScale       0              0
0          0      zf/(zn-zf)        -1
0          0      zn*zf/(zn-zf)      0
*/
float LinearizeDepth(float depth)
{
    return (_LIGHT_NEAR_PLANE * _LIGHT_FAR_PLANE)
        / (_LIGHT_FAR_PLANE + depth * (_LIGHT_NEAR_PLANE - _LIGHT_NEAR_PLANE));
}

float2 rotatedPoisson(float2 poissonSample, float2 rotation)
{
    return float2(
        poissonSample.x * rotation.x - poissonSample.y * rotation.y,
        poissonSample.x * rotation.y + poissonSample.y * rotation.x
    );
}

// PCSS functions
float PCF(float3 projCoords, float2 filterRadiusUV, float2 poissonOffset, SamplerComparisonState samplerCmp,
    Texture2D shadowMap)
{
    float sum = 0.0f;

    for (int i = 0; i < _PCF_SAMPLES; i++)
    {
        float2 selectedSample = POISSON_SAMPLES_32[i];
        float2 offset = rotatedPoisson(selectedSample, poissonOffset) * filterRadiusUV;

        sum += shadowMap.SampleCmpLevelZero(samplerCmp, projCoords.xy + offset, projCoords.z); // projCoords.z < sample value ? for all nearest texel
    }

    return saturate(sum / _PCF_SAMPLES);
}

void findBlocker(out float avgBlockerDepth, out float numBlockers, float3 projCoords, float zReceiver, float2 poissonOffset,
    SamplerState sampler, Texture2D shadowMap)
{
    // This uses similar triangles to compute what area of the shadow map we should search (in light view space)
    float2 searchSize = float2(_LIGHT_WORLD_SIZE * (zReceiver - _LIGHT_NEAR_PLANE) / zReceiver);
    // Measure the searchSize according to the frustum size and clamp search size into [0, 1]
    searchSize /= _LIGHT_FRUSTUM_SIZE;

    float blockerSum = 0;
    numBlockers = 0;
    for (int i = 0; i < _BLOCKER_SEARCH_SAMPLES; i++)
    {
        float2 selectedSample = POISSON_SAMPLES_8[i];
        float2 offset = rotatedPoisson(selectedSample, poissonOffset) * searchSize;

        float shadowMapDepth = shadowMap.SampleLevel(sampler, projCoords.xy + offset, 0).r;
        if (shadowMapDepth < projCoords.z)
        {
            blockerSum += abs(LinearizeDepth(shadowMapDepth));
            numBlockers++;
        }
    }

    avgBlockerDepth = blockerSum / numBlockers;
}

// Reference: https://developer.download.nvidia.com/whitepapers/2008/PCSS_Integration.pdf
// All calculation should be performed in light view space!
float PCSS(float3 projCoords, float zReceiver, float2 poissonOffset, SamplerState sampler, SamplerComparisonState samplerCmp,
    Texture2D shadowMap)
{
    // 1. blocker search
    float avgBlockerDepth = 0;
    float numBlockers = 0;
    findBlocker(avgBlockerDepth, numBlockers, projCoords, zReceiver, poissonOffset, sampler, shadowMap);

    // no occluders -> no shadow
    if (numBlockers < 1) return 1.0f;

    // 2. penumbra size
    float penumbraSize = _LIGHT_WORLD_SIZE * (zReceiver - avgBlockerDepth) / avgBlockerDepth; // depth in light view space

    // Mapping penumbra size to the shadow map (divide by frustum size) using similar triangle to determine the filter size
    // The similar triangle takes sample point in area light as upper vertex and penumbra size as bottom edge
    float2 filterRadiusUV = float2(penumbraSize * _LIGHT_NEAR_PLANE / zReceiver);
    filterRadiusUV /= _LIGHT_FRUSTUM_SIZE;

    print("filterRadiusUV = ", filterRadiusUV);

    // 3. PCF filtering
    return PCF(projCoords, filterRadiusUV, poissonOffset, samplerCmp, shadowMap);
}


float newPCSS(float3 samplePos, float3 projCoords, float3 shadingPosLightView, float zReceiver, float2 poissonOffset,
    LightParams lightParams, SamplerState sampler, SamplerComparisonState samplerCmp, Texture2D shadowMap)
{
    // 1. blocker search
    float avgBlockerDepth = 0;
    float numBlockers = 0;
    findBlocker(avgBlockerDepth, numBlockers, projCoords, zReceiver, poissonOffset, sampler, shadowMap);

    // no occluders -> no shadow
    if (numBlockers < 1) return 1.0f;

    // 2. map samplePos to shadow map
    float3 shadingPosOnBlocker = (avgBlockerDepth / zReceiver) * shadingPosLightView;
    float2 samplePosView = mul(float4(samplePos, 1.0f), lightParams.lightView).xy; // samplePosView.z = 0
    samplePosView = shadingPosOnBlocker.xy + samplePosView * (zReceiver - avgBlockerDepth) / zReceiver;
    float4 samplePosH = mul(float4(samplePosView, shadingPosOnBlocker.z, 1.0f), lightParams.lightProj);
    float3 samplePosNDC = samplePosH.xyz / samplePosH.w;
    samplePosNDC.y *= -1;
    float3 samplePosProj = float3(samplePosNDC.xy * 0.5f + 0.5f, samplePosNDC.z); // screen space

    // 3. calculate visibility
    float visibility = shadowMap.SampleCmpLevelZero(samplerCmp, samplePosProj.xy, projCoords.z);

    return visibility;
}
