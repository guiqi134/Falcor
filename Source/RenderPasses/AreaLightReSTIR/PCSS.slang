#include "Utils/Math/MathConstants.slangh"
#define _DEVICE_NEED_INCLUDE
#include "HostDeviceSharedDefinitions.h"

import Utils.Debug.PixelDebug;
import Utils.Sampling.SampleGenerator;
import PoissonDisk;

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

float shadowMapPCF(float3 projCoords, float2 filterRadiusUV, float2 poissonOffset, SamplerComparisonState samplerCmp,
    Texture2D shadowMap, float2 rotation)
{
    float sum = 0.0f;
    // float2 t = frac(projCoords.xy * gShadowMapDim + 0.5f); // Interpolation percentage in x, y
    //filterRadiusUV = min(filterRadiusUV, 0.03f);

    for (int i = 0; i < _PCF_SAMPLES; i++)
    {
        float2 selectedSample = POISSON_SAMPLES_256[i];
        float2 offset = rotatedPoisson(selectedSample, rotation) * filterRadiusUV; // rotated poisson
        offset = selectedSample * filterRadiusUV;

        //float2 offset = (POISSON_SAMPLES_256[i] * poissonOffset) * filterRadiusUV; // random translation poisson

        // Use hardware PCF
        sum += shadowMap.SampleCmpLevelZero(samplerCmp, projCoords.xy + offset, projCoords.z); // projCoords.z <= sample value ? for all nearest texel


        // Gathers the four texels: (-,+),(+,+),(+,-),(-,-)
        // float4 tmp = gShadowMap.Gather(gSampler, projCoords.xy + offset);
        // tmp.x = tmp.x < projCoords.z ? 0.0f : 1.0f;
        // tmp.y = tmp.y < projCoords.z ? 0.0f : 1.0f;
        // tmp.z = tmp.z < projCoords.z ? 0.0f : 1.0f;
        // tmp.w = tmp.w < projCoords.z ? 0.0f : 1.0f;
        // sum += lerp(lerp(tmp.w, tmp.z, t.x), lerp(tmp.x, tmp.y, t.x), t.y); // bilinear filter comparison results

        /*float shadowMapDepth = gShadowMap.SampleLevel(gSampler, projCoords.xy + offset, 0).r;
        sum += shadowMapDepth < projCoords.z ? 0.0f : 1.0f;*/
    }

    return saturate(sum / _PCF_SAMPLES);
}

void findBlocker(out float avgBlockerDepth, out uint numBlockers, float3 projCoords, float zReceiver, float2 poissonOffset,
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
        //float2 poissonSample = POISSON_SAMPLES_8[i];
        // float2 offset = float2(poissonSample.x * poissonOffset.x - poissonSample.y * poissonOffset.y, 
        //     poissonSample.x * poissonOffset.y + poissonSample.y * poissonOffset.x)
        //     * searchSize;
        float2 offset = (POISSON_SAMPLES_8[i] + poissonOffset) * searchSize; 
        float shadowMapDepth = shadowMap.SampleLevel(sampler, projCoords.xy + offset, 0).r;
        if (shadowMapDepth < projCoords.z)
        {
            float linearDepth = abs(LinearizeDepth(shadowMapDepth));
            blockerSum += linearDepth;
            numBlockers++;
        }
    }

    avgBlockerDepth = blockerSum / numBlockers;
}

// Reference: https://developer.download.nvidia.com/whitepapers/2008/PCSS_Integration.pdf
// All calculation should be performed in light view space!
float shadowMapPCSS(BlockerSearchOutputs bso, float2 poissonOffset, SamplerComparisonState samplerCmp, Texture2D shadowMap, float2 rotation)
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

    //filterRadiusUV = float2(0.001f);

    // 3. PCF filtering
    return shadowMapPCF(projCoords, filterRadiusUV, poissonOffset, samplerCmp, shadowMap, rotation);
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

    // 2. map samplePos to shadow map
    float3 shadingPosOnBlocker = (avgBlockerDepth / zReceiver) * shadingPosLightView; 
    float2 samplePosView = mul(float4(samplePos, 1.0f), lightParams.lightView).xy; // samplePosView.z = 0
    samplePosView = shadingPosOnBlocker.xy + samplePosView * (zReceiver - avgBlockerDepth) / zReceiver; 
    float4 samplePosH = mul(float4(samplePosView, shadingPosOnBlocker.z, 1.0f), lightParams.lightProj);
    float3 samplePosNDC = samplePosH.xyz / samplePosH.w;
    samplePosNDC.y *= -1;
    float3 samplePosProj = float3(samplePosNDC.xy * 0.5f + 0.5f, samplePosNDC.z); // normalized screen space

    // 3. calculate visibility
    float visibility = shadowMap.SampleCmpLevelZero(samplerCmp, samplePosProj.xy, projCoords.z); // already add depth bias
        
    return visibility;
}

// TODO: optimize this part
void blockerSearch(inout BlockerSearchOutputs bso, float bias, float3 posW, float2 poissonOffset, SamplerState sampler,
    Texture2D shadowMap, LightParams lightParams)
{
    float4 lightSpacePosH = mul(float4(posW, 1.0f), lightParams.lightSpaceMat); // clipping space
    float3 lightSpacePos = lightSpacePosH.xyz / lightSpacePosH.w; // NDC
    lightSpacePos.y *= -1;
    float3 projCoords = float3(lightSpacePos.xy * 0.5f + 0.5f, lightSpacePos.z - bias); // screen space
    float3 shadingPosLightView = mul(float4(posW, 1.0f), lightParams.lightView).xyz;
    float zReceiver = abs(shadingPosLightView.z);

    uint numBlockers;
    float avgBlockerDepth;
    findBlocker(avgBlockerDepth, numBlockers, projCoords, zReceiver, poissonOffset, sampler, shadowMap);

    // Initialize structure
    bso.projCoords.xyz = projCoords;
    bso.shadingPosLightView.xyz = shadingPosLightView;
    bso.projCoords.w = avgBlockerDepth;
    bso.shadingPosLightView.w = numBlockers < 1 ? false : true;
    // bso.poissonOffset = poissonOffset;
}

