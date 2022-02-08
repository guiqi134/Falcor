#include "Utils/Math/MathConstants.slangh"

import Utils.Debug.PixelDebug;
import Utils.Sampling.SampleGenerator;

import PoissonDisk;

// This per-pixel data should be passed between shaders
// struct BlockerSearchOutputs
// {
//     float3 projCoords;
//     float3 lightSpaceView;
//     float2 minMaxBlockerDepth;
//     float avgblockerDepth;
//     float zReceiver;
// }

struct PCSS
{
    float4x4 gLightSpaceMat;
    float4x4 gLightView;
    float4x4 gLightProj;
    float3 gLightPos; // light center world position
    float2 gShadowMapDim;
    float nearZ;
    float farZ;
    Texture2D<float4> gShadowMap;
    SamplerComparisonState gSamplerCmp;
    SamplerState gSampler;

    float LinearizeDepth(float depth)
    {
        return (nearZ * farZ) / (farZ + depth * (nearZ - farZ));
    }

    // Calculate random rotation and perform on poisson disk (angle in radians)
    float2x2 randomRotation(float angle)
    {
        return float2x2(
            cos(angle), -sin(angle),
            sin(angle), cos(angle)
        );
    }

    float shadowMapPCF(float3 projCoords, float2 filterRadiusUV, float2 poissonOffset)
    {
        float sum = 0.0f;
        float2 t = frac(projCoords.xy * gShadowMapDim + 0.5f); // Interpolation percentage in x, y

        for (int i = 0; i < PCF_SAMPLES; i++)
        {
            // Use hardware PCF
            float2 offset = (POISSON_SAMPLES_64[i] + poissonOffset) * filterRadiusUV; // Perform rotation

            //sum += gShadowMap.SampleCmpLevelZero(gSamplerCmp, projCoords.xy + offset, projCoords.z); // projCoords.z <= sample value ? for all nearest texel

            // Gathers the four texels: (-,+),(+,+),(+,-),(-,-)
            float4 tmp = gShadowMap.Gather(gSampler, projCoords.xy + offset);
            tmp.x = tmp.x < projCoords.z ? 0.0f : 1.0f;
            tmp.y = tmp.y < projCoords.z ? 0.0f : 1.0f;
            tmp.z = tmp.z < projCoords.z ? 0.0f : 1.0f;
            tmp.w = tmp.w < projCoords.z ? 0.0f : 1.0f;
            sum += lerp(lerp(tmp.w, tmp.z, t.x), lerp(tmp.x, tmp.y, t.x), t.y); // bilinear filter comparison results

            /*float shadowMapDepth = gShadowMap.SampleLevel(gSampler, projCoords.xy + offset, 0).r;
            sum += shadowMapDepth < projCoords.z ? 0.0f : 1.0f;*/
        }

        return saturate(sum / PCF_SAMPLES);
    }

    void findBlocker(out float avgBlockerDepth, out float2 minMaxBlockerDepth, out uint numBlockers, 
        float3 projCoords, float zReceiver, float2 poissonOffset)
    {
        // This uses similar triangles to compute what area of the shadow map we should search (in light view space)
        float2 searchSize = float2(LIGHT_WORLD_SIZE * (zReceiver - NEAR_PLANE) / zReceiver);
        // Measure the searchSize according to the frustum size and clamp search size into [0, 1]
        searchSize /= LIGHT_FRUSTUM_SIZE;

        float blockerSum = 0;
        minMaxBlockerDepth = float2(FLT_MAX, FLT_MIN);
        numBlockers = 0;
        for (int i = 0; i < _BLOCKER_SEARCH_SAMPLES; i++)
        {
            float2 offset = (POISSON_SAMPLES_8[i] + poissonOffset) * searchSize; 
            float shadowMapDepth = gShadowMap.SampleLevel(gSampler, projCoords.xy + offset, 0).r;
            if (shadowMapDepth < projCoords.z)
            {
                float linearDepth = abs(LinearizeDepth(shadowMapDepth));
                blockerSum += linearDepth;
                minMaxBlockerDepth.x = min(minMaxBlockerDepth.x, linearDepth);
                minMaxBlockerDepth.y = max(minMaxBlockerDepth.y, linearDepth);
                numBlockers++;
            }
        }

        avgBlockerDepth = blockerSum / numBlockers;
    }

    // Reference: https://developer.download.nvidia.com/whitepapers/2008/PCSS_Integration.pdf
    // All calculation should be performed in light view space!
    // float shadowMapPCSS(float3 posW, float3 N, float3 L, float2 poissonOffset)
    // {
    //     // Self-shadowing bias
    //     float bias = max(0.006 * (1.0 - dot(N, L)), 0.0005); // TODO: better bias selecting algorithm 

    //     float4 lightSpacePosH = mul(float4(posW, 1.0f), gLightSpaceMat);
    //     float3 lightSpacePos = lightSpacePosH.xyz / lightSpacePosH.w;
    //     lightSpacePos.y *= -1;
    //     float3 projCoords = float3(lightSpacePos.xy * 0.5f + 0.5f, lightSpacePos.z - bias); // screen space
    //     float zView = abs(mul(float4(posW, 1.0f), gLightView).z); // Depth is negative in -z direction, for length we need to use absolute value

    //     // 1. blocker search
    //     float avgBlockerDepth = 0;
    //     uint numBlockers = 0;
    //     findBlocker(avgBlockerDepth, numBlockers, projCoords, zView, poissonOffset);

    //     // no occluders -> no shadow
    //     if (numBlockers < 1) return 1.0f;

    //     // 2. penumbra size
    //     float penumbraSize = LIGHT_WORLD_SIZE * (zView - avgBlockerDepth) / avgBlockerDepth; // depth in light view space

    //     // Mapping penumbra size to the shadow map (divide by frustum size) using similar triangle to determine the filter size
    //     // The similar triangle takes sample point in area light as upper vertex and penumbra size as bottom edge
    //     float2 filterRadiusUV = float2(penumbraSize * NEAR_PLANE / zView);
    //     filterRadiusUV /= LIGHT_FRUSTUM_SIZE;

    //     print("penumbraSize = ", penumbraSize);
    //     print("filterRadiusUV = ", filterRadiusUV);

    //     // 3. PCF filtering
    //     return shadowMapPCF(projCoords, filterRadiusUV, poissonOffset);
    // }

    // New idea for PCSS: instead of using PCF, we just use the mapped light sample
    float shadowMapPCSSModified(float3 samplePos, float avgBlockerDepth, float3 projCoords, float zReceiver,
        float3 lightSpaceView)
    {
        // 1. blocker search
        // 2. map samplePos to shadow map
        float3 shadingPosOnBlocker = (avgBlockerDepth / zReceiver) * lightSpaceView; 
        float2 samplePosView = mul(float4(samplePos, 1.0f), gLightView).xy; // samplePosView.z = 0
        samplePosView = shadingPosOnBlocker.xy + samplePosView * (zReceiver - avgBlockerDepth) / zReceiver; 
        float4 samplePosH = mul(float4(samplePosView, shadingPosOnBlocker.z, 1.0f), gLightProj);
        float3 samplePosNDC = samplePosH.xyz / samplePosH.w;
        samplePosNDC.y *= -1;
        float3 samplePosProj = float3(samplePosNDC.xy * 0.5f + 0.5f, samplePosNDC.z); // screen space

        // 3. calculate visibility
        // TODO: change to hardware PCF
        // float2 t = frac(projCoords.xy * gShadowMapDim + 0.5f); // Interpolation percentage in x, y
        // float4 tmp = gShadowMap.Gather(gSampler, samplePosProj.xy);
        // tmp.x = tmp.x < projCoords.z ? 0.0f : 1.0f;
        // tmp.y = tmp.y < projCoords.z ? 0.0f : 1.0f;
        // tmp.z = tmp.z < projCoords.z ? 0.0f : 1.0f;
        // tmp.w = tmp.w < projCoords.z ? 0.0f : 1.0f;
        // float visibility = lerp(lerp(tmp.w, tmp.z, t.x), lerp(tmp.x, tmp.y, t.x), t.y); // bilinear filter comparison results

        // The main overhead compared with blocker search 
        float visibility = gShadowMap.SampleCmpLevelZero(gSamplerCmp, samplePosProj.xy, projCoords.z); 
        
        // visibility = max(_MIN_VISIBILITY, visibility);

        return visibility;
    }

    bool blockerSearch(out float avgBlockerDepth, out float2 minMaxBlockerDepth, out float3 projCoords, out float zReceiver,
        out float3 lightSpaceView, float NdotL, float3 posW, float2 poissonOffset)
    {
        // float bias = _DEPTH_BIAS < 0.0f ? max(0.01 * (1.0 - NdotL), 0.001) : _DEPTH_BIAS;
        float bias = _DEPTH_BIAS < 0.0f ? max(0.006 * (1.0 - NdotL), 0.0005) : _DEPTH_BIAS;

        float4 lightSpacePosH = mul(float4(posW, 1.0f), gLightSpaceMat);
        float3 lightSpacePos = lightSpacePosH.xyz / lightSpacePosH.w; // NDC
        lightSpacePos.y *= -1;
        projCoords = float3(lightSpacePos.xy * 0.5f + 0.5f, lightSpacePos.z - bias); // screen space
        lightSpaceView = mul(float4(posW, 1.0f), gLightView).xyz;
        zReceiver = abs(lightSpaceView.z);

        uint numBlockers;
        findBlocker(avgBlockerDepth, minMaxBlockerDepth, numBlockers, projCoords, zReceiver, poissonOffset);

        return numBlockers < 1 ? false : true;
    }
};

