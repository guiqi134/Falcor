#include "Utils/Math/MathConstants.slangh"
import Utils.Sampling.SampleGenerator;
import Utils.Math.MathHelpers;
import Utils.Color.ColorHelpers;
import Scene.RaytracingInline;
import Scene.Scene;
import Scene.Shading;
import Experimental.Scene.Material.StandardMaterial;
import Experimental.Scene.Material.MaterialHelpers;
import Experimental.Scene.Lights.LightHelpers;
import Experimental.Scene.Lights.EmissiveLightSampler;
import Utils.Debug.PixelDebug;

import ShadingDataLoader;
import Helpers;
import PCSS;
import ReSTIR;


cbuffer CB
{
    uint2 gViewportDims;
    uint gFrameIndex;
    uint gOutputBufferIndex;
    ResamplingRuntimeParameters gParams;
    ShadingDataLoader gShadingDataLoader;
    LightParams gLightParams;

    SamplerComparisonState gSamplerCmp;
    SamplerState gLinearSampler;
    SamplerState gPointSampler;
    EmissiveLightSampler gEmissiveLightSampler;
}

RWStructuredBuffer<Reservoir> gReservoirs;
Texture2D<float4> gShadowMap;

static struct Data
{
    StandardMaterial standardMaterial;
} gData;

// Calculate random rotation and perform on poisson disk (angle in radians)
float2 randomRotation(float angle)
{
    return float2(cos(angle), sin(angle));
}

[numthreads(16, 16, 1)]
void main(uint2 threadIndex : SV_DispatchThreadID, uint3 groupIndex : SV_GROUPID)
{
    if (any(threadIndex >= gViewportDims))
    {
        return;
    }

    printSetPixel(threadIndex);

    SampleGenerator sg = SampleGenerator.create(threadIndex, gFrameIndex + 1 * 13);
    SampleGenerator sgFixed = SampleGenerator.create(threadIndex, 0);

    Reservoir r = emptyReservoir();

    // Prepare shading data from vBuffer
    ShadingData sd;
    bool valid = gShadingDataLoader.loadShadingData(threadIndex, gViewportDims, gScene.camera, sd);

    if (valid)
    {
        float3 posWOrigin = sd.computeNewRayOrigin();
        BlockerSearchOutputs bso;
            
#if _NEED_BLOCKER_SEARCH 
        print(111);
        // Compute average blocker depth (~0.5ms)
        float3 L = normalize(gLightParams.lightPos - posWOrigin);
        float NdotL = dot(sd.N, L);
        float bias = _DEPTH_BIAS * saturate(1.0 - NdotL) + _DEPTH_BIAS;
        blockerSearch(bso, bias, sd.posW, sampleNext2D(sgFixed) - 0.5f, gLinearSampler, gShadowMap, gLightParams); // translated poisson
        // blockerSearch(bso, NdotL, sd.posW, randomRotation(sampleNext1D(sg) * M_2PI), gLinearSampler, gShadowMap, gLightParams); // rotated poisson

        r.projCoords = bso.projCoords;
        r.shadingPosLightView = bso.shadingPosLightView;
#endif

        // Sampling analytic area light
        for (uint i = 0; i < _INITIAL_AREA_LIGHT_SAMPLES; i++)
        {
            const uint lightCount = gScene.getLightCount();
            if (lightCount == 0) break;

            AnalyticLightSample lightSample;
            uint lightIndex = min(uint(sampleNext1D(sg) * lightCount), lightCount - 1);
            if (sampleLight(posWOrigin, gScene.getLight(lightIndex), sg, lightSample) && dot(sd.N, lightSample.dir) >= kMinCosTheta)
            {
                float lightSelectionPdf = 1.0f / lightCount;
                float invPdf = 1.0f / (lightSample.pdf * lightSelectionPdf);
                float targetPdf = luminance(lightSample.Li * lightSample.pdf * gData.standardMaterial.eval(sd, lightSample.dir));

                // Add visibility to target pdf using PCSS -> more overhead
                float3 toLight = lightSample.posW - posWOrigin; // unnormalized
                float distance = sqrt(max(FLT_MIN, dot(toLight, toLight)));
                float visibility = 1.0f;

// _TARGET_PDF == 0 -> unshadowed 
#if _TARGET_PDF == 1 && _SHADOW_TYPE == 1 // New PCSS
                visibility = bso.shadingPosLightView.w ? shadowMapNewPCSS(lightSample.posW, bso, gSamplerCmp, gShadowMap, gLightParams) : 1.0f;
#elif _TARGET_PDF == 2 // Shadow Ray
                RayDesc ray;
                ray.Origin = posWOrigin;
                ray.Direction = toLight / distance;
                ray.TMin = 0.001f;
                ray.TMax = distance - 0.001f;

                visibility = traceSceneVisibilityRay<1>(ray, RAY_FLAG_NONE, 0xff);
#endif
                targetPdf *= visibility;
                streamSample(r, lightIndex, lightSample.posW, sampleNext1D(sg), targetPdf, invPdf);
            }
            else
            {
                r.M += 1;
            }
        }
        finalizeResampling(r, 1.0, r.M);

        // Pass area Light reservoir to current pixel reservoir
        // areaLightReservoir.M = 1; // Q: why do this? In order to pass weightSum to Reservoir r
        // combineReservoirs(r, areaLightReservoir, sampleNext1D(sg), areaLightReservoir.targetPdf);
        // finalizeResampling(r, 1.0, 1.0);
        // r.M = 1;

        // r = areaLightReservoir;

// Visibility reuse only need for when reusing enabled and target pdf is unshadowed
#if _NEED_VISIBILITY_REUSE
        if (isValidReservoir(r))
        {
            uint lightIndex = getReservoirLightIndex(r);
            float3 samplePos = r.posW;
            float3 toLight = samplePos - posWOrigin; // unnormalized
            float distSqr = max(FLT_MIN, dot(toLight, toLight));
            float distance = sqrt(distSqr);

            // Trace shadow ray or New PCSS for selected light sample
            float visibility = 1.0f;
#if _SHADOW_TYPE == 0
            RayDesc ray;
            ray.Origin = posWOrigin;
            ray.Direction = toLight / distance;
            ray.TMin = 0.001f;
            ray.TMax = distance - 0.001f;

            visibility = traceSceneVisibilityRay<1>(ray, RAY_FLAG_NONE, 0xff);
#elif _SHADOW_TYPE == 1
            visibility = bso.shadingPosLightView.w ? shadowMapNewPCSS(samplePos, bso, gSamplerCmp, gShadowMap, gLightParams) : 1.0f;
#endif
            if (!visibility)
            {
                r.weightSum = 0.0f;
                r.lightData = 0;
            }
        }
#endif
    }

    // print("r.W = ", r.weightSum);

    storeReservoir(gParams, r, gReservoirs, threadIndex, gOutputBufferIndex);
}
