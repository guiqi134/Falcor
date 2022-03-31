#include "Utils/Math/MathConstants.slangh"
import Utils.Sampling.SampleGenerator;
import Utils.Math.MathHelpers;
import Utils.Color.ColorHelpers;
import Scene.RaytracingInline;
import Scene.Scene;
import Scene.Shading;
import Experimental.Scene.Material.StandardMaterial;
import Experimental.Scene.Material.MaterialHelpers;
import Utils.Debug.PixelDebug;

import PCSS;
import ReSTIR;
import ShadingDataLoader;
import Helpers;

cbuffer CB
{
    uint2 gViewportDims;
    uint gFrameIndex;
    uint gInputBufferIndex;
    uint gOutputBufferIndex;
    ResamplingRuntimeParameters gParams;
    ShadingDataLoader gShadingDataLoader;
    LightParams gLightParams;

    SamplerComparisonState gSamplerCmp;
    SamplerState gLinearSampler;
    SamplerState gPointSampler;
}

RWStructuredBuffer<Reservoir> gReservoirs;
Buffer<float2> gNeighborOffsetBuffer;

Texture2D<float4> gShadowMap;

static struct Data
{
    StandardMaterial standardMaterial;
} gData;

static const uint numSamples = 3; // number of spatial reuse samples
static const uint numDisocclusionBoostSamples = 4; // ???
static const uint targetHistoryLength = 20;
static const float samplingRadius = 30.0f;
static const float positionThreshold = 1.0f;
static const float normalThreshold = 0.5f;
static const uint neighborOffsetMask = 8191;

[numthreads(16, 16, 1)]
void main(uint2 threadIndex : SV_DispatchThreadID, uint3 groupIndex : SV_GROUPID)
{
    if (any(threadIndex >= gViewportDims))
    {
        return;
    }

    printSetPixel(threadIndex);

    SampleGenerator sg = SampleGenerator.create(threadIndex, gFrameIndex + 3 * 13);
    Reservoir spatialResult = emptyReservoir();
    ShadingData sd;
    bool valid = gShadingDataLoader.loadShadingData(threadIndex, gViewportDims, gScene.camera, sd);
    
    if (valid)
    {
        // RayOrigin or offseted posW
        float3 posWOrigin = sd.computeNewRayOrigin();

        // Current pixel's reservoir
        Reservoir centerSample = loadReservoir(gParams, gReservoirs, threadIndex, gInputBufferIndex);
        uint centerLightIndex = getReservoirLightIndex(centerSample); // Delete?

        Reservoir state = emptyReservoir();

        // Get the light sample in current reservoir
        int selected = -1; // selected spatial neighbor iteration index
        uint selectedLight;
        if (isValidReservoir(centerSample))
        {
            selectedLight = getReservoirLightIndex(centerSample);
        }

        // Add current pixel's reservoir to the empty one
        combineReservoirs(state, centerSample, 0.5f, centerSample.targetPdf);
        state.projCoords = centerSample.projCoords;
        state.shadingPosLightView = centerSample.shadingPosLightView;

        // Set the number of spatial samples to reuse
        int numSpatialSamples = numSamples;
        if (centerSample.M < targetHistoryLength)
        {
            numSpatialSamples = max(numSpatialSamples, numDisocclusionBoostSamples);
        }
        numSpatialSamples = min(numSpatialSamples, 32);

        uint startIdx = sampleNext1D(sg) * neighborOffsetMask; // [0, 8191)
        uint cachedResult = 0;
        for (uint i = 0; i < numSpatialSamples; i++)
        {
            // Random pick offset value from the offset buffer
            uint sampleIdx = (startIdx + i) & neighborOffsetMask;
            int2 spatialOffset = int2(float2(gNeighborOffsetBuffer[sampleIdx].xy) * samplingRadius); 
            int2 idx = threadIndex + spatialOffset;

            if (!isReprjTextureCoordValid(idx, gViewportDims))
            {
                continue;
            }

            ShadingData neighborSd;
            if (!gShadingDataLoader.loadShadingData(idx, gViewportDims, gScene.camera, neighborSd))
            {
                continue;
            }
            
            if (!isReprjNormalValid(sd.N, neighborSd.N, normalThreshold))
            {
                continue;
            }

            if (!isReprjPositionValid(sd.posW, neighborSd.posW, positionThreshold))
            {
                continue;
            }

            // Ensure neighbor has similar material info
            if (!areMaterialsSimilar(sd, neighborSd))
            {
                continue;
            }

            // Each iteration add 1 to the highest bit. After iteration, cachedResult = ... 0000 1111 = 15;
            cachedResult |= (1u << uint(i)); 
            Reservoir neighborSample = loadReservoir(gParams, gReservoirs, idx, gInputBufferIndex);

            // Combine neighbor reservoirs to center 
            if (isValidReservoir(neighborSample))
            {
                uint lightIndex = getReservoirLightIndex(neighborSample);
                float3 samplePos = neighborSample.posW;

                float3 toLight = samplePos - posWOrigin;
                float distSqr = max(FLT_MIN, dot(toLight, toLight));
                float distance = sqrt(distSqr);
                float3 dir = toLight / distance;
                float3 Le = gScene.getLight(lightIndex).intensity;

                float targetPdfCurrent = luminance(Le * gData.standardMaterial.eval(sd, dir));

                float visibility = 1.0f;
#if _TARGET_PDF == 1 && _SHADOW_TYPE == 1
                BlockerSearchOutputs bso = { state.projCoords, state.shadingPosLightView };
                visibility = state.shadingPosLightView.w ? shadowMapNewPCSS(samplePos, bso, gSamplerCmp, gShadowMap, gLightParams) : 1.0f;
#elif _TARGET_PDF == 2
                RayDesc ray;
                ray.Origin = posWOrigin;
                ray.Direction = toLight / distance;
                ray.TMin = 0.001f;
                ray.TMax = distance - 0.001f;

                visibility = traceSceneVisibilityRay<1>(ray, RAY_FLAG_NONE, 0xff);
#endif

                targetPdfCurrent *= visibility;
                if (combineReservoirs(state, neighborSample, sampleNext1D(sg), targetPdfCurrent))
                {
                    selected = i;
                    selectedLight = lightIndex;
                }
            }
            else
            {
                state.M += neighborSample.M;
            }
        }

        // Compute the unbiased weight 
        // if (false)
        if (isValidReservoir(state))
        {
            float p_y = state.targetPdf;
            float p_sum = state.targetPdf * centerSample.M;
            uint count = centerSample.M;

            for (uint i = 0; i < numSpatialSamples; i++)
            {
                if ((cachedResult & (1u << uint(i))) == 0) 
                {
                    continue;
                }
                uint sampleIdx = (startIdx + i) & neighborOffsetMask;
                int2 idx = threadIndex + int2(float2(gNeighborOffsetBuffer[sampleIdx].xy) * samplingRadius);
            
                ShadingData neighborSd;
                gShadingDataLoader.loadShadingData(idx, gViewportDims, gScene.camera, neighborSd);
                float3 neighborPosW = neighborSd.computeNewRayOrigin();

                // Target sample is fixed here
                uint lightIndex = selectedLight;
                float3 samplePos = state.posW;

                float3 toLight = samplePos - neighborPosW; // replace neighborPosW
                float distSqr = max(FLT_MIN, dot(toLight, toLight));
                float distance = sqrt(distSqr);
                float3 dir = toLight / distance;
                float3 Le = gScene.getLight(lightIndex).intensity;

                Reservoir neighborSample = loadReservoir(gParams, gReservoirs, idx, gInputBufferIndex);

                // Visibility test: trace shadow ray / shadow map
                float targetPdfNeighbor = 0.0f;
#if _SHADOW_TYPE == 0
                RayDesc ray;
                ray.Origin = neighborPosW;
                ray.Direction = toLight / distance;
                ray.TMin = 0.001f;
                ray.TMax = distance - 0.001f;
                
                if (traceSceneVisibilityRay<1>(ray, RAY_FLAG_NONE, 0xff))
                {
                    targetPdfNeighbor = luminance(Le * gData.standardMaterial.eval(neighborSd, dir));
                }
#elif _SHADOW_TYPE == 1
                BlockerSearchOutputs bso = { neighborSample.projCoords, neighborSample.shadingPosLightView };
                float visibility = neighborSample.shadingPosLightView.w ? shadowMapNewPCSS(samplePos, bso, gSamplerCmp, gShadowMap, gLightParams) : 1.0f;
                targetPdfNeighbor = luminance(Le * gData.standardMaterial.eval(neighborSd, dir) * visibility);
#endif
                print(p_y);
                print(targetPdfNeighbor);

                p_y = selected == i ? targetPdfNeighbor : p_y; // selected neighbor sample
                p_sum += targetPdfNeighbor * neighborSample.M;
            }

            finalizeResampling(state, p_y, p_sum);
        }
        else
        {
            finalizeResampling(state, 1.f, state.M);
        }

        spatialResult = state;

    }

    storeReservoir(gParams, spatialResult, gReservoirs, threadIndex, gOutputBufferIndex);
}
