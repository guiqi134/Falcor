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
    bool gUsePairwiseMIS;

    SamplerComparisonState gSamplerCmp;
    SamplerState gLinearSampler;
    SamplerState gPointSampler;
}

RWStructuredBuffer<Reservoir> gReservoirs;
//StructuredBuffer<float2> gNeighborOffsetBuffer;
Texture1D<float2> gNeighborOffsetBuffer;

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

float3 computeTargetPdf(Reservoir sample, float4 projCoords, float4 shadingPosLightView, ShadingData sd, inout uint lightIndex)
{
    lightIndex = getReservoirLightIndex(sample);
    float3 samplePos = sample.posW;
    float3 posWOrigin = sd.computeNewRayOrigin();

    float3 toLight = samplePos - posWOrigin;
    float distSqr = max(FLT_MIN, dot(toLight, toLight));
    float distance = sqrt(distSqr);
    float3 dir = toLight / distance;
    float3 Le = gScene.getLight(lightIndex).intensity;

    float visibility;
#if _TARGET_PDF == 1 && _SHADOW_TYPE == 1
    BlockerSearchOutputs bso = { projCoords, shadingPosLightView };
    visibility = shadingPosLightView.w ? shadowMapNewPCSS(samplePos, bso, gSamplerCmp, gShadowMap, gLightParams) : 1.0f;
#elif _TARGET_PDF == 2
    RayDesc ray;
    ray.Origin = posWOrigin;
    ray.Direction = toLight / distance;
    ray.TMin = 0.001f;
    ray.TMax = distance - 0.001f;

    visibility = traceSceneVisibilityRay<1>(ray, RAY_FLAG_NONE, 0xff);
#endif
    
    return Le * gData.standardMaterial.eval(sd, dir) * visibility;
}

float pairwiseMisWeight(float q0, float q1, float N0, float N1)
{
    return (q1 == 0.f) ? 0.f : (N0 * q0) / (q0 * N0 + q1 * N1);
}

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
        uint validNeighbors = 1; //count self as a neighbor

        Reservoir state = emptyReservoir();

        // Get the light sample in current reservoir
        int selected = -1; // selected spatial neighbor iteration index
        uint selectedLight;

        if (!gUsePairwiseMIS)
        {
            if (isValidReservoir(centerSample))
            {
                selectedLight = getReservoirLightIndex(centerSample);
            }

            // Add current pixel's reservoir to the empty one
            combineReservoirs(state, centerSample, 0.5f, centerSample.targetPdf);
        }
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
        float centerMISWeight = 1.f;

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

            // compute pairwise MIS
            // shift center to neighbor

            // Combine neighbor reservoirs to center

            validNeighbors++;
            if (gUsePairwiseMIS)
            {
                uint dummy = -1;
                float pHat_centerSampleAtNeighbor = luminance(computeTargetPdf(centerSample, neighborSample.projCoords, neighborSample.shadingPosLightView, neighborSd, dummy));
                float pHat_centerSampleAtCenter = centerSample.targetPdf;
                centerMISWeight += 1.f - pairwiseMisWeight(pHat_centerSampleAtNeighbor, pHat_centerSampleAtCenter, neighborSample.M * numSpatialSamples, centerSample.M);
            }

            if (isValidReservoir(neighborSample))
            {
                // compute target pdf
                uint lightIndex = -1;
                float targetPdfCurrent = luminance(computeTargetPdf(neighborSample, centerSample.projCoords, centerSample.shadingPosLightView, sd, lightIndex));

                float neighborMISWeight = 1.f;

                if (gUsePairwiseMIS)
                {
                    neighborMISWeight = pairwiseMisWeight(neighborSample.targetPdf, targetPdfCurrent, neighborSample.M * numSpatialSamples, centerSample.M);
                }
                print(neighborMISWeight);
                if (combineReservoirs(state, neighborSample, sampleNext1D(sg), targetPdfCurrent, neighborMISWeight, gUsePairwiseMIS))
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

        if (gUsePairwiseMIS)
        {
            print(centerMISWeight);
            if (isValidReservoir(centerSample))
                combineReservoirs(state, centerSample, sampleNext1D(sg), centerSample.targetPdf, centerMISWeight, gUsePairwiseMIS);
            else
                state.M += centerSample.M;
        }

        // Compute the unbiased weight
        if (!gUsePairwiseMIS && isValidReservoir(state))
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

                p_y = selected == i ? targetPdfNeighbor : p_y; // selected neighbor sample
                p_sum += targetPdfNeighbor * neighborSample.M;
            }

            finalizeResampling(state, p_y, p_sum);
        }
        else
        {
            print(centerMISWeight / validNeighbors);
            print(1.f / state.M);
            print(state.weightSum / (gUsePairwiseMIS ? validNeighbors : state.M));
            finalizeResampling(state, 1.f, gUsePairwiseMIS ? validNeighbors : state.M);
        }

        spatialResult = state;
    }

    storeReservoir(gParams, spatialResult, gReservoirs, threadIndex, gOutputBufferIndex);
}
