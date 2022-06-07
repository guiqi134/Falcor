#include "Utils/Math/MathConstants.slangh"
import Utils.Sampling.SampleGenerator;
import Utils.Math.MathHelpers;
import Utils.Color.ColorHelpers;
import Scene.RaytracingInline;
import Scene.Scene;
import Scene.Shading;
import Scene.Material.MaterialFactory;
import Rendering.Materials.StandardMaterial;
import Utils.Debug.PixelDebug;

import ReSTIR;
import PCSS;
import ShadingDataLoader;
import Helpers;

cbuffer CB
{
    uint2 gViewportDims;
    uint gFrameIndex;
    uint gInputBufferIndex;
    uint gHistoryBufferIndex;
    uint gOutputBufferIndex;
    ResamplingRuntimeParameters gParams;
    ShadingDataLoader gShadingDataLoader;
    LightParams gLightParams;
    bool gUsePairwiseMIS;

    SamplerComparisonState gSamplerCmp;
    SamplerState gLinearSampler;
    SamplerState gPointSampler;
}

RWStructuredBuffer<Reservoir> gReservoirs; // current + previous
Texture2D<float4> gShadowMap;

static struct Data
{
    StandardMaterial standardMaterial;
} gData;

// Thresholds to reject neighbor reservoirs with substantially different geometry/material
static const int maxHistoryLength = 20; // ???
static const float positionThreshold = 0.1f;
static const float normalThreshold = 0.65f;

float3 computeTargetPdf(Reservoir sample, float4 projCoords, float4 shadingPosLightView, const ShadingData sd, inout uint lightIndex, const IBSDF bsdf, SampleGenerator sg)
{
    lightIndex = getReservoirLightIndex(sample);
    float3 samplePos = sample.posW;
    float3 posWOrigin = sd.computeNewRayOrigin();

    float3 toLight = samplePos - posWOrigin;
    float distSqr = max(FLT_MIN, dot(toLight, toLight));
    float distance = sqrt(distSqr);
    float3 dir = toLight / distance;
    float3 Le = gScene.getLight(lightIndex).intensity;

    float visibility = 1.0f;
#if _TARGET_PDF == 1 && _SHADOW_TYPE == 1
    BlockerSearchOutputs bso = { projCoords, shadingPosLightView };
    visibility = shadingPosLightView.w ? shadowMapNewPCSS(samplePos, bso, gSamplerCmp, gShadowMap, gLightParams) : 1.0f;
#elif _TARGET_PDF == 2 && RTX_GPU
    Ray ray = Ray(posWOrigin, toLight / distance, 0.001f, distance - 0.001f);
    visibility = traceSceneVisibilityRay<1>(ray, RAY_FLAG_NONE, 0xff);
#endif

    return Le * bsdf.eval(sd, dir, sg) * visibility;
}

[numthreads(16, 16, 1)]
void main(uint2 threadIndex : SV_DispatchThreadID, uint3 groupIndex : SV_GROUPID)
{
    if (any(threadIndex >= gViewportDims))
    {
        return;
    }

    printSetPixel(threadIndex);

    ShadingData sd;
    let lod = ExplicitLodTextureSampler(0.f);
    bool valid = gShadingDataLoader.loadShadingData(threadIndex, gViewportDims, gScene.camera, sd, lod);
    let bsdf = gScene.materials.getBSDF(sd, lod);

    SampleGenerator sg = SampleGenerator(threadIndex, gFrameIndex + 2 * 13);
    Reservoir temporalResult = emptyReservoir();

    if (valid)
    {
        float3 posWOrigin = sd.computeNewRayOrigin();

        // Bound the growth of M
        Reservoir curSample = loadReservoir(gParams, gReservoirs, threadIndex, gInputBufferIndex);
        int historyLimit = min(Reservoir::c_MaxM, maxHistoryLength * curSample.M);

        int selectedLightPrevID = -1;
        if (isValidReservoir(curSample))
        {
            selectedLightPrevID = getReservoirLightIndex(curSample);
        }

        // Always combine curSample to state if curSample is a vaild reservoir
        Reservoir state = emptyReservoir();

        if (!gUsePairwiseMIS)
            combineReservoirs(state, curSample, 0.5f, curSample.targetPdf);

        state.projCoords = curSample.projCoords;
        state.shadingPosLightView = curSample.shadingPosLightView;

        // Calculate previous frame pixel position using motion vector 
        float2 motion = gShadingDataLoader.motionVector[threadIndex];
        motion.xy += (sampleNext2D(sg) - 0.5);
        float2 reprojectedSamplePosition = float2(threadIndex) + motion.xy;
        int2 prevPos = int2(round(reprojectedSamplePosition));

        ShadingData neighborSd; // load from previous vBuffer
        bool foundNeighbor = false;
        const float radius = 0;
        
        // Continue to find reprojecting position using uniform random disk
        // The iteration tries to find a neighbor with similar geometry info
        for (int i = 0; i < 9; i++)
        {
            int2 offset = 0;
            if (i > 0)
            {
                offset.x = int((sampleNext1D(sg) - 0.5) * radius);
                offset.y = int((sampleNext1D(sg) - 0.5) * radius);
            }
            int2 idx = prevPos + offset;

            // Check whether the idx inside the screen
            if (!isReprjTextureCoordValid(idx, gViewportDims))
            {
                continue;
            }

            if (!gShadingDataLoader.loadShadingDataPrevFrame(idx, gViewportDims, gScene.camera, neighborSd, lod))
            {
                continue;
            }

            // Check current and previous normal difference 
            if (!isReprjNormalValid(sd.N, neighborSd.N, normalThreshold))
            {
                continue;
            }

            // Check current and previous position difference
            if (!isReprjPositionValid(sd.posW, neighborSd.posW, positionThreshold))
            {
                continue;
            }

            prevPos = idx;
            foundNeighbor = true;
            break;
        }

        bool selectedPreviousSample = false;

        // Calculate current target pdf using previous sample position
        Reservoir prevSample = loadReservoir(gParams, gReservoirs, prevPos, gHistoryBufferIndex);

        // Check current and previous shadow map depth difference
        float centerMISWeight = 1.f;

        if (foundNeighbor)
        {
            float neighborMISWeight = 1.f;

            prevSample.M = min(prevSample.M, historyLimit);
            uint originalPrevLightID = getReservoirLightIndex(prevSample);
            float targetPdfCurrent = 0.0f;

            if (gUsePairwiseMIS)
            {
                uint dummy = -1;
                let neighborBsdf = gScene.materials.getBSDF(neighborSd, lod);
                float pHat_centerSampleAtNeighbor = luminance(computeTargetPdf(curSample, prevSample.projCoords, prevSample.shadingPosLightView, neighborSd, dummy,
                    neighborBsdf, sg));
                float pHat_centerSampleAtCenter = curSample.targetPdf;
                centerMISWeight = pHat_centerSampleAtCenter == 0.f ? 0.f : float(pHat_centerSampleAtCenter) / (pHat_centerSampleAtCenter + pHat_centerSampleAtNeighbor * prevSample.M);
            }

            if (isValidReservoir(prevSample))
            {
                // compute target pdf
                targetPdfCurrent = luminance(computeTargetPdf(prevSample, curSample.projCoords, curSample.shadingPosLightView, sd, originalPrevLightID, bsdf, sg));
                if (gUsePairwiseMIS)
                {
                    neighborMISWeight = prevSample.targetPdf * prevSample.M / (targetPdfCurrent + prevSample.targetPdf * prevSample.M);
                }
            }

            bool sampleSelected = combineReservoirs(state, prevSample, sampleNext1D(sg), targetPdfCurrent, neighborMISWeight, gUsePairwiseMIS);
            if (sampleSelected)
            {
                selectedPreviousSample = true;
                selectedLightPrevID = int(originalPrevLightID);
            }
        }

        if (gUsePairwiseMIS)
        {
            combineReservoirs(state, curSample, sampleNext1D(sg), curSample.targetPdf, centerMISWeight, gUsePairwiseMIS);
        }

        // Debias part 
        if (!gUsePairwiseMIS && isValidReservoir(state))
        {
            float p_y = state.targetPdf;
            float p_sum = state.targetPdf * curSample.M;
            uint count = curSample.M;

            uint lightIndex = selectedLightPrevID;
            float3 samplePos = state.posW;
            float3 neighborPosW = neighborSd.computeNewRayOrigin();

            float3 toLight = samplePos - neighborPosW; // unnormalized
            float distSqr = max(FLT_MIN, dot(toLight, toLight));
            float distance = sqrt(distSqr);
            float3 dir = toLight / distance;
            float3 Le = gScene.getLight(lightIndex).intensity;

            // Visibility test: trace shadow ray / shadow map
            float targetPdfNeighbor = 0.0f;
#if _SHADOW_TYPE == 0 && RTX_GPU
            Ray ray = Ray(neighborPosW, toLight / distance, 0.001f, distance - 0.001f);
            
            if (traceSceneVisibilityRay<1>(ray, RAY_FLAG_NONE, 0xff))
            {
                targetPdfNeighbor = luminance(Le * gData.standardMaterial.eval(neighborSd, dir));
            }
#elif _SHADOW_TYPE == 1
            BlockerSearchOutputs bso = { prevSample.projCoords, prevSample.shadingPosLightView };
            float visibility = prevSample.shadingPosLightView.w ? shadowMapNewPCSS(samplePos, bso, gSamplerCmp, gShadowMap, gLightParams) : 1.0f;
            targetPdfNeighbor = luminance(Le * gData.standardMaterial.eval(neighborSd, dir) * visibility);
#endif
            if (targetPdfNeighbor > 0.0f)
            {
                count += prevSample.M;
            }

            p_sum += targetPdfNeighbor * prevSample.M;

            finalizeResampling(state, p_y, p_sum);
        }
        else
        {
            finalizeResampling(state, 1.0, gUsePairwiseMIS ? 1.f : state.M);
        }
        temporalResult = state;
    }

    storeReservoir(gParams, temporalResult, gReservoirs, threadIndex, gOutputBufferIndex);
}
