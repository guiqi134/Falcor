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
    PCSS gPCSS;
}

RWStructuredBuffer<Reservoir> gReservoirs; // current + previous

static struct Data
{
    StandardMaterial standardMaterial;
} gData;

// Thresholds to reject neighbor reservoirs with substantially different geometry/material
static const int maxHistoryLength = 5; // 1 is ok 
static const float positionThreshold = 0.1f;
static const float normalThreshold = 0.65f;

[numthreads(16, 16, 1)]
void main(uint2 threadIndex : SV_DispatchThreadID, uint3 groupIndex : SV_GROUPID)
{
    if (any(threadIndex >= gViewportDims))
    {
        return;
    }

    printSetPixel(threadIndex);

    SampleGenerator sg = SampleGenerator.create(threadIndex, gFrameIndex + 2 * 13);
    Reservoir temporalResult = emptyReservoir();
    ShadingData sd;
    bool valid = gShadingDataLoader.loadShadingData(threadIndex, gViewportDims, gScene.camera, sd);

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

        Reservoir state = emptyReservoir();
        combineReservoirs(state, curSample, 0.5f, curSample.targetPdf); // p: 0.5 < 1, always combine
        state.projCoords = curSample.projCoords;
        state.shadingPosLightView = curSample.shadingPosLightView;

        // Calculate previous frame pixel position using motion vector 
        float2 motion = gShadingDataLoader.motionVector[threadIndex];
        motion.xy += (sampleNext2D(sg) - 0.5);
        float2 reprojectedSamplePosition = float2(threadIndex) + motion.xy;
        int2 prevPos = int2(round(reprojectedSamplePosition));

        ShadingData neighborSd; // load from previous vBuffer
        bool foundNeighbor = false;
        const float radius = 4;
        
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

            if (!gShadingDataLoader.loadShadingDataPrevFrame(idx, gViewportDims, gScene.camera, neighborSd))
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
        if (foundNeighbor)
        {
            prevSample.M = min(prevSample.M, historyLimit);
            uint originalPrevLightID = getReservoirLightIndex(prevSample);
            float targetPdfCurrent = 0.0f;
            if (isValidReservoir(prevSample))
            {
                uint lightIndex = originalPrevLightID;
                float3 samplePos = prevSample.posW;

                float3 toLight = samplePos - posWOrigin; // unnormalized
                float distSqr = max(FLT_MIN, dot(toLight, toLight));
                float distance = sqrt(distSqr);
                float3 dir = toLight / distance;
                float3 Le = gScene.getLight(lightIndex).intensity;

                targetPdfCurrent = luminance(Le * gData.standardMaterial.eval(sd, dir));

                float visibility = 1.0f;
#if _TARGET_PDF == 1
                BlockerSearchOutputs bso = { state.projCoords, state.shadingPosLightView };
                visibility = state.shadingPosLightView.w ? gPCSS.shadowMapPCSSModified(samplePos, bso) : 1.0f;
                visibility = max(_MIN_VISIBILITY, visibility);
#elif _TARGET_PDF == 2
                RayDesc ray;
                ray.Origin = posWOrigin;
                ray.Direction = toLight / distance;
                ray.TMin = 0.001f;
                ray.TMax = distance - 0.001f;

                visibility = traceSceneVisibilityRay<1>(ray, RAY_FLAG_NONE, 0xff);
#endif
                targetPdfCurrent *= visibility;
            }

            bool sampleSelected = combineReservoirs(state, prevSample, sampleNext1D(sg), targetPdfCurrent);
            if (sampleSelected)
            {
                selectedPreviousSample = true;
                selectedLightPrevID = int(originalPrevLightID);
            }
        }
        
        // Debias part 
        // if (false)
        if (isValidReservoir(state))
        {
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
#if _SHADING_MODE == 0
            RayDesc ray;
            ray.Origin = neighborPosW;
            ray.Direction = toLight / distance;
            ray.TMin = 0.001f;
            ray.TMax = distance - 0.001f;
            
            if (traceSceneVisibilityRay<1>(ray, RAY_FLAG_NONE, 0xff))
            {
                targetPdfNeighbor = luminance(Le * gData.standardMaterial.eval(neighborSd, dir));
            }
#elif _SHADING_MODE == 1
            BlockerSearchOutputs bso = { prevSample.projCoords, prevSample.shadingPosLightView };
            float visibility = prevSample.shadingPosLightView.w ? 
                gPCSS.shadowMapPCSSModified(samplePos, bso) : 1.0f;
            print("visibility = ", visibility);
            // visibility = max(_MIN_VISIBILITY, visibility);
            targetPdfNeighbor = luminance(Le * gData.standardMaterial.eval(neighborSd, dir)) *visibility;
#endif
            print("targetPdfNeighbor = ", targetPdfNeighbor);
            print("state.weightSum = ", state.weightSum);
            if (targetPdfNeighbor > 0.0f)
            {
                count += prevSample.M;
            }
            finalizeResampling(state, 1.0, count);
            print("prevSample.M = ", prevSample.M);
            print(count);
        }
        else
        {
            finalizeResampling(state, 1.0, state.M);
        }
        temporalResult = state;

        print("temporalResult.W = ", temporalResult.weightSum);
        print("temporalResult.M = ", temporalResult.M);
    }

    storeReservoir(gParams, temporalResult, gReservoirs, threadIndex, gOutputBufferIndex);
}
