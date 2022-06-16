/***************************************************************************
 # Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
 #
 # NVIDIA CORPORATION and its licensors retain all intellectual property
 # and proprietary rights in and to this software, related documentation
 # and any modifications thereto.  Any use, reproduction, disclosure or
 # distribution of this software and related documentation without an express
 # license agreement from NVIDIA CORPORATION is strictly prohibited.
 **************************************************************************/

#include "RtxdiBridge.hlsli"
#include "InlineShadowRay.slangh"

#include "../rtxdi/ResamplingFunctions.hlsli"


[numthreads(4, 8, 1)]
void main(uint3 threadId : SV_DispatchThreadID)
{
    // Find where we are on the screen and int he reservoir
    const uint2 launchIdx = threadId.xy;
    const RTXDI_ResamplingRuntimeParameters params = gRtxdiParams;
    uint2 pixelPosition = RTXDI_ReservoirToPixelPos(launchIdx, params);

    RAB_RandomSamplerState rng = RAB_InitRandomSampler(pixelPosition, 1);
    //RAB_RandomSamplerState tileRng = RAB_InitRandomSampler(pixelPosition / RTXDI_TILE_SIZE_IN_PIXELS, 1);
    RAB_RandomSamplerState tileRng = RAB_InitRandomSampler(pixelPosition, 1);

    RAB_Surface surface = RAB_GetGBufferSurface(pixelPosition, false);

    RAB_LightSample lightSample;
    RTXDI_Reservoir reservoir = RTXDI_SampleLightsForSurface(rng, tileRng, surface,
        0,  // Number of samples to take from ReGIR? 
        32, // Number of samples to take from local (i.e., triangle) lights?
        0,  // Number of samples to take from infinite (i.e., directional) lights?
        0,  // Number of samples to take from the environment map?
        params, gRisBuffer, lightSample);

#if 0
    if (RTXDI_IsValidReservoir(reservoir))
    {
        // Trace a shadow ray to our selected light
        //float4 rayData = lightSample.getDirectionAndDistance();
        //RayDesc ray = { surface.worldPos, gEpsilon, rayData.xyz, rayData.w - gEpsilon };
        if (!RAB_GetConservativeVisibility(surface, lightSample))
        {
            RTXDI_StoreVisibilityInReservoir(reservoir, 0, true);
        }
    }
#endif

    RTXDI_StoreReservoir(reservoir, params, gLightReservoirs, launchIdx, 0 /*initialOutputBufferIndex*/ );
}
