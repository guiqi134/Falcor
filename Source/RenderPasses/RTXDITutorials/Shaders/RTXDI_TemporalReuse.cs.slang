/***************************************************************************
 # Copyright (c) 2015-21, NVIDIA CORPORATION. All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions
 # are met:
 #  * Redistributions of source code must retain the above copyright
 #    notice, this list of conditions and the following disclaimer.
 #  * Redistributions in binary form must reproduce the above copyright
 #    notice, this list of conditions and the following disclaimer in the
 #    documentation and/or other materials provided with the distribution.
 #  * Neither the name of NVIDIA CORPORATION nor the names of its
 #    contributors may be used to endorse or promote products derived
 #    from this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS "AS IS" AND ANY
 # EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 # IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 # PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 # CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 # PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 # PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 # OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 # OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **************************************************************************/

 // Define the size of our thread-groups.  Do this since we need to pass RTXDI
 // this value (as a #define) to configure the (optional) boiling filter 
#define CURRENT_GROUP_SIZE 16

// Settings needed to use the boiling filter; define before #including the RTXDI shader
#define RTXDI_ENABLE_BOILING_FILTER
#define RTXDI_BOILING_FILTER_GROUP_SIZE CURRENT_GROUP_SIZE


#include "RtxdiBridge.slangh"


cbuffer ReuseCB
{
    uint  gCurFrameInputReservoirIndex;
    uint  gPrevFrameInputReservoirIndex;
    uint  gOutputReservoirIndex;
    uint  gMaxHistory;
    uint  gBiasCorrectionMode;     // 0 = 1/M (don't use), 1 = MIS w/o visibility, 2 = Pairwise MIS, 3 = MIS with extra visibility rays
    float gDepthThreshold;
    float gNormalThreshold;
    float gBoilingFilterStrength;  // in 0..1
    bool  gUseVisibilityShortcut;
    bool  gEnablePermutationSampling;
}


[numthreads(CURRENT_GROUP_SIZE, CURRENT_GROUP_SIZE, 1)]
void main(uint2 threadId : SV_DispatchThreadID, uint2 groupIndex : SV_GroupThreadID)
{
    printSetPixel(threadId);
    
    // Find where we are on the screen and in the reservoir
    const uint2 launchIdx = threadId.xy;
    const RTXDI_ResamplingRuntimeParameters params = gRtxdiParams;
    uint2 pixelPosition = RTXDI_ReservoirToPixelPos(launchIdx, params);

    // Initialize our random number generator.  
    RAB_RandomSamplerState rng = RAB_InitRandomSampler(pixelPosition, 2);

    // Grab the current pixel's surface from our G-bufer
    RAB_Surface surface = RAB_GetGBufferSurface(pixelPosition, false);

    // Create our reservoir temporary for accumulating spatial samples
    RTXDI_Reservoir temporalResult = RTXDI_EmptyReservoir();

    // Actually execute the resampling on valid visibile geometry
    uint currentLightIndex = -1;
    if (RAB_IsSurfaceValid(surface))
    {
        // We want to combine our previously-computed light sample...
        RTXDI_Reservoir curSample = RTXDI_LoadReservoir(params, launchIdx, gCurFrameInputReservoirIndex);
        currentLightIndex = RTXDI_GetReservoirLightIndex(curSample);

        // ... with the results of our temporal resampling.  Set parameters here...
        RTXDI_TemporalResamplingParameters tparams;
        tparams.screenSpaceMotion = gMotionVectorTexture[pixelPosition].xyz * float3(gFrameSize, 1.0f);
        tparams.sourceBufferIndex = gPrevFrameInputReservoirIndex; 
        tparams.maxHistoryLength = gMaxHistory;
        tparams.biasCorrectionMode = gBiasCorrectionMode;
        tparams.depthThreshold = gDepthThreshold;
        tparams.normalThreshold = gNormalThreshold;
        tparams.enableVisibilityShortcut = gUseVisibilityShortcut;      // false in this tutorial
        tparams.enablePermutationSampling = gEnablePermutationSampling; // false in this tutorial

        // ... then actually spawn the temporal resampling.
        RAB_LightSample selectedSample = (RAB_LightSample)0;
        int2            selectedSamplePos = 0;
        temporalResult = RTXDI_TemporalResampling(pixelPosition, surface, curSample,
            rng, tparams, params, selectedSamplePos, selectedSample);
    }

    // If requested (and if we're in a compute shader), spawn a filter to surpress boiling
#ifdef RTXDI_ENABLE_BOILING_FILTER
    if (gBoilingFilterStrength > 0)
    {
        RTXDI_BoilingFilter(groupIndex, gBoilingFilterStrength, params, temporalResult);
    }
#endif

    // Check if current pixel's sample has been replaced in reusing
    if (currentLightIndex != RTXDI_GetReservoirLightIndex(temporalResult)) 
        gHasSampleChangedInReusing[launchIdx] = float4(1.0f);

    // Store the reservoir after temporal resampling to our output reservoir buffer
    RTXDI_StoreReservoir(temporalResult, params, launchIdx, gOutputReservoirIndex);
}
