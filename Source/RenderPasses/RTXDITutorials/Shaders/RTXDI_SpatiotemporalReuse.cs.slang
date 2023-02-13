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
    uint  gCurFrameInputReservoirIndex; // = uint(2)
    uint  gPrevFrameInputReservoirIndex;
    uint  gOutputReservoirIndex;
    uint  gMaxHistory;
    uint  gBiasCorrectionMode;     // 0 = 1/M (don't use), 1 = MIS w/o visibility, 2 = Pairwise MIS, 3 = MIS with extra visibility rays
    float gDepthThreshold;
    float gNormalThreshold;
    float gBoilingFilterStrength;  // in 0..1
    float gReuseRadius;
    uint  gNumSamples;
    uint  gSamplesInDisocclusions;
    bool  gUseVisibilityShortcut;
    bool gEnablePermutationSampling;
    uint gVisMode;
}

[numthreads(CURRENT_GROUP_SIZE, CURRENT_GROUP_SIZE, 1)]
void main(uint2 threadId : SV_DispatchThreadID, uint2 groupIndex : SV_GroupThreadID)
{
    // printSetPixel(threadId);

    // Find where we are on the screen and in the reservoir
    const uint2 launchIdx = threadId.xy;
    const RTXDI_ResamplingRuntimeParameters params = gRtxdiParams;
    uint2 pixelPosition = RTXDI_ReservoirToPixelPos(launchIdx, params);

    // Initialize our random number generator.
    RAB_RandomSamplerState rng = RAB_InitRandomSampler(pixelPosition, 2);

    // Grab the current pixel's surface from our G-bufer
    RAB_Surface surface = RAB_GetGBufferSurface(pixelPosition, false);

    // Create our reservoir temporary for accumulating spatial samples
    RTXDI_Reservoir restirResult = RTXDI_EmptyReservoir();
    RTXDI_Reservoir restirRankingResult = RTXDI_EmptyReservoir();

    if (RAB_IsSurfaceValid(surface))
    {
        // We want to combine our previously-computed light sample...
        RTXDI_Reservoir curSample = RTXDI_LoadReservoir(params, launchIdx, gCurFrameInputReservoirIndex);

        // ... with the results of our spatiotemporal resampling.  Set parameters here...
        RTXDI_SpatioTemporalResamplingParameters stparams;
        stparams.screenSpaceMotion = gMotionVectorTexture[pixelPosition].xyz * float3(gFrameSize, 1.0f); // motion vector is in [-1, 1]
        stparams.sourceBufferIndex = gPrevFrameInputReservoirIndex;
        stparams.maxHistoryLength = gMaxHistory;
        stparams.biasCorrectionMode = gBiasCorrectionMode;
        stparams.depthThreshold = gDepthThreshold;
        stparams.normalThreshold = gNormalThreshold;
        stparams.numSamples = gNumSamples + 1;
        stparams.numDisocclusionBoostSamples = gSamplesInDisocclusions;
        stparams.samplingRadius = gReuseRadius;
        stparams.enableVisibilityShortcut = gUseVisibilityShortcut;
        stparams.enablePermutationSampling = gEnablePermutationSampling;

        // ... then actually spawn the spatiotemporal resampling.
        RAB_LightSample selectedSample[2] = { (RAB_LightSample)0, (RAB_LightSample)0 };
        int2 selectedSamplePos = 0;

        if (gOnlyIsmForRanking)
        {
            RTXDI_Reservoir curSamples[2] = {
                curSample,
                RTXDI_LoadReservoir(params, launchIdx, gCurFrameInputReservoirIndex + RTXDI_RANKING_RESERVOIR_OFFSET)
            };
            restirResult = RTXDI_SpatioTemporalResampling(pixelPosition, surface, curSamples,
                rng, stparams, params, selectedSamplePos, selectedSample, restirRankingResult);
        }
        else
        {
            restirResult = RTXDI_SpatioTemporalResampling(pixelPosition, surface, curSample,
                rng, stparams, params, selectedSamplePos, selectedSample[0]);
        }
    }

    // If requested (and if we're in a compute shader), spawn a filter to surpress boiling
#ifdef RTXDI_ENABLE_BOILING_FILTER
    if (gBoilingFilterStrength > 0)
    {
        RTXDI_BoilingFilter(groupIndex, gBoilingFilterStrength, params, restirResult);
    }
#endif

    // Store the reservoir after spatiotemporal resampling to our output reservoir buffer
    RTXDI_StoreReservoir(restirResult, params, launchIdx, gOutputReservoirIndex);

    if (gOnlyIsmForRanking)
        RTXDI_StoreReservoir(restirRankingResult, params, launchIdx, gOutputReservoirIndex + RTXDI_RANKING_RESERVOIR_OFFSET);
}
