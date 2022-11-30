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

#include "RtxdiBridge.slangh"

cbuffer ReuseCB
{
    uint  gInputReservoirIndex;
    uint  gOutputReservoirIndex;
    uint  gNumSamples;
    uint  gSamplesInDisocclusions;
    uint  gMaxHistory;
    uint  gBiasCorrectionMode;     // 0 = 1/M (don't use), 1 = MIS w/o visibility, 2 = Pairwise MIS, 3 = MIS with extra visibility rays
    float gReuseRadius;
    float gDepthThreshold;
    float gNormalThreshold;
}

[numthreads(16, 16, 1)]
void main(uint2 threadId : SV_DispatchThreadID)
{
    printSetPixel(threadId);
    
    // Find where we are on the screen and in the reservoir
    const uint2 launchIdx = threadId.xy;
    const RTXDI_ResamplingRuntimeParameters params = gRtxdiParams;
    uint2 pixelPosition = RTXDI_ReservoirToPixelPos(launchIdx, params);

    // Initialize our random number generator.  
    RAB_RandomSamplerState rng = RAB_InitRandomSampler(pixelPosition, 3);

    // Grab the current pixel's surface from our G-bufer
    RAB_Surface surface = RAB_GetGBufferSurface(pixelPosition, false);

    // Create our reservoir temporary for accumulating spatial samples
    RTXDI_Reservoir spatialResult = RTXDI_EmptyReservoir();

    // Actually execute the resampling on valid visibile geometry
    uint currentLightIndex = -1;
    if (RAB_IsSurfaceValid(surface))
    {
        // We want to combine our previously-computed light sample...
        RTXDI_Reservoir centerSample = RTXDI_LoadReservoir(params, launchIdx, gInputReservoirIndex);
        currentLightIndex = RTXDI_GetReservoirLightIndex(spatialResult);

        // ... with the results of our spatial resampling.  Set parameters here...
        RTXDI_SpatialResamplingParameters sparams;
        sparams.sourceBufferIndex = gInputReservoirIndex;
        sparams.numSamples = gNumSamples;
        sparams.numDisocclusionBoostSamples = gSamplesInDisocclusions;
        sparams.targetHistoryLength = gMaxHistory;
        sparams.biasCorrectionMode = gBiasCorrectionMode;
        sparams.samplingRadius = gReuseRadius;
        sparams.depthThreshold = gDepthThreshold;
        sparams.normalThreshold = gNormalThreshold;

        // ... then actually spawn the spatial resampling.
        RAB_LightSample selectedSample = (RAB_LightSample)0;
        spatialResult = RTXDI_SpatialResampling(pixelPosition, surface, centerSample,
            rng, sparams, params, selectedSample);
    }

    // Check if current pixel's sample has been replaced in reusing
    if (currentLightIndex != RTXDI_GetReservoirLightIndex(spatialResult)) 
        gHasSampleChangedInReusing[launchIdx] = float4(1.0f);

    // Store the reservoir after spatial resampling to our output reservoir buffer
    RTXDI_StoreReservoir(spatialResult, params, launchIdx, gOutputReservoirIndex);
}
