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

/** A shader used to shoot shadow rays at the sample in the reservoir,
    particularly useful for shooting visibility after generating initial candidates
*/

#include "RtxdiBridge.slangh"

cbuffer SampleCB
{
    uint gReservoirIndex; // Which of our reservoirs should we use for output?
    uint gVisMode;
}


[numthreads(16, 16, 1)]
void main(uint3 threadId : SV_DispatchThreadID)
{
    // printSetPixel(threadId.xy);

    print("Test Candidate Visibility pass----------------------------------", 1);

    print("gVisMode = ", (uint)gVisMode);

    // Find where we are on the screen and in the reservoir
    const uint2 launchIdx = threadId.xy;
    const RTXDI_ResamplingRuntimeParameters params = gRtxdiParams;
    uint2 pixelPosition = RTXDI_ReservoirToPixelPos(launchIdx, params);

    RAB_RandomSamplerState rng = RAB_InitRandomSampler(pixelPosition, 10);

    // Get data about the pixel we're shading from our G-buffer
    RAB_Surface surface = RAB_GetGBufferSurface(pixelPosition, false);

    // Get the current pixel's reservour.
    RTXDI_Reservoir reservoir = RTXDI_LoadReservoir(params, launchIdx, gReservoirIndex);
    RTXDI_Reservoir rankingReservoir = RTXDI_LoadReservoir(params, launchIdx, gReservoirIndex + RTXDI_RANKING_RESERVOIR_OFFSET);

    // Get the light sample
    uint lightIndex = RTXDI_GetReservoirLightIndex(reservoir); // triangle light + the rest
    float2 lightUV = RTXDI_GetReservoirSampleUV(reservoir);
    RAB_LightInfo lightInfo = RAB_LoadLightInfo(lightIndex, false);
    RAB_LightSample lightSample = RAB_SamplePolymorphicLight(lightInfo, surface, lightUV); // shading and ranking reservoir has the same sample

    // Shoot a shadow ray and, if it's shadowed, update the visibility
    if (RAB_IsSurfaceValid(surface) && RTXDI_IsValidReservoir(reservoir))
    {
        int localLightID = toLocalLightIndex(lightIndex, gTotalLightMeshCount, gRtxdiParams.firstPointLight, gRtxdiParams.environmentLightIndex, gFalcorLightIDtoOurs); // mesh light + point light
        uint bufferIndex = gScreenSize.x * launchIdx.y + launchIdx.x;

        if (gDebugLightID >= 0) localLightID = gDebugLightID;

        // Evaluate the visiblity for shading reservoir
        uint shadowOption;
        uint visMode = !gDisableRankingUpdate && gOnlyIsmForRanking && !gSplitReservoirs ? uint(Visibility::AllISM) : gVisMode;
        float isVisible = evaluateVisibility(bufferIndex, localLightID, surface, lightSample, visMode, rng, shadowOption);
        if (isVisible == 0.0f)
        {
            RTXDI_StoreVisibilityInReservoir(reservoir, 0, true);
            RTXDI_StoreReservoir(reservoir, params, launchIdx, gReservoirIndex);
        }

        print("visMode = ", visMode);
        print("shadowOption = ", shadowOption);

        // TODO: better implemtation for this parameter
        if (gUseAllShadowMapsForRanking)
        {
            print("evalute all shadow maps for ranking reservoir", 1);
            isVisible = evaluateVisibility(bufferIndex, localLightID, surface, lightSample, (uint)Visibility::ShadowMap_ISM, rng, shadowOption, true);
            if (isVisible == 0.0f)
            {
                RTXDI_StoreVisibilityInReservoir(rankingReservoir, 0, true);
                RTXDI_StoreReservoir(rankingReservoir, params, launchIdx, gReservoirIndex + RTXDI_RANKING_RESERVOIR_OFFSET);
            }
        }
        else
        {
            // Use pixels without visibility to rank light faces
            isVisible = gSortingRules == (uint)SortingRules::LightFaces_NoVisiblity ? 1.0f : isVisible;

            // If we are already using ISM to evaluate, then no need to do it again
            if (shadowOption == uint(ShadowOptions::ISM))
            {
                if (isVisible == 0.0f)
                {
                    RTXDI_StoreVisibilityInReservoir(rankingReservoir, 0, true);
                    RTXDI_StoreReservoir(rankingReservoir, params, launchIdx, gReservoirIndex + RTXDI_RANKING_RESERVOIR_OFFSET);
                }
            }
            else
            {
                // Also evalute the ISM for ranking reservoir, but we need to have:
                // 1. Ranking is been updated
                // 2. Light samples are not collected after RIS
                if (gVisMode != uint(Visibility::AllShadowRay) && gOnlyIsmForRanking && gSplitReservoirs
                    && !gDisableRankingUpdate
                    && gPlaceForRankingData == uint(PlacesForRankingData::AfterReusing))
                {
                    print("evalute the ISM for ranking reservoir", 1);
                    if (gSortingRules != (uint)SortingRules::LightFaces_NoVisiblity)
                        isVisible = evaluateVisibility(bufferIndex, localLightID, surface, lightSample, (uint)Visibility::AllISM, rng, shadowOption);
                    if (isVisible == 0.0f)
                    {
                        RTXDI_StoreVisibilityInReservoir(rankingReservoir, 0, true);
                        RTXDI_StoreReservoir(rankingReservoir, params, launchIdx, gReservoirIndex + RTXDI_RANKING_RESERVOIR_OFFSET);
                    }
                }
            }
        }

        print("isVisible = ", isVisible);

        // Store each pixel's light ID first for later face statistics computation
        if (gVisMode != uint(Visibility::AllShadowRay) && gPlaceForRankingData == uint(PlacesForRankingData::AfterVisibilityCheck) && !gDisableRankingUpdate)
        {
            if (!gSplitReservoirs) rankingReservoir = reservoir;
            storeLightSampleForRanking(rankingReservoir, params, true, launchIdx, isVisible);
        }
    }
}
