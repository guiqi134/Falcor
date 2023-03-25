#include "Utils/Math/MathConstants.slangh"

import Utils.Debug.PixelDebug;
import ISMHelpers;

#ifndef PSM_COUNT
#define PSM_COUNT 40
#endif

#ifndef HIGH_RES_ISM_COUNT
#define HIGH_RES_ISM_COUNT 400
#endif

#ifndef DEBUG_PRINT_MODE
#define DEBUG_PRINT_MODE 0
#endif

cbuffer CB
{
    uint gTotalLightFaceCount;
    uint gFirstSpotLightID;
    uint gMaxIsmPerArray;
    uint gLowIsmSize;
    uint gHighIsmSize;
    uint gIterationCount;
    uint gTemporalReusingLength;
    float gConfidenceScaler;
    bool gCompareToBoundary;
    uint gMaxSupportedShadowMaps1024;
    uint gMaxPsmPerArray;
    bool gUseHighResISM;

    uint gVisibility;
    uint gSortingRules;
    uint gFlickerReduction;
}

Buffer<uint2> gSortedLightsBuffer; // only need top N light/face index
Buffer<float> gBoundaryVarianceBuffer;

RWBuffer<uint> gReusingLightIndexBuffer;
RWBuffer<uint> gHighResIsmFaceIdxBuffer;
RWStructuredBuffer<LightShadowMapData> gLightShadowDataBuffer;
RWBuffer<uint> gTotalValidPixels;

RWBuffer<uint> gShadowMapSizeBuffer;
RWBuffer<float4> gRecordReusingLightData;

bool flickerReduction(const LightFaceData newLightFaceData, const LightFaceData oldLightFaceData, const uint2 boundarySize, const uint boundary)
{
    bool canChange = true;
    if (gFlickerReduction == (uint)FlickerReduction::RegionRestricted)
    {
        uint prevRanking = newLightFaceData.prevRanking;
        uint currRanking = newLightFaceData.currRanking;

        uint3 limitedRegion = uint3(boundary - boundary.x, boundary, boundary + boundarySize.y);
        canChange = !((prevRanking >= limitedRegion.x && prevRanking < limitedRegion.z) && (currRanking >= limitedRegion.x && currRanking < limitedRegion.z));
    }
    else if (gFlickerReduction == (uint)FlickerReduction::VarianceCheck && gIterationCount > 0)
    {
        // Get the PSM boundary light face frequency
        float xNew = newLightFaceData.frequency;
        float lowerBoundNew = xNew - gConfidenceScaler * sqrt(newLightFaceData.varianceUnbiased);
        float xOld = oldLightFaceData.frequency;
        float upperBoundOld = xOld + gConfidenceScaler * sqrt(oldLightFaceData.varianceUnbiased);

        // Check if there is an intersection between the range of old and new only using stdDev
        if (!gCompareToBoundary)
        {
            if (lowerBoundNew > upperBoundOld)
                canChange = true;
            else
                canChange = false;
        }
        // Idea 2: Check if the boundary value will be inside the new one's current confidence range
        else
        {
            float boundaryFreq = gSortedLightsBuffer[PSM_COUNT].y;
            float boundaryStdDev = sqrt(gBoundaryVarianceBuffer[1]);
            print("boundaryFreq = ", boundaryFreq);
            print("boundaryVar = ", boundaryStdDev);

            if (lowerBoundNew >= boundaryFreq + gConfidenceScaler * boundaryStdDev)
                canChange = true;
            else
                canChange = false;
        }

        print("lowerBoundNew = ", lowerBoundNew);
        print("upperBoundOld = ", upperBoundOld);

        print("newLightFaceData x = ", xNew);
        print("newLightFaceData stdDev = ", sqrt(newLightFaceData.varianceUnbiased));
        print("oldLightFaceData x = ", xOld);
        print("oldLightFaceData stdDev = ", sqrt(oldLightFaceData.varianceUnbiased));

        print("canChange = ", canChange);
    }

    return canChange;
}

[numthreads(1, 1, 1)]
void main(uint3 threadID: SV_DispatchThreadID)
{
    if (threadID.x == 0) return;

    printSetPixel(threadID.xy);

    uint rank = 0;

    // Initialize the array
    bool highIsmArrayValidMask[HIGH_RES_ISM_COUNT];
    for (uint i = 0; i < HIGH_RES_ISM_COUNT; i++)
        highIsmArrayValidMask[i] = true;

    // Process PSM section in the ranking
    if (gVisibility != (uint)Visibility::AllISM)
    {
        bool psmArrayValidMask[PSM_COUNT];
        uint3 newPsmFaces[PSM_COUNT];
        uint currNewPsmIdx = 0u;

        print("PSM_COUNT = ", PSM_COUNT);

        // Initialize the array
        for (uint i = 0; i < PSM_COUNT; i++)
            psmArrayValidMask[i] = true;

        for (; rank < PSM_COUNT; rank++)
        {
            uint globalFaceID = gSortedLightsBuffer[rank].x;
            uint2 localLightIDAndFaceIdx = getLocalLightIdAndFaceIdx(globalFaceID, gFirstSpotLightID);
            uint localLightID = localLightIDAndFaceIdx.x;
            uint faceIdx = localLightIDAndFaceIdx.y;
            LightFaceData lightFaceData = gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx];

            // Rank as a potiential PSM light face
            if (lightFaceData.shadowMapType != (uint)ShadowMapType::PSM)
            {
                // This is a new PSM light face
                newPsmFaces[currNewPsmIdx] = uint3(globalFaceID, localLightID, faceIdx);
                currNewPsmIdx++;
            }
            else
            {
                uint validMaskIdx = lightFaceData.psmTexArrayIdx;
                psmArrayValidMask[validMaskIdx] = false;
            }

            // ISM array index should be updated for every face
            if (gVisibility != (uint)Visibility::ShadowMap_FullyLit)
            {
                int globalIsmTexArrayIdx = int(rank);
                gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx].ismTexArrayIdx = globalIsmTexArrayIdx % (int)gMaxIsmPerArray;
                gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx].whichIsmTexArray = uint(globalIsmTexArrayIdx / gMaxIsmPerArray);
            }
        }

        print("currNewPsmIdx = ", currNewPsmIdx);
        print("gMaxIsmPerArray = ", gMaxIsmPerArray);

        // See if the new PSM face can be changed
        currNewPsmIdx = 0u;
        for (uint arrayIdx = 0; arrayIdx < PSM_COUNT; arrayIdx++)
        {
            if (psmArrayValidMask[arrayIdx])
            {
                // Get new and old light face
                uint3 newPsmLightFaceIDs = newPsmFaces[currNewPsmIdx];
                uint localLightID = newPsmLightFaceIDs.y;
                uint faceIdx = newPsmLightFaceIDs.z;
                LightFaceData newLightFaceData = gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx];

                uint oldPsmLightFaceID = gReusingLightIndexBuffer[arrayIdx];
                uint2 oldLocalLightIDAndFaceIdx = oldPsmLightFaceID != UINT32_MAX ? getLocalLightIdAndFaceIdx(oldPsmLightFaceID, gFirstSpotLightID) : uint2(0);
                uint oldLocalLightID = oldLocalLightIDAndFaceIdx.x;
                uint oldFaceIdx = oldLocalLightIDAndFaceIdx.y;
                LightFaceData oldLightFaceData = gLightShadowDataBuffer[oldLocalLightID].lightFaceData[oldFaceIdx];

                if (oldPsmLightFaceID == UINT32_MAX || flickerReduction(newLightFaceData, oldLightFaceData, uint2(PSM_COUNT * 0.5f), PSM_COUNT))
                {
                    // Only update the index for new PSM light face.
                    gReusingLightIndexBuffer[arrayIdx] = newPsmLightFaceIDs.x;

                    int globalPsmTexArrayIdx = (int)arrayIdx;
                    gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx].psmTexArrayIdx = globalPsmTexArrayIdx < gMaxSupportedShadowMaps1024 ? globalPsmTexArrayIdx :
                    int((globalPsmTexArrayIdx - gMaxSupportedShadowMaps1024) % gMaxPsmPerArray);
                    gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx].whichPsmTexArray = globalPsmTexArrayIdx < gMaxSupportedShadowMaps1024 ? 0u :
                    int((globalPsmTexArrayIdx - gMaxSupportedShadowMaps1024) / gMaxPsmPerArray + 1u);

                    // Update other variables of new PSM light face (Only for light face)
                    gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx].shadowMapType = uint(ShadowMapType::PSM);
                    gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx].shadowMapSize = arrayIdx < gMaxSupportedShadowMaps1024 ? 1024 : 512;

                    // Clear the shadow map type of old light face
                    if (oldPsmLightFaceID != UINT32_MAX)
                    {
                        gLightShadowDataBuffer[oldLocalLightID].lightFaceData[oldFaceIdx].shadowMapType = uint(ShadowMapType::None);
                        gLightShadowDataBuffer[oldLocalLightID].lightFaceData[oldFaceIdx].shadowMapSize = 0u;
                    }
                }
                else
                {
                    // If the new face is high-res ISM and now it is rejected, it also should not change when processing the high ISM section
                    if (newLightFaceData.shadowMapType == uint(ShadowMapType::HighIsm))
                    {
                        print("newLightFaceData.highResIsmTexArrayIdx = ", newLightFaceData.highResIsmTexArrayIdx);
                        highIsmArrayValidMask[newLightFaceData.highResIsmTexArrayIdx] = false;
                    }
                }

                currNewPsmIdx++;
            }
        }

    }

    print("rank = ", rank);

    // Process High-res ISM section in the ranking
    if (gUseHighResISM && gVisibility != (uint)Visibility::AllISM && gVisibility != (uint)Visibility::ShadowMap_FullyLit)
    {
        uint3 newHighIsmFaces[HIGH_RES_ISM_COUNT];
        uint currNewHighIsmIdx = 0u;

        print("HIGH_RES_ISM_COUNT = ", HIGH_RES_ISM_COUNT);

        for (; rank < PSM_COUNT + HIGH_RES_ISM_COUNT; rank++)
        {
            uint globalFaceID = gSortedLightsBuffer[rank].x;
            uint2 localLightIDAndFaceIdx = getLocalLightIdAndFaceIdx(globalFaceID, gFirstSpotLightID);
            uint localLightID = localLightIDAndFaceIdx.x;
            uint faceIdx = localLightIDAndFaceIdx.y;
            LightFaceData lightFaceData = gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx];

            // Rank as a potiential High-res ISM light face
            if (lightFaceData.shadowMapType != uint(ShadowMapType::HighIsm))
            {
                // PSM light cannot be changed
                if (lightFaceData.shadowMapType != uint(ShadowMapType::PSM))
                {
                    newHighIsmFaces[currNewHighIsmIdx] = uint3(globalFaceID, localLightID, faceIdx);
                    currNewHighIsmIdx++;
                }
            }
            else
            {
                uint validMaskIdx = lightFaceData.highResIsmTexArrayIdx;
                highIsmArrayValidMask[validMaskIdx] = false;
            }

            // ISM array index should be updated for every face
            if (gVisibility != (uint)Visibility::ShadowMap_FullyLit)
            {
                int globalIsmTexArrayIdx = int(rank);
                gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx].ismTexArrayIdx = globalIsmTexArrayIdx % (int)gMaxIsmPerArray;
                gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx].whichIsmTexArray = uint(globalIsmTexArrayIdx / gMaxIsmPerArray);
            }
        }

        print("currNewHighIsmIdx = ", currNewHighIsmIdx);

        // See if the new high-res ISM face can be changed
        currNewHighIsmIdx = 0u;
        for (uint arrayIdx = 0; arrayIdx < HIGH_RES_ISM_COUNT; arrayIdx++)
        {
            if (highIsmArrayValidMask[arrayIdx])
            {
                // Get new and old light face
                uint3 newLightFaceIDs = newHighIsmFaces[currNewHighIsmIdx];
                uint localLightID = newLightFaceIDs.y;
                uint faceIdx = newLightFaceIDs.z;
                LightFaceData newLightFaceData = gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx];

                print("newLightFaceIDs.x = ", newLightFaceIDs.x);
                print("newLightFaceData.shadowMapType = ", newLightFaceData.shadowMapType);
                // print("newLightFaceIDs.highResIsmTexArrayIdx = ", oldLightFaceData.highResIsmTexArrayIdx);

                uint oldLightFaceID = gHighResIsmFaceIdxBuffer[arrayIdx];
                uint2 oldLocalLightIDAndFaceIdx = oldLightFaceID != UINT32_MAX ? getLocalLightIdAndFaceIdx(oldLightFaceID, gFirstSpotLightID) : uint2(0);
                uint oldLocalLightID = oldLocalLightIDAndFaceIdx.x;
                uint oldFaceIdx = oldLocalLightIDAndFaceIdx.y;
                LightFaceData oldLightFaceData = gLightShadowDataBuffer[oldLocalLightID].lightFaceData[oldFaceIdx];

                // If the old face becomes a PSM face, then the change must happen
                bool canChange = oldLightFaceID == UINT32_MAX || oldLightFaceData.shadowMapType == uint(ShadowMapType::PSM) ||
                                 flickerReduction(newLightFaceData, oldLightFaceData, uint2(HIGH_RES_ISM_COUNT * 0.25f), HIGH_RES_ISM_COUNT);

                print("canChange = ", canChange);

                if (canChange)
                {
                    gHighResIsmFaceIdxBuffer[arrayIdx] = newLightFaceIDs.x;
                    gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx].highResIsmTexArrayIdx = int(arrayIdx % gMaxIsmPerArray);
                    gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx].whichHighResIsmTexArray = arrayIdx / gMaxIsmPerArray;

                    gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx].shadowMapType = uint(ShadowMapType::HighIsm);
                    gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx].shadowMapSize = gHighIsmSize;

                    // Clear the shadow map type of old light face.
                    // We should avoid the old face that has just changed to the PSM face in previous step
                    if (oldLightFaceID != UINT32_MAX && oldLightFaceData.shadowMapType != uint(ShadowMapType::PSM))
                    {
                        print("oldLightFaceID = ", oldLightFaceID);
                        print("oldLightFaceData.shadowMapType = ", oldLightFaceData.shadowMapType);
                        print("oldLightFaceData.highResIsmTexArrayIdx = ", oldLightFaceData.highResIsmTexArrayIdx);

                        gLightShadowDataBuffer[oldLocalLightID].lightFaceData[oldFaceIdx].shadowMapType = uint(ShadowMapType::None);
                        gLightShadowDataBuffer[oldLocalLightID].lightFaceData[oldFaceIdx].shadowMapSize = 0u;
                    }
                }

                currNewHighIsmIdx++;
            }
        }
    }

    print("rank = ", rank);

    // Process the rest Low-res ISM in the ranking
    if (gVisibility != uint(Visibility::ShadowMap_FullyLit))
    {
        for (; rank < gTotalLightFaceCount; rank++)
        {
            uint globalFaceID = gSortedLightsBuffer[rank].x;
            uint2 localLightIDAndFaceIdx = getLocalLightIdAndFaceIdx(globalFaceID, gFirstSpotLightID);
            uint localLightID = localLightIDAndFaceIdx.x;
            uint faceIdx = localLightIDAndFaceIdx.y;
            LightFaceData lightFaceData = gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx];

            int globalIsmTexArrayIdx = int(rank);
            gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx].ismTexArrayIdx = globalIsmTexArrayIdx % (int)gMaxIsmPerArray;
            gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx].whichIsmTexArray = uint(globalIsmTexArrayIdx / gMaxIsmPerArray);

            // At this point, consider the light face as ISM face if its shadow map type is None
            if (lightFaceData.shadowMapType == uint(ShadowMapType::None))
            {
                // print("globalFaceID = ", globalFaceID);
                gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx].shadowMapType = (uint)ShadowMapType::LowIsm;
                gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx].shadowMapSize = gLowIsmSize;

                // Count ISM face pixels
                gTotalValidPixels[1] += gSortedLightsBuffer[rank].y;
            }
        }
    }

#if DEBUG_PRINT_MODE
    // Record size data
    for (uint r = 0; r < gTotalLightFaceCount; r++)
    {
        if (gSortingRules == (uint)SortingRules::LightFaces)
        {
            for (uint face = 0; face < gLightShadowDataBuffer[r].numShadowMaps; face++)
            {
                uint shadowMapSize = gLightShadowDataBuffer[r].lightFaceData[face].shadowMapSize;
                if (shadowMapSize > 0)
                {
                    gShadowMapSizeBuffer[(uint)log2(shadowMapSize) - 5] += 1;
                }
            }
        }
        else
        {
            uint shadowMapSize = gLightShadowDataBuffer[r].lightFaceData[0].shadowMapSize;
            if (shadowMapSize > 0)
            {
                gShadowMapSizeBuffer[(uint)log2(shadowMapSize) - 5] += 1;
            }
        }
    }

    // Record reusing list light data
    for (uint i = 0; i < PSM_COUNT; i++)
    {
        uint currLightOrGlobalFaceIdx = gReusingLightIndexBuffer[i];
        uint2 localLightIDAndFaceIdx = getLocalLightIdAndFaceIdx(currLightOrGlobalFaceIdx, gFirstSpotLightID);
        uint localLightID = localLightIDAndFaceIdx.x;
        uint faceIdx = localLightIDAndFaceIdx.y;
        LightFaceData lightFaceData = gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx];

        gRecordReusingLightData[i] = uint4(localLightID, faceIdx, lightFaceData.frequency, 0);
    }
#endif
}
