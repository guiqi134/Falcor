/** This shader not only stores the shadow maps generated in current frame, but also update the light data buffer
*/

#include "../HostDeviceSharedDefinitions.h"
#include "Utils/Math/MathConstants.slangh"

import Utils.Debug.PixelDebug;
import ISMHelpers;

#ifndef VALID_ARRAY_SIZE
#define VALID_ARRAY_SIZE 40
#endif

cbuffer CB
{
    uint gTotalBinCount;
    uint gMaxReusingCount;
    uint gTopN;
    uint gTemporalReusingLength;
    uint gShadowMapsPerLight;
    uint gCurrFrameReusingStartIdx; // where should updates start?
    uint gIsmPerLight;
    uint gIsmSize;
    uint gVisibility;
    bool gFullSizeShadowMaps;
    uint gTotalScreenPixels;
    uint gSceneName;
    uint gAgeThreshold;
    bool gLimitSwitchRegion;
    uint gMaxIsmPerArray;
    uint gSortingRules;
    uint gMaxSupportedShadowMaps1024;
    uint gMaxPsmPerArray;
    uint gTemporalReusingFix;
}

Buffer<uint2> gSortedLightsBuffer; // only need top N light/face index
RWBuffer<uint> gReusingLightIndexBuffer;
RWBuffer<uint> gIsmLightIndexBuffer;
RWStructuredBuffer<LightShadowMapData> gLightShadowDataBuffer;
RWBuffer<uint> gTotalValidPixels;

RWBuffer<uint> gShadowMapSizeBuffer;
RWBuffer<uint4> gRecordReusingLightData;

// Updating light shadow map data for the parts changed in scene animation
StructuredBuffer<LightShadowMapData> gNewLightShadowDataBuffer;

[numthreads(256, 1, 1)]
void updateCenterCS(uint3 threadId: SV_DispatchThreadID)
{
    printSetPixel(threadId.xy);

    gLightShadowDataBuffer[threadId.x].centerPosW = gNewLightShadowDataBuffer[threadId.x].centerPosW;

    for (uint face = 0; face < gShadowMapsPerLight; face++)
    {
        gLightShadowDataBuffer[threadId.x].lightFaceData[face].viewMat = gNewLightShadowDataBuffer[threadId.x].lightFaceData[face].viewMat;
    }
}

[numthreads(256, 1, 1)]
void generalUpdates(uint3 threadId: SV_DispatchThreadID)
{
    if (gSortingRules == (uint)SortingRules::LightFaces)
    {
        uint globalFaceIdx = gSortedLightsBuffer[threadId.x].x;
        uint localLightIdx = globalFaceIdx / gShadowMapsPerLight;
        uint faceIdx = globalFaceIdx % gShadowMapsPerLight;

        gLightShadowDataBuffer[localLightIdx].lightFaceData[faceIdx].currRanking = threadId.x;
        gLightShadowDataBuffer[localLightIdx].lightFaceData[faceIdx].accumulatedFrames += 1;
        gLightShadowDataBuffer[localLightIdx].lightFaceData[faceIdx].psmAge += 1;
    }
    else
    {
        uint localLightIdx = gSortedLightsBuffer[threadId.x].x;

        // Write the ranking into light current ranking data
        gLightShadowDataBuffer[localLightIdx].lightFaceData[0].currRanking = threadId.x;
        // Count the frames accumulated
        gLightShadowDataBuffer[threadId.x].lightFaceData[0].accumulatedFrames += 1;
        gLightShadowDataBuffer[threadId.x].lightFaceData[0].psmAge += 1;
    }
}

// This is at the end of stochastic shadow map function
[numthreads(256, 1, 1)]
void updatePrevViewMatrices(uint3 threadID: SV_DispatchThreadID)
{
    for (uint face = 0; face < gShadowMapsPerLight; face++)
    {
        gLightShadowDataBuffer[threadID.x].lightFaceData[face].prevViewMat = gLightShadowDataBuffer[threadID.x].lightFaceData[face].viewMat;
    }
}

[numthreads(1, 1, 1)]
void main(uint3 threadId : SV_DispatchThreadID)
{
    if (threadId.x == 0) return;

    printSetPixel(threadId.xy);
    print("Update Light Mesh Data Shader", 1);

    uint reusingBufferIdx = gCurrFrameReusingStartIdx;
    print("reusingBufferIdx = ", reusingBufferIdx);

    // Find the top N lights in this frame and only update the valid light (light that is less posssible to cause flickering)
    if (gVisibility != (uint)Visibility::AllISM)
    {
        bool validMask[VALID_ARRAY_SIZE];
        uint validCount = 0;

        print("VALID_ARRAY_SIZE = ", VALID_ARRAY_SIZE);

        // 1. Iterate over all the light face in current frame's reusing section
        for (uint j = 0; j < gTopN; j++)
        {
            // Get previous stored light mesh/face index in reusing buffer, we're going to update its data
            uint prevLightOrGlobalFaceIdx = gReusingLightIndexBuffer[reusingBufferIdx + j];

            // It is valid if there is no light data
            if (prevLightOrGlobalFaceIdx == UINT32_MAX)
            {
                validMask[j] = true;
                validCount++;
            }
            else
            {
                // Get the local light index and face index
                uint localLightIdx = gSortingRules == (uint)SortingRules::LightFaces ? prevLightOrGlobalFaceIdx / gShadowMapsPerLight : prevLightOrGlobalFaceIdx;
                uint faceIdx = gSortingRules == (uint)SortingRules::LightFaces ? prevLightOrGlobalFaceIdx % gShadowMapsPerLight : 0u;
                LightFaceData lightFaceData = gLightShadowDataBuffer[localLightIdx].lightFaceData[faceIdx]; // only for read

                // Mark this face as old
                gLightShadowDataBuffer[localLightIdx].lightFaceData[faceIdx].newPsmLightFace = false;
                gLightShadowDataBuffer[localLightIdx].lightFaceData[faceIdx].psmAge = 0;

                // Limit the light switching around the boundary region
                bool isSwitchingRegionOk = true;
                uint prevRanking = lightFaceData.prevRanking;
                uint currRanking = lightFaceData.currRanking;
                if (gLimitSwitchRegion)
                {
                    const uint2 regionSize = uint2(2u);
                    uint3 limitedRegion = gTopN * uint3(gTemporalReusingLength - regionSize.x, gTemporalReusingLength, gTemporalReusingLength + regionSize.y);
                    isSwitchingRegionOk = !((prevRanking >= limitedRegion.x && prevRanking < limitedRegion.z) && (currRanking >= limitedRegion.x && currRanking < limitedRegion.z));
                    // isSwitchingRegionOk = !((prevRanking >= limitedRegion.x && prevRanking < limitedRegion.y) && (currRanking >= limitedRegion.y && currRanking < limitedRegion.z));

                    // print("limitedRegion = ", limitedRegion);
                    // print("prevRanking = ", prevRanking);
                    // print("currRanking = ", currRanking);
                }

                // print("isSwitchingRegionOk = ", isSwitchingRegionOk);

                // Check if the light accumulated frames is large enough
                bool isAccumulatedFramesOk = lightFaceData.accumulatedFrames >= gAgeThreshold;
                isAccumulatedFramesOk = true;

                // Clear previous shadow map data if it is completely valid
                if (isSwitchingRegionOk && isAccumulatedFramesOk)
                {
                    validMask[j] = true;
                    validCount++;

                    gLightShadowDataBuffer[localLightIdx].lightFaceData[faceIdx].shadowMapType = 0; // this is most important
                    gLightShadowDataBuffer[localLightIdx].lightFaceData[faceIdx].shadowMapSize = 0;
                }
                else
                {
                    validMask[j] = false;
                }
            }
        }

        print("validCount = ", validCount);
        print("gCurrFrameReusingStartIdx = ", gCurrFrameReusingStartIdx);
        print("gTotalBinCount = ", gTotalBinCount);

        // 2. Iterate over face ranking list in this frame. Updating PSM light face ID and light shadow data for valid index selected in previous step
        uint3 newPsmLightFaces[VALID_ARRAY_SIZE];
        uint currNewPsmIdx = 0;
        uint j = 0;
        for (uint i = 0; i < gTotalBinCount; i++)
        {
            // Get light and face index
            uint lightOrGlobalFaceIdx = gSortedLightsBuffer[i].x;
            uint localLightIdx = gSortingRules == (uint)SortingRules::LightFaces ? lightOrGlobalFaceIdx / gShadowMapsPerLight : lightOrGlobalFaceIdx;
            uint faceIdx = gSortingRules == (uint)SortingRules::LightFaces ? lightOrGlobalFaceIdx % gShadowMapsPerLight : 0u;
            LightFaceData lightFaceData = gLightShadowDataBuffer[localLightIdx].lightFaceData[faceIdx]; // only for read

            gLightShadowDataBuffer[localLightIdx].lightFaceData[faceIdx].rankingFreqInFrame = uint2(i, gSortedLightsBuffer[i].y);

            // Update old PSM light data and some partial data of new PSM light
            if (lightFaceData.shadowMapType != 1 && j < validCount)
            {
                // Record the ID of new PSM light
                if (lightFaceData.newPsmLightFace)
                {
                    print("lightOrGlobalFaceIdx = ", lightOrGlobalFaceIdx);
                    newPsmLightFaces[currNewPsmIdx] = uint3(lightOrGlobalFaceIdx, localLightIdx, faceIdx);
                    currNewPsmIdx++;
                }
                else
                {
                    uint validMaskIndex = (gSortingRules == (uint)SortingRules::LightFaces ? lightFaceData.psmTexArrayIdx : lightFaceData.psmTexArrayIdx / gShadowMapsPerLight) % gTopN;
                    validMask[validMaskIndex] = false;

                    print("validMaskIndex = ", validMaskIndex);
                }

                // Mark the light face as PSM face
                gLightShadowDataBuffer[localLightIdx].lightFaceData[faceIdx].shadowMapType = 1;

                // Reset the age
                gLightShadowDataBuffer[localLightIdx].lightFaceData[faceIdx].psmAge = 0;

                // Use the ranking change to see if this is a switch from ISM to PSM
                uint prevRanking = lightFaceData.prevRanking;
                uint currRanking = lightFaceData.currRanking;
                if (prevRanking >= 20 && currRanking < 20)
                    gLightShadowDataBuffer[localLightIdx].lightFaceData[faceIdx].accumulatedFrames = 0;

                // Adaptive select the shadow map size
                uint shadowMapSize;
                if (gFullSizeShadowMaps)
                {
                    uint maxSupportedSize = gSortingRules == (uint)SortingRules::LightFaces ? gMaxSupportedShadowMaps1024 : kMaxSupportedLights1024;
                    shadowMapSize = i < maxSupportedSize ? 1024 : 512;
                }
                else
                {
                    uint numPixels = gSortedLightsBuffer[i].y;
                    float percentage = float(numPixels) / float(gTotalValidPixels[0]);

                    if (percentage >= 0.05) shadowMapSize = 1024;
                    else if (percentage >= 0.005) shadowMapSize = 512;
                    else if (percentage >= 0.002) shadowMapSize = 256;
                    else if (percentage >= 0.001) shadowMapSize = 128;
                    else if (percentage >= 0.00025) shadowMapSize = 64;
                    else if (percentage > 0.0) shadowMapSize = 32;
                    else shadowMapSize = 1;
                }
                gLightShadowDataBuffer[localLightIdx].lightFaceData[faceIdx].shadowMapSize = shadowMapSize;

                // Move pointer to next location
                j++;
            }
        }

        // Replace target positions in reusing list with new PSM light face and update indexing data
        currNewPsmIdx = 0;
        for (j = 0; j < gTopN; j++)
        {
            if (validMask[j])
            {
                uint3 newPsmLightFace = newPsmLightFaces[currNewPsmIdx];
                uint currReusingBufferIdx = reusingBufferIdx + j;
                uint localLightIdx = newPsmLightFace.y;
                uint faceIdx = newPsmLightFace.z;

                print("newPsmLightFace = ", newPsmLightFace);

                // Only update the index for new PSM light face.
                gReusingLightIndexBuffer[currReusingBufferIdx] = newPsmLightFace.x;
                if (gSortingRules == (uint)SortingRules::LightFaces)
                {
                    int globalPsmTexArrayIdx = (int)currReusingBufferIdx;
                    gLightShadowDataBuffer[localLightIdx].lightFaceData[faceIdx].psmTexArrayIdx = globalPsmTexArrayIdx < gMaxSupportedShadowMaps1024 ? globalPsmTexArrayIdx :
                    int((globalPsmTexArrayIdx - gMaxSupportedShadowMaps1024) % gMaxPsmPerArray);
                    gLightShadowDataBuffer[localLightIdx].lightFaceData[faceIdx].whichPsmTexArray = globalPsmTexArrayIdx < gMaxSupportedShadowMaps1024 ? 0u :
                    int((globalPsmTexArrayIdx - gMaxSupportedShadowMaps1024) / gMaxPsmPerArray + 1u);
                }
                else
                {
                    int globalPsmTexArrayStartIdx = int(currReusingBufferIdx * gShadowMapsPerLight);
                    for (uint face = 0; face < gShadowMapsPerLight; face++)
                    {
                        int globalPsmTexArrayIdx = int(globalPsmTexArrayStartIdx + face);
                        gLightShadowDataBuffer[localLightIdx].lightFaceData[face].psmTexArrayIdx = globalPsmTexArrayIdx < gMaxSupportedShadowMaps1024 ? globalPsmTexArrayIdx :
                        int((globalPsmTexArrayIdx - gMaxSupportedShadowMaps1024) % gMaxPsmPerArray);
                        gLightShadowDataBuffer[localLightIdx].lightFaceData[face].whichPsmTexArray = globalPsmTexArrayIdx < gMaxSupportedShadowMaps1024 ? 0u :
                        int((globalPsmTexArrayIdx - gMaxSupportedShadowMaps1024) / gMaxPsmPerArray + 1u);
                    }
                }

                currNewPsmIdx++;
            }
        }
    }

    // 3. Iterate over face ranking list in this frame. Also update all ISM light face resources
    if (gVisibility != (uint)Visibility::ShadowMap_FullyLit)
    {
        uint bufferSize;
        gIsmLightIndexBuffer.GetDimensions(bufferSize);
        for (uint bufferIdx = 0; bufferIdx < bufferSize; bufferIdx++)
        {
            // Get light and face index
            uint lightOrGlobalFaceIdx = gSortedLightsBuffer[bufferIdx].x;
            uint localLightIdx = gSortingRules == (uint)SortingRules::LightFaces ? lightOrGlobalFaceIdx / gShadowMapsPerLight : lightOrGlobalFaceIdx;
            uint faceIdx = gSortingRules == (uint)SortingRules::LightFaces ? lightOrGlobalFaceIdx % gShadowMapsPerLight : 0u;
            LightFaceData lightFaceData = gLightShadowDataBuffer[localLightIdx].lightFaceData[faceIdx]; // only for read

            if (gSortingRules == (uint)SortingRules::LightFaces)
            {
                int globalIsmTexArrayIdx = int(bufferIdx);
                gLightShadowDataBuffer[localLightIdx].lightFaceData[faceIdx].ismTexArrayIdx = globalIsmTexArrayIdx % (int)gMaxIsmPerArray;
                gLightShadowDataBuffer[localLightIdx].lightFaceData[faceIdx].whichIsmTexArray = uint(globalIsmTexArrayIdx / gMaxIsmPerArray);
            }
            else
            {
                int globalIsmTexArrayStartIdx = int(bufferIdx * gIsmPerLight);
                for (uint face = 0; face < gIsmPerLight; face++)
                {
                    int globalIsmTexArrayIdx = int(globalIsmTexArrayStartIdx + face);
                    gLightShadowDataBuffer[localLightIdx].lightFaceData[face].ismTexArrayIdx = globalIsmTexArrayIdx % (int)gMaxIsmPerArray;
                    gLightShadowDataBuffer[localLightIdx].lightFaceData[face].whichIsmTexArray = uint(globalIsmTexArrayIdx / gMaxIsmPerArray);
                }
            }


            if (lightFaceData.shadowMapType != 1)
            {
                // ISM light face can be considered as a potiential new PSM light face
                gLightShadowDataBuffer[localLightIdx].lightFaceData[faceIdx].newPsmLightFace = true;
                gLightShadowDataBuffer[localLightIdx].lightFaceData[faceIdx].psmAge = 0;

                // Store new light mesh index and update its light data
                gIsmLightIndexBuffer[bufferIdx] = lightOrGlobalFaceIdx;
                gLightShadowDataBuffer[localLightIdx].lightFaceData[faceIdx].shadowMapType = 2;
                gLightShadowDataBuffer[localLightIdx].lightFaceData[faceIdx].shadowMapSize = gIsmSize;

                // Use the ranking change to see if this is a switch from SM to ISM
                uint prevRanking = lightFaceData.prevRanking;
                uint currRanking = lightFaceData.currRanking;
                if (prevRanking < 20 && currRanking >= 20)
                    gLightShadowDataBuffer[localLightIdx].lightFaceData[faceIdx].accumulatedFrames = 0;

                // Count ISM lights pixels
                gTotalValidPixels[1] += gSortedLightsBuffer[bufferIdx].y;
            }
        }
    }

    // 4. Write current ranking into previous ranking
    for (uint j = 0; j < gTopN; j++)
    {
        uint currLightOrGlobalFaceIdx = gReusingLightIndexBuffer[reusingBufferIdx + j];
        uint localLightIdx = gSortingRules == (uint)SortingRules::LightFaces ? currLightOrGlobalFaceIdx / gShadowMapsPerLight : currLightOrGlobalFaceIdx;
        uint faceIdx = gSortingRules == (uint)SortingRules::LightFaces ? currLightOrGlobalFaceIdx % gShadowMapsPerLight : 0u;

        gLightShadowDataBuffer[localLightIdx].lightFaceData[faceIdx].prevRanking = gLightShadowDataBuffer[localLightIdx].lightFaceData[faceIdx].currRanking;
    }

    // Record size data
    {
        for (uint r = 0; r < gTotalBinCount; r++)
        {
            if (gSortingRules == (uint)SortingRules::LightFaces)
            {
                for (uint face = 0; face < gShadowMapsPerLight; face++)
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
    }

    // Record reusing list light data
    for (uint i = 0; i < gMaxReusingCount; i++)
    {
        uint currLightOrGlobalFaceIdx = gReusingLightIndexBuffer[i];
        // if (i > 100)
        //     print("currLightOrGlobalFaceIdx = ", currLightOrGlobalFaceIdx);
        uint localLightIdx = gSortingRules == (uint)SortingRules::LightFaces ? currLightOrGlobalFaceIdx / gShadowMapsPerLight : currLightOrGlobalFaceIdx;
        uint faceIdx = gSortingRules == (uint)SortingRules::LightFaces ? currLightOrGlobalFaceIdx % gShadowMapsPerLight : 0u;
        LightFaceData lightFaceData = gLightShadowDataBuffer[localLightIdx].lightFaceData[faceIdx];

        // gRecordReusingLightData[i] = uint4(lightFaceData.accumulatedFrames, lightFaceData.rankingFreqInFrame.x, lightFaceData.rankingFreqInFrame.y, 0);
        gRecordReusingLightData[i] = uint4(lightFaceData.newPsmLightFace, lightFaceData.rankingFreqInFrame.x, lightFaceData.rankingFreqInFrame.y, 0);
    }
}


