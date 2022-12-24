/** This shader not only stores the shadow maps generated in current frame, but also update the light data buffer
*/

#include "../HostDeviceSharedDefinitions.h"
#include "Utils/Math/MathConstants.slangh"

import Utils.Debug.PixelDebug;

cbuffer CB
{
    uint gTotalLightsCount;
    uint gMaxReusingCount;
    uint gLightTopN;
    uint gTemporalReusingLength;
    uint gShadowMapsPerLight;
    uint gCurrFrameLightStartIdx; // where should updates start?
    uint gIsmPerLight;
    uint gVisibility;
    bool gFullSizeShadowMaps;
    uint gTotalScreenPixels;
    uint gSceneName;
}

Buffer<uint2> gSortedLightsBuffer; // only need top N light index
RWBuffer<uint> gTotalValidPixels;
RWBuffer<uint> gReusingLightIndexBuffer;
RWBuffer<uint> gIsmLightIndexBuffer;
RWStructuredBuffer<LightShadowMapData> gLightShadowDataBuffer;
RWBuffer<uint> gShadowMapSizeBuffer;

[numthreads(1, 1, 1)]
void main(uint3 threadId : SV_DispatchThreadID)
{
    if (threadId.x == 0) return;

    // printSetPixel(threadId.xy);
    print("Update Light Mesh Data Shader", 1);

    uint reusingBufferIdx = gCurrFrameLightStartIdx;

    // Find the top N lights in this frame and updating their shadow data
    if (gVisibility != (uint)Visibility::AllISM)
    {
        uint validIndex[4];
        uint validCount = 0;

        for (uint j = 0; j < gLightTopN; j++)
        {
            // Get previous stored light mesh index in reusing buffer, we're going to update its data
            uint prevLocalLightIdx = gReusingLightIndexBuffer[reusingBufferIdx + j];

            // It is valid if there is no light data
            if (prevLocalLightIdx == UINT32_MAX)
            {
                validIndex[validCount] = reusingBufferIdx + j;
                validCount++;
            }
            else
            {
                // Clear previous shadow map data if it is old enough
                if (true)
                {
                    validIndex[validCount] = reusingBufferIdx + j;
                    validCount++;

                    gLightShadowDataBuffer[prevLocalLightIdx].shadowMapType = 0; // this is most important
                    gLightShadowDataBuffer[prevLocalLightIdx].shadowMapCount = 0;
                    gLightShadowDataBuffer[prevLocalLightIdx].shadowMapSize = 1;
                }
            }
        }

        // Then updating SM resuing list from the target index
        for (uint i = 0, j = 0; i < gMaxReusingCount && j < validCount; i++)
        {
            uint localLightIdx = gSortedLightsBuffer[i].x;
            uint reusingBufferValidIdx = validIndex[j];
            uint prevLocalLightIdx = gReusingLightIndexBuffer[reusingBufferValidIdx];

            if (gLightShadowDataBuffer[localLightIdx].shadowMapType != 1)
            {
                // Store new light mesh index and update its light data
                gReusingLightIndexBuffer[reusingBufferValidIdx] = localLightIdx;
                gLightShadowDataBuffer[localLightIdx].shadowMapCount = (int)gShadowMapsPerLight;
                gLightShadowDataBuffer[localLightIdx].shadowMapType = 1;
                gLightShadowDataBuffer[localLightIdx].ranking = reusingBufferValidIdx;
                gLightShadowDataBuffer[localLightIdx].reusingArrayIndex = (int)reusingBufferValidIdx;

                // Adaptive select the shadow map size
                uint shadowMapSize;
                if (!gFullSizeShadowMaps)
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
                else
                {
                    shadowMapSize = i < kMaxSupportedLights1024 ? 1024 : 512;
                    shadowMapSize = 128;
                    // shadowMapSize = i < 20 ? shadowMapSize : 128; // just for shadow map resolution test
                }
                gLightShadowDataBuffer[localLightIdx].shadowMapSize = shadowMapSize;

                // Move pointer to next location
                j++;
            }

        }
    }

    // gLightShadowDataBuffer[115].shadowMapType = 0;

    // Also update all lights using ISM if it is not a regular shadow map type
    if (gVisibility != (uint)Visibility::ShadowMap_FullyLit)
    {
        uint bufferSize;
        gIsmLightIndexBuffer.GetDimensions(bufferSize);
        for (uint bufferIdx = 0; bufferIdx < bufferSize; bufferIdx++)
        {
            uint localLightIdx = gSortedLightsBuffer[bufferIdx].x;
            gLightShadowDataBuffer[localLightIdx].ismArrayStartIndex = (int)(bufferIdx * gIsmPerLight);

            if (gLightShadowDataBuffer[localLightIdx].shadowMapType != 1)
            {
                // Store new light mesh index and update its light data
                gIsmLightIndexBuffer[bufferIdx] = localLightIdx;
                gLightShadowDataBuffer[localLightIdx].shadowMapCount = (int)gIsmPerLight;
                gLightShadowDataBuffer[localLightIdx].shadowMapType = 2;
                gLightShadowDataBuffer[localLightIdx].shadowMapSize = 128;
                gLightShadowDataBuffer[localLightIdx].age = 0;

                // Count ISM lights pixels
                gTotalValidPixels[1] += gSortedLightsBuffer[bufferIdx].y;
            }
        }
    }



    // Record size data
    {
        for (uint r = 0; r < gTotalLightsCount; r++)
        {
            uint shadowMapSize = gLightShadowDataBuffer[r].shadowMapSize;

            if (shadowMapSize >= 32)
            {
                gShadowMapSizeBuffer[(uint)log2(shadowMapSize) - 5] += 1;
            }
        }
    }
}

// Updating light shadow map data for the parts changed in scene animation
Buffer<float3> gNewCenterBuffer;

[numthreads(256, 1, 1)]
void updateCenterCS(uint3 threadId: SV_DispatchThreadID)
{
    gLightShadowDataBuffer[threadId.x].centerPosW = gNewCenterBuffer[threadId.x];
}
