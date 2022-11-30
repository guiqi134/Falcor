/** This shader not only stores the shadow maps generated in current frame, but also update the light data buffer
*/

#include "../HostDeviceSharedDefinitions.h"
#include "Utils/Math/MathConstants.slangh"

import Utils.Debug.PixelDebug;

static const uint kMaxLightsPerTexArray = 340u;
static const uint kMaxLightsPerTexArray1024 = 168u;
static const uint kMaxSupportedLights1024 = 336;

cbuffer CB
{
    uint gTotalLightsCount;
    uint gMaxReusingCount;
    uint gLightTopN;
    uint gShadowMapsPerLight;
    uint gCurrFrameLightStartIdx; // where should updates start?
    uint gIsmPerLight;
    uint gVisibility;
    bool gFullSizeShadowMaps;
    uint gTotalScreenPixels;
    uint gSceneName;
}

Buffer<uint2> gSortedLightsBuffer; // only need top N light index
Buffer<uint> gTotalValidPixels;
RWBuffer<uint> gReusingLightIndexBuffer;
RWBuffer<uint> gIsmLightIndexBuffer;
RWStructuredBuffer<LightShadowMapData> gLightShadowDataBuffer;
RWBuffer<uint> gShadowMapSizeBuffer;

static const uint2 lightUpdatingData[132] = {
    uint2(51, 512), uint2(26, 512), uint2(50, 512), uint2(47, 512), uint2(48, 512), uint2(129, 512), uint2(30, 512), uint2(128, 512),
    uint2(49, 512), uint2(29, 512), uint2(46, 512), uint2(130, 512), uint2(127, 512), uint2(71, 512), uint2(7, 512), uint2(131, 512),
    uint2(101, 512), uint2(13, 512), uint2(69, 512), uint2(8, 512),

    uint2(98, 128), uint2(100, 128), uint2(33, 128), uint2(65, 256),
    uint2(12, 256), uint2(72, 128), uint2(34, 128), uint2(70, 128), uint2(97, 128), uint2(6, 256), uint2(95, 128), uint2(67, 64),
    uint2(32, 256), uint2(94, 256), uint2(41, 256), uint2(85, 128), uint2(73, 128), uint2(35, 64), uint2(102, 128), uint2(11, 512),

    uint2(66, 64), uint2(99, 64), uint2(42, 64), uint2(86, 64), uint2(83, 64), uint2(21, 64), uint2(84, 64), uint2(22, 64),
    uint2(52, 64), uint2(104, 64), uint2(103, 64), uint2(37, 64), uint2(24, 64), uint2(36, 64), uint2(9, 64), uint2(96, 64),
    uint2(10, 64), uint2(15, 64), uint2(25, 64), uint2(74, 64), uint2(92, 64), uint2(62, 64), uint2(80, 64), uint2(79, 64),
    uint2(16, 64), uint2(58, 64), uint2(106, 64), uint2(23, 64), uint2(20, 64), uint2(68, 64), uint2(31, 64), uint2(109, 64),
    uint2(14, 64), uint2(107, 64), uint2(91, 64), uint2(1, 64), uint2(59, 64), uint2(54, 64), uint2(110, 64), uint2(17, 64),
    uint2(63, 64), uint2(64, 64), uint2(105, 64), uint2(88, 64), uint2(126, 64), uint2(87, 64), uint2(0, 64), uint2(77, 64),
    uint2(18, 64), uint2(53, 64), uint2(89, 64), uint2(81, 64), uint2(19, 64), uint2(82, 64), uint2(60, 64), uint2(78, 64),
    uint2(108, 64), uint2(125, 64), uint2(93, 64), uint2(39, 64), uint2(90, 64), uint2(40, 64), uint2(124, 64), uint2(111, 64),
    uint2(76, 32), uint2(75, 64), uint2(61, 64), uint2(55, 64), uint2(43, 64), uint2(116, 64), uint2(57, 64), uint2(45, 64),
    uint2(38, 64), uint2(119, 64), uint2(56, 64), uint2(121, 64), uint2(123, 64), uint2(118, 64), uint2(117, 64), uint2(120, 64),
    uint2(114, 64), uint2(112, 64), uint2(122, 64), uint2(115, 64), uint2(113, 64), uint2(2, 64), uint2(3, 64), uint2(4, 64),
    uint2(5, 64), uint2(27, 64), uint2(28, 64), uint2(44, 64),
};


[numthreads(1, 1, 1)]
void main(uint3 threadId : SV_DispatchThreadID)
{
    if (threadId.x == 0) return;

    // printSetPixel(threadId.xy);
    print("Update Light Mesh Data Shader", 1);

    uint reusingBufferIdx = gCurrFrameLightStartIdx;
    uint i = 0;
    uint prevShadowMapLights[8];

    if (gVisibility != (uint)Visibility::AllISM)
    {
        // Clear previous shadow map data first if there is any
        for (uint j = 0; j < gLightTopN; j++)
        {
            // Get previous stored light mesh index in reusing buffer, we're going to update its data
            uint prevLocalLightIdx = gReusingLightIndexBuffer[reusingBufferIdx + j];

            // Add to a staging array
            prevShadowMapLights[j] = prevLocalLightIdx;

            // Clear previous shadow map data if there is any
            if (prevLocalLightIdx != UINT32_MAX)
            {
                gLightShadowDataBuffer[prevLocalLightIdx].startIndex = -1;
                gLightShadowDataBuffer[prevLocalLightIdx].shadowMapCount = 0;
                gLightShadowDataBuffer[prevLocalLightIdx].shadowMapType = 0;
            }
        }

        // Then perform updating
        for (; i < gMaxReusingCount && reusingBufferIdx < (gCurrFrameLightStartIdx + gLightTopN); i++)
        {
            uint localLightIdx = gSortedLightsBuffer[i].x;
            print("i = ", i);
            print("localLightIdx = ", localLightIdx);
            print("shadowMapCount = ", gLightShadowDataBuffer[localLightIdx].shadowMapCount);

            if (gLightShadowDataBuffer[localLightIdx].shadowMapType != 1)
            {
                // Light has no shadow maps stored, go and store its data

                // Store new light mesh index and update its light data
                gReusingLightIndexBuffer[reusingBufferIdx] = localLightIdx;
                gLightShadowDataBuffer[localLightIdx].startIndex = (int)(reusingBufferIdx * gShadowMapsPerLight);
                gLightShadowDataBuffer[localLightIdx].shadowMapCount = (int)gShadowMapsPerLight;
                gLightShadowDataBuffer[localLightIdx].shadowMapType = 1;
                gLightShadowDataBuffer[localLightIdx].shadowMapSize = 1024;

                // Move pointer to next location
                reusingBufferIdx++;
            }
        }

        // TODO: get the new lights added into the list. No need to process the duplicate light
    }

    // Also update the rest lights using ISM if it is not a regular shadow map type
    if (gVisibility != (uint)Visibility::ShadowMap_FullyLit && gVisibility != (uint)Visibility::Experiment)
    {
        uint bufferSize;
        gIsmLightIndexBuffer.GetDimensions(bufferSize);
        // bufferSize = 12; // Only store top 12 lights
        for (uint bufferIdx = 0; bufferIdx < bufferSize; i++)
        {
            uint localLightIdx = gSortedLightsBuffer[i].x;
            if (gLightShadowDataBuffer[localLightIdx].shadowMapType != 1)
            {
                // Store new light mesh index and update its light data
                gIsmLightIndexBuffer[bufferIdx] = localLightIdx;
                gLightShadowDataBuffer[localLightIdx].startIndex = (int)(bufferIdx * gIsmPerLight);
                gLightShadowDataBuffer[localLightIdx].shadowMapCount = (int)gIsmPerLight;
                gLightShadowDataBuffer[localLightIdx].shadowMapType = 2;
                gLightShadowDataBuffer[localLightIdx].shadowMapSize = 128;

                bufferIdx++;
            }
        }
    }

    // Experiment: updating light shadow map size for each light
    if (gVisibility == (uint)Visibility::Experiment)
    {
        for (uint i = 0; i < gTotalLightsCount; i++)
        {
            uint shadowMapSize;
            uint lightID = gSortedLightsBuffer[i].x;

            if (!gFullSizeShadowMaps)
            {
                uint numPixels = gSortedLightsBuffer[i].y;
                float percentage = float(numPixels) / float(gTotalValidPixels[0]);

                if (percentage >= 0.1 && i < kMaxLightsPerTexArray) shadowMapSize = 1024;
                else if (percentage >= 0.01) shadowMapSize = 512;
                else if (percentage >= 0.004) shadowMapSize = 256;
                else if (percentage >= 0.002) shadowMapSize = 128;
                else if (percentage >= 0.0005) shadowMapSize = 64;
                else if (percentage > 0.0) shadowMapSize = 32;
                else shadowMapSize = 1;
            }
            else
            {
                shadowMapSize = gSceneName != 2 ? 1024 :
                    i < kMaxSupportedLights1024 ? 1024 :
                    512;
            }

            gLightShadowDataBuffer[lightID].shadowMapSize = shadowMapSize;

            // Record size data
            if (shadowMapSize >= 32)
            {
                gShadowMapSizeBuffer[(uint)log2(shadowMapSize) - 5] += 1;
            }

            // Updating light texture array index
            uint maxLightsPerTexArray = shadowMapSize != 1024 ? kMaxLightsPerTexArray : kMaxLightsPerTexArray1024;
            gLightShadowDataBuffer[lightID].texArrayIndex = i / maxLightsPerTexArray;
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
