/** This shader not only stores the shadow maps generated in current frame, but also update the light mesh data buffer
*/

#include "../HostDeviceSharedDefinitions.h"
#include "Utils/Math/MathConstants.slangh"

import Utils.Debug.PixelDebug;

cbuffer CB
{
    uint gTotalLightMeshCount;
    uint gLightMeshTopN;
    uint gShadowMapsPerLight;
    uint gCurrFrameLightStartIdx;
}

Buffer<uint2> gSortedLightMeshBuffer; // only need top N light index
RWBuffer<uint> gReusingLightIndexBuffer;
RWStructuredBuffer<LightMeshData> gLightMeshDataBuffer;

[numthreads(1, 1, 1)]
void main(uint3 threadId : SV_DispatchThreadID)
{
    printSetPixel(threadId.xy);
    print("Update Light Mesh Data Shader", 1);

    uint reusingBufferIdx = gCurrFrameLightStartIdx;

    // Clear previous shadow map data first if there is any
    for (uint j = 0; j < gLightMeshTopN; j++)
    {
        // Get previous stored light mesh index in reusing buffer, we're going to update its data
        uint prevLightMeshIndex = gReusingLightIndexBuffer[reusingBufferIdx + j];

        // Clear previous shadow map data if there is any
        if (prevLightMeshIndex != UINT32_MAX)
        {
            gLightMeshDataBuffer[prevLightMeshIndex].startIndex = -1;
            gLightMeshDataBuffer[prevLightMeshIndex].shadowMapCount = 0;
        }
    }

    // Then perform updating
    for (uint i = 0; i < gTotalLightMeshCount && reusingBufferIdx < (gCurrFrameLightStartIdx + gLightMeshTopN); i++)
    {
        uint lightMeshIndex = gSortedLightMeshBuffer[i].x;
        print("i = ", i);
        print("lightMeshIndex = ", lightMeshIndex);
        print("shadowMapCount = ", gLightMeshDataBuffer[lightMeshIndex].shadowMapCount);

        if (gLightMeshDataBuffer[lightMeshIndex].shadowMapCount == 0) 
        {
            // Light has no shadow maps stored, go and store its data
        
            // Store new light mesh index and update its light data
            gReusingLightIndexBuffer[reusingBufferIdx] = lightMeshIndex;
            gLightMeshDataBuffer[lightMeshIndex].startIndex = (int)(reusingBufferIdx * gShadowMapsPerLight);
            gLightMeshDataBuffer[lightMeshIndex].shadowMapCount = (int)gShadowMapsPerLight;

            // Move pointer to next location
            reusingBufferIdx++;
        }
    }

}
