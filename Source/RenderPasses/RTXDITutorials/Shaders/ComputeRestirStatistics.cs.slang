#include "RtxdiBridge.slangh"

import Utils.Debug.PixelDebug;

[numthreads(16, 16, 1)]
void main(uint3 threadID: SV_DispatchThreadID)
{
    const uint2 launchIdx = threadID.xy;
    uint bufferIndex = gScreenSize.x * launchIdx.y + launchIdx.x;
    int lightID = gPrevLightSelectionBuffer[bufferIndex];

    // Get surface from our G-buffer
    RAB_Surface surface = RAB_GetGBufferSurface((int2)launchIdx, false);
    bool isValidSurface = RAB_IsSurfaceValid(surface);

    if (lightID == -1 || !isValidSurface) return;

    // Project pixel's shading point to post-rotated light face
    float3 posW = RAB_GetSurfaceWorldPos(surface);
    float3 lightPosW = gLightShadowDataBuffer[lightID].centerPosW;
    float3 lightToSurfaceW = normalize(posW - lightPosW);
    uint faceIdx = getClosestFace(lightToSurfaceW, gLightShadowDataBuffer[lightID]);

    // Compuate global face ID
    gPrevLightSelectionBuffer[bufferIndex] = int(lightID < gFirstSpotLightID ? kShadowMapsPerPointLight * lightID + faceIdx :
        kShadowMapsPerPointLight * gFirstSpotLightID + (lightID - gFirstSpotLightID));
}
