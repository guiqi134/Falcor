#include "../HostDeviceSharedDefinitions.h"

import Utils.Debug.PixelDebug;

cbuffer reconstructCB
{
    uint gCurrFrameReusingStartIdx;
    uint gTopN;
    uint gShadowMapSize;
    uint gShadowMapsPerLight;
}

RWTexture2DArray<float> gSortedPSMs;
Texture2DArray<float4> gSortedMotionTextures;
StructuredBuffer<LightShadowMapData> gLightShadowDataBuffer;
Buffer<uint> gReusingLightIndexBuffer;

[numthreads(16, 16, 1)]
void main(uint3 threadID: SV_DispatchThreadID)
{
    // printSetPixel(threadID.xy);

    uint texArrayIdx = threadID.z;
    uint2 psmPixelPos = threadID.xy;

    // Early out if this PSM is updated in this frame
    if (texArrayIdx >= gCurrFrameReusingStartIdx && texArrayIdx < gCurrFrameReusingStartIdx + gTopN) return;

    // Motion on xy direction should be scaled by PSM size.
    float3 motionPsmSpace = gSortedMotionTextures[threadID].xyz * float3(gShadowMapSize, gShadowMapSize, 1.0f);

    // Early out if there is no motion for this pixel
    if (all(motionPsmSpace == float3(0))) return;

    // Get light and face index
    uint lightOrGlobalFaceIdx = gReusingLightIndexBuffer[texArrayIdx];
    uint localLightIdx = lightOrGlobalFaceIdx / gShadowMapsPerLight;
    uint faceIdx = lightOrGlobalFaceIdx % gShadowMapsPerLight;

    print("motionPsmSpace = ", motionPsmSpace);

    // Then, all the direction should be scaled by PSM age
    float motionScale = gLightShadowDataBuffer[localLightIdx].lightFaceData[faceIdx].psmAge;
    motionPsmSpace *= motionScale;

    print("motionScale = ", motionScale);

    // Compute the reprojected position using motion vector
    int2 reprojectedPixelPos = int2(round(float2(psmPixelPos) + motionPsmSpace.xy));
    float reprojectedDepth = saturate(gSortedPSMs[threadID] + motionPsmSpace.z);

    print("motionPsmSpace = ", motionPsmSpace);
    print("reprojectedPixelPos = ", reprojectedPixelPos);

    // Check if the reprojeced pixel is inside the PSM
    if (all(reprojectedPixelPos >= 0) && all(reprojectedPixelPos < gShadowMapSize))
    {
        float targetDpeth = gSortedPSMs[uint3(reprojectedPixelPos, texArrayIdx)];
        gSortedPSMs[uint3(reprojectedPixelPos, texArrayIdx)] = min(targetDpeth, reprojectedDepth);
    }

    // How to handle the disoccluded pixels?
    //  -> 1. Set the disoccluded (previous) pixel to 1.0f
    //  -> 2. TODO: use ISM
    // gSortedPSMs[threadID] = 1.0f;

}
