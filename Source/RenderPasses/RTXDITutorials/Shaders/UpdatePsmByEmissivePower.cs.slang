import Scene.Scene;
import Utils.Debug.PixelDebug;
import ISMHelpers;

cbuffer CB
{
    uint gFirstSpotLightID;
    uint gLightFaceTopN;
}

RWStructuredBuffer<LightShadowMapData> gLightShadowDataBuffer;
Buffer<uint> gLightFaceSortByPowerBuffer;

[numthreads(256, 1, 1)]
void main(uint3 threadId: SV_DispatchThreadID)
{
    printSetPixel(threadId.xy);

    if (threadId.x >= gLightFaceTopN) return;

    uint2 localLightIDAndFaceIdx = getLocalLightIdAndFaceIdx(gLightFaceSortByPowerBuffer[threadId.x], gFirstSpotLightID);
    uint localLightID = localLightIDAndFaceIdx.x;
    uint faceIdx = localLightIDAndFaceIdx.y;

    gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx].shadowMapType = uint(ShadowMapType::PSM);
    gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx].shadowMapSize = 1024;
    // gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx].typeChanged = true;
    // gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx].fadeInFrameCount = 1;

    int globalPsmTexArrayIdx = threadId.x;
    gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx].psmTexArrayIdx = globalPsmTexArrayIdx;
    gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx].whichPsmTexArray = 0u;

    print("localLightIDAndFaceIdx = ", localLightIDAndFaceIdx);
    print("globalPsmTexArrayIdx = ", globalPsmTexArrayIdx);
    print("shadow map type = ", gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx].shadowMapType);
}
