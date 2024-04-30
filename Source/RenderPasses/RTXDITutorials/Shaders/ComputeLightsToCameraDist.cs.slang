import Utils.Debug.PixelDebug;
import ISMHelpers;

cbuffer CB
{
    uint gFirstSpotLightID;
    float3 gCameraPosW;
}

StructuredBuffer<LightShadowMapData> gLightShadowDataBuffer;
RWStructuredBuffer<float2> gSortBuffer;

[numthreads(1024, 1, 1)]
void computeDistances(uint3 threadID: SV_DispatchThreadID)
{
    printSetPixel(threadID.xy);

    print("computeDistances()-----------------------------", 1);

    uint2 localLightIDAndFaceIdx = getLocalLightIdAndFaceIdx(threadID.x, gFirstSpotLightID);
    uint localLightID = localLightIDAndFaceIdx.x;
    // uint faceIdx = localLightIDAndFaceIdx.y;
    float3 lightCenterPosW = gLightShadowDataBuffer[localLightID].centerPosW;
    float distance = length(lightCenterPosW - gCameraPosW);
    gSortBuffer[threadID.x] = float2(threadID.x, distance);

    print("distance = ", distance);
    print("gSortBuffer = ", gSortBuffer[threadID.x]);
}

StructuredBuffer<float2> gStagingSortedBuffer;
RWStructuredBuffer<uint2> gSourceSortedBuffer;

[numthreads(1024, 1, 1)]
void copyStagingToSource(uint3 threadID: SV_DispatchThreadID)
{
    printSetPixel(threadID.xy);
    print("copyStagingToSource()-----------------------------", 1);

    gSourceSortedBuffer[threadID.x].x = (uint)gStagingSortedBuffer[threadID.x].x;
    gSourceSortedBuffer[threadID.x].y = uint(1000 * gStagingSortedBuffer[threadID.x].y);

    print("gStagingSortedBuffer[threadID.x] = ", gStagingSortedBuffer[threadID.x]);
}
