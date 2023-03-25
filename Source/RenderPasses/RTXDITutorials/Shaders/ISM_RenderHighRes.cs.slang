// #include "../HostDeviceSharedDefinitions.h"
#include "Utils/Math/MathConstants.slangh"

import Scene.Scene;
import Utils.Sampling.TinyUniformSampleGenerator;
import Utils.Math.MathHelpers;
import Utils.Debug.PixelDebug;
import ISMHelpers;

typedef TinyUniformSampleGenerator SampleGenerator;

// TODO: define on CPU side
#ifndef NUM_ISM_TEXTURE_ARRAYS
#define NUM_ISM_TEXTURE_ARRAYS 3
#endif

#ifndef DEBUG_PRINT_MODE
#define DEBUG_PRINT_MODE 0
#endif

cbuffer ismCB
{
    uint gLaunchWidth;
    uint gFirstSpotLightID;
    uint gTotalHighResIsmCount;
    uint gIsmSize;
}

StructuredBuffer<LightShadowMapData> gLightShadowDataBuffer;
StructuredBuffer<PackedIsmPointSample> gPointsBuffer;
Buffer<uint> gHighResIsmFaceIdxBuffer;

RWBuffer<uint> gHighIsmPerLightCountingBuffer;
RWBuffer<uint> gHighIsmPerFaceCountingBuffer;

// Low-res or High-res ISMs in light ranking order
RWTexture2DArray<uint> gSortedLightsISMsUint[NUM_ISM_TEXTURE_ARRAYS];

[numthreads(16, 16, 1)]
void renderIsmCS(uint3 threadID: SV_DispatchThreadID)
{
    printSetPixel(threadID.xy);

    uint globalThreadID = threadID.y * gLaunchWidth + threadID.x;

    SampleGenerator sg = SampleGenerator(threadID.xy, 6 * 13);

    // Get the point info
    IsmPointSample pointSample = unpackIsmPointSample(gPointsBuffer[globalThreadID]);
    GeometryInstanceID instanceID = { pointSample.instanceID };
    float3 posW = mul(float4(pointSample.pos, 1.0f), gScene.getWorldMatrix(instanceID)).xyz;

    uint highResIsmArrayIdx = uint(sampleNext1D(sg) * gTotalHighResIsmCount);
    uint globalFaceID = gHighResIsmFaceIdxBuffer[highResIsmArrayIdx];
    uint2 localLightIDAndFaceIdx = getLocalLightIdAndFaceIdx(globalFaceID, gFirstSpotLightID);
    uint selectedLightID = localLightIDAndFaceIdx.x;
    uint faceIdx = localLightIDAndFaceIdx.y;

    // Get the sample light info
    LightShadowMapData lightShadowData = gLightShadowDataBuffer[selectedLightID];

    // Project the point to the light face
    float3 lightCenterW = lightShadowData.centerPosW;
    float3 lightToPoint = posW - lightCenterW;
    float4x4 lightView = lightShadowData.lightFaceData[faceIdx].viewMat;
    float3 posLightView = mul(float4(posW, 1.0f), lightView).xyz;
    float4 posLightClipping = mul(float4(posLightView, 1.0f), lightShadowData.persProjMat);
    float3 posLightNDC = posLightClipping.xyz / posLightClipping.w;
    faceIdx = posW.y <= lightCenterW.y ? 0 : 1;
    float4x4 viewMat = lightShadowData.lightFaceData[3 - faceIdx].viewMat;
    posLightView = mul(float4(posW, 1.0f), viewMat).xyz;
    posLightNDC = paraboloidProjection(posLightView, lightShadowData.nearFarPlane.x, lightShadowData.nearFarPlane.y);
    float3 ismProjCoord = float3(posLightNDC.xy * float2(0.5f, -0.5f) + 0.5f, posLightNDC.z);

    // Cull the point if it's outside the texture space
    if (any(ismProjCoord.xy < 0.0f) || any(ismProjCoord.xy > 1.0f)) return;

    // Cull the point if it is behind the near plane. Note: view direction is -z
    if (posLightView.z > -lightShadowData.nearFarPlane.x) return;

#if DEBUG_PRINT_MODE
    if (gUseHighResISM)
        InterlockedAdd(gHighIsmPerLightCountingBuffer[selectedLightID], 1);
    else
        InterlockedAdd(gPerLightCountingBuffer[selectedLightID], 1);

    // Count if it is a valid point
    uint globalIsmID = selectedLightID < gFirstSpotLightID ? kShadowMapsPerPointLight * selectedLightID + faceIdx :
            kShadowMapsPerPointLight * gFirstSpotLightID + (selectedLightID - gFirstSpotLightID);
    if (gUseHighResISM)
        InterlockedAdd(gHighIsmPerFaceCountingBuffer[globalIsmID], 1);
    else
        InterlockedAdd(gPerFaceCountingBuffer[globalIsmID], 1);
#endif

    // Transform to pixel position in ISM texture
    uint2 ismTexPixelPos = uint2(ismProjCoord.xy * gIsmSize);
    float depth = ismProjCoord.z;

    // Perform depth testing using atomic operation
    uint3 ismTexArrayLoc = uint3(ismTexPixelPos, 0);
    ismTexArrayLoc.z = lightShadowData.lightFaceData[faceIdx].highResIsmTexArrayIdx;
    uint whichTexArray = lightShadowData.lightFaceData[faceIdx].whichHighResIsmTexArray;
    uint originDepth;
    InterlockedMin(gSortedLightsISMsUint[whichTexArray][ismTexArrayLoc], asuint(depth), originDepth);
}
