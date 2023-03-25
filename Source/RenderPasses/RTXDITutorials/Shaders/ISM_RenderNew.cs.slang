// #include "../HostDeviceSharedDefinitions.h"
#include "Utils/Math/MathConstants.slangh"

import Scene.Scene;
import Utils.Sampling.TinyUniformSampleGenerator;
import Utils.Math.MathHelpers;
import Utils.Debug.PixelDebug;
import ISMHelpers;

// typedef TinyUniformSampleGenerator SampleGenerator;

// TODO: define on CPU side
#ifndef NUM_ISM_TEXTURE_ARRAYS
#define NUM_ISM_TEXTURE_ARRAYS 3
#endif

#ifndef DEBUG_PRINT_MODE
#define DEBUG_PRINT_MODE 0
#endif

cbuffer ismCB
{
    uint gTotalPointSamples;
    uint gLaunchWidth;
    uint gFirstSpotLightID;
    uint gIsmSize;
}

StructuredBuffer<LightShadowMapData> gLightShadowDataBuffer;
StructuredBuffer<PackedIsmPointSample> gPointsBuffer;

RWBuffer<uint> gPerLightCountingBuffer;
RWBuffer<uint> gPerFaceCountingBuffer;

// Low-res or High-res ISMs in light ranking order
RWTexture2DArray<uint> gSortedLightsISMsUint[NUM_ISM_TEXTURE_ARRAYS];

[numthreads(16, 16, 1)]
void renderIsmCS(uint3 threadID: SV_DispatchThreadID)
{
    printSetPixel(threadID.xy);

    uint pointBufferIdx = threadID.y * gLaunchWidth + threadID.x;
    if (pointBufferIdx >= gTotalPointSamples) return;

    // Get the point info
    IsmPointSample pointSample = unpackIsmPointSample(gPointsBuffer[pointBufferIdx]);
    GeometryInstanceID instanceID = { pointSample.instanceID };
    float3 posW = mul(float4(pointSample.pos, 1.0f), gScene.getWorldMatrix(instanceID)).xyz;
    uint selectedLightID = pointSample.selectedLightID;

    print("selectedLightID = ", selectedLightID);
    print("posW = ", posW);

    // Get the sample light info
    LightShadowMapData lightShadowData = gLightShadowDataBuffer[selectedLightID]; // optimize this!

    // Project the point to the light face
    float3 lightCenterW = lightShadowData.centerPosW;
    float3 lightToPoint = posW - lightCenterW;
    // uint faceIdx = getClosestFace(normalize(lightToPoint), lightShadowData); // optimize this!
    uint faceIdx = getFaceIndex(normalize(lightToPoint)); // optimize this!
    float4x4 lightView = lightShadowData.lightFaceData[faceIdx].viewMat;
    float3 posLightView = mul(float4(posW, 1.0f), lightView).xyz;
    float4 posLightClipping = mul(float4(posLightView, 1.0f), lightShadowData.persProjMat);
    float3 posLightNDC = posLightClipping.xyz / posLightClipping.w;
    float3 ismProjCoord = float3(posLightNDC.xy * float2(0.5f, -0.5f) + 0.5f, posLightNDC.z);
    print("ismProjCoord = ", ismProjCoord);

    // Cull the point if it's outside the texture space
    if (any(ismProjCoord.xy < 0.0f) || any(ismProjCoord.xy > 1.0f)) return;

    // Cull the point if it is behind the near plane. Note: view direction is -z
    if (posLightView.z > -lightShadowData.nearFarPlane.x) return;

#if DEBUG_PRINT_MODE
    InterlockedAdd(gPerLightCountingBuffer[selectedLightID], 1);

    // Count if it is a valid point
    uint globalIsmID = selectedLightID < gFirstSpotLightID ? kShadowMapsPerPointLight * selectedLightID + faceIdx :
        kShadowMapsPerPointLight * gFirstSpotLightID + (selectedLightID - gFirstSpotLightID);
    InterlockedAdd(gPerFaceCountingBuffer[globalIsmID], 1);
#endif

    // Transform to pixel position in ISM texture
    uint2 ismTexPixelPos = uint2(ismProjCoord.xy * gIsmSize);
    float depth = ismProjCoord.z;

    // Perform depth testing using atomic operation
    uint3 ismTexArrayLoc = uint3(ismTexPixelPos, lightShadowData.lightFaceData[faceIdx].ismTexArrayIdx);
    uint whichTexArray = lightShadowData.lightFaceData[faceIdx].whichIsmTexArray;
    uint originDepth;
    print("ismTexArrayLoc = ", ismTexArrayLoc);
    InterlockedMin(gSortedLightsISMsUint[whichTexArray][ismTexArrayLoc], asuint(depth), originDepth);
}
