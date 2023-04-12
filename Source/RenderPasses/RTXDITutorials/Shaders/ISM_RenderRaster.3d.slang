// #include "../HostDeviceSharedDefinitions.h"
#include "Utils/Math/MathConstants.slangh"

import Scene.Raster;
import Utils.Sampling.TinyUniformSampleGenerator;
import Utils.Debug.PixelDebug;
import Utils.Math.MathHelpers;
import ISMHelpers;

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
    uint gFirstSpotLightID;
    uint gIsmSize;
}

StructuredBuffer<LightShadowMapData> gLightShadowDataBuffer;

RWBuffer<uint> gPerLightCountingBuffer;
RWBuffer<uint> gPerFaceCountingBuffer;

struct VertexIn
{
    float3 pos : POSITION;
    uint packedLightInstanceID : PACKED_LIGHT_ID_INSTANCE_ID;
}

struct VertexOut
{
    float3 posW : POSW;
    float3 posLightView : POS_LIGHT_VIEW;
    float lightNearPlane : LIGHT_NEAR_PLANE;
    nointerpolation GeometryInstanceID instanceID : INSTANCE_ID; ///< Geometry instance ID.
    uint rtIndex : SV_RenderTargetArrayIndex;
    float4 posH : SV_POSITION;
}

struct PSOut
{
    float ism : SV_Target0;
    float depth : SV_Depth;
}

VertexOut vsMain(VertexIn vIn)
{
    VertexOut vsOut;

    // Get the point info
    uint selectedLightID = vIn.packedLightInstanceID & 0xFFFF;
    uint id = vIn.packedLightInstanceID >> 16;
    GeometryInstanceID instanceID = { id };
    float3 posW = mul(float4(vIn.pos, 1.0f), gScene.getWorldMatrix(instanceID)).xyz;
    vsOut.instanceID = instanceID;
    vsOut.posW = posW;

    // Get the sample light info
    LightShadowMapData lightShadowData = gLightShadowDataBuffer[selectedLightID];

    // Project the point to the light face
    float3 lightCenterW = lightShadowData.centerPosW;
    float3 lightToPoint = posW - lightCenterW;
    uint faceIdx = selectedLightID < gFirstSpotLightID ? getFaceIndex(normalize(lightToPoint)) : 0;
    float4x4 lightView = lightShadowData.lightFaceData[faceIdx].viewMat;
    float3 posLightView = mul(float4(posW, 1.0f), lightView).xyz;
    vsOut.posH = mul(float4(posLightView, 1.0f), lightShadowData.persProjMat);
    vsOut.posLightView = posLightView;

    // Set point to ISM
    vsOut.rtIndex = lightShadowData.lightFaceData[faceIdx].ismTexArrayIdx;
    vsOut.lightNearPlane = lightShadowData.nearFarPlane.x;

    return vsOut;
}

PSOut psMain(VertexOut psIn)
{
    // printSetPixel(uint2(psIn.posH.xy));

    // let lod = ExplicitLodTextureSampler(0.f);
    // if (alphaTest(psIn.vsOut, psIn.triangleIndex, lod)) discard;

    float2 ismProjCoord = psIn.posH.xy / gIsmSize;

    // Cull the point if it's outside the texture space
    if (any(ismProjCoord.xy < 0.0f) || any(ismProjCoord.xy > 1.0f)) discard;

    // Cull the point if it is behind the near plane. Note: view direction is -z
    if (psIn.posLightView.z > -psIn.lightNearPlane) discard;

    PSOut psOut;
    psOut.ism = psIn.posH.z;
    psOut.depth = psIn.posH.z;

    return psOut;
}

