#include "HostDeviceSharedDefinitions.h"

import Scene.Raster;
import ISMHelpers;

cbuffer PerFrameCB
{
    float4 gColor;
    float2 gNearFarPlane;
    uint gLightID;
};

StructuredBuffer<LightShadowData> gLightShadowDataBuffer;

float paraboloidProjToView(float depth, float near, float far)
{
    return depth * (far - near) + near;
}

VSOut vsMain(VSIn vIn)
{
    VSOut vsOut;
    vsOut = defaultVS(vIn);

    float4x4 lightViewMat = gLightShadowDataBuffer[gLightID].viewMats[0];
    float3 vertexView = mul(float4(vsOut.posW, 1.0f), lightViewMat).xyz;
    float3 vertexClip = paraboloidProjection(vertexView, gNearFarPlane.x, gNearFarPlane.y);
    vsOut.posH = float4(vertexClip, 1.0f);

    return vsOut;
}

float4 psMain(VSOut psIn) : SV_TARGET
{
    // Cannot do clipping in wireframe mode?
    // float depthView = paraboloidProjToView(psIn.posH.z, gNearFarPlane.x, gNearFarPlane.y);
    // clip(-(depthView + gNearFarPlane.x));

    return gColor;
}
