#include "HostDeviceSharedDefinitions.h"

import Scene.Raster;
import Scene.Scene;
import ISMHelpers;

#define NUM_CONTROL_POINTS 3

cbuffer PerFrameCB
{
    float4 gColor;
    float2 gNearFarPlane;
    uint gLightID;
};

StructuredBuffer<LightShadowData> gLightShadowDataBuffer;

// struct VSIn
// {
//     float3 pos : POSITION;
//     // float4 normal : NORMAL;
//     // float4 color : COLOR;
// }

// struct VSOut
// {
//     float4 posH : SV_Position;
// }

struct HS_PatchConstantDataOut
{
    float edgeTessFactor[3] : SV_TessFactor;      // [4] for quad
    float insideTessFactor : SV_InsideTessFactor; // [2] for quad
}

struct HS_ControlPointsOut
{
    float4 posH : SV_POSITION;
}

struct DSOut
{
    float4 posH : SV_POSITION;
}

float paraboloidProjToView(float depth, float near, float far)
{
    return depth * (far - near) + near;
}

float3 interpolate3D(float3 v0, float3 v1, float3 v2, float3 uvw)
{
    return v0 * uvw.x + v1 * uvw.y + v2 * uvw.z;
}

VSOut vsMain(VSIn vIn)
{
    VSOut vsOut;
    vsOut = defaultVS(vIn);
    // vsOut.posH = float4(vIn.pos, 1.0f);

    return vsOut;
}

// Hull Shader Patch Constant Fucntion
HS_PatchConstantDataOut calcPatchConstants() // uint patchID : SV_PrimitiveID
{
    HS_PatchConstantDataOut data;

    data.edgeTessFactor[0] = 2.0f;
    data.edgeTessFactor[1] = 2.0f;
    data.edgeTessFactor[2] = 2.0f;
    data.insideTessFactor = 2.0f;

    return data;
}

[domain("tri")]
[partitioning("integer")]
[outputtopology("triangle_cw")] // should this be ccw?
[outputcontrolpoints(3)]
[patchconstantfunc("calcPatchConstants")]
HS_ControlPointsOut hsMain(InputPatch<VSOut, NUM_CONTROL_POINTS> ip, uint i: SV_OutputControlPointID, uint patchID : SV_PrimitiveID)
{
    HS_ControlPointsOut hsOut;
    hsOut.posH = ip[i].posH;

    return hsOut;
}

[domain("tri")]
DSOut dsMain(HS_ControlPointsOut input, float3 uvw : SV_DomainLocation, const OutputPatch<HS_ControlPointsOut, NUM_CONTROL_POINTS> patch)
{
    DSOut dsOut;

    // Barycentric interpolation
    // dsOut.posH = mul(float4(interpolate3D(patch[0].posH.xyz, patch[1].posH.xyz, patch[2].posH.xyz, uvw), 1.0f), gScene.camera.getViewProj());
    dsOut.posH = patch[0].posH;

    return dsOut;
}

float4 psMain() : SV_TARGET
{
    return gColor;
}
