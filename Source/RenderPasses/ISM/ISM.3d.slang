#include "HostDeviceSharedDefinitions.h"

import Scene.Raster;
import Utils.Sampling.TinyUniformSampleGenerator;
import Utils.Debug.PixelDebug;
import ISMHelpers;

#define NUM_CONTROL_POINTS 3

static const float tessFactor = 1.0f;
typedef TinyUniformSampleGenerator SampleGenerator;

cbuffer ismCB
{
    float4x4 gCameraView;
    float4 gConstColor;
    uint gTotalLightCount;
    uint2 gScreenSize;
    float2 gLightNearFar;
    float2 gCameraNearFar;
    bool gWireframe;
}

StructuredBuffer<LightShadowData> gLightShadowDataBuffer;
RWBuffer<float4> gPointsBuffer;
RWBuffer<uint> gCounterBuffer; // Counter for each light, size must larger than total light count

struct VertexOut
{
    float3 posW : POSW;
    float3 normalW : NORMAL;

    nointerpolation GeometryInstanceID instanceID : INSTANCE_ID; ///< Geometry instance ID.
    float4 posH : SV_POSITION;
}

struct HS_PatchConstantDataOut
{
    float edgeTessFactor[3] : SV_TessFactor; // [4] for quad
    float insideTessFactor : SV_InsideTessFactor; // [2] for quad
}

struct HS_ControlPointsOut
{
    // float3 posW : POSW;        ///< Position in world space.
    // float3 normalW : NORMAL;   ///< Shading normal in world space (not normalized!).

    float4 posH : SV_POSITION;
}

struct DSOut
{
    // float3 posW : POSW;
    // float3 normalW : NORMAL;

    float4 posH : SV_POSITION;
}

struct GSOut
{
    float3 posW : POSW;
    float3 posView : PosView;
    // float4 posPersProj : PersProj;
    // float4 posParaProj : ParaProj;
    // float3 normalW : NORMAL;
    uint rtIndex : SV_RenderTargetArrayIndex;
    // uint triangleIndex : SV_PrimitiveID;

    float4 posH : SV_POSITION;
}

struct PSOut
{
    float ism : SV_Target0;
    float depth : SV_Depth;
}

VertexOut vsMain(VSIn vIn)
{
    VertexOut vsOut;

    const GeometryInstanceID instanceID = { vIn.instanceID };

    float4x4 worldMat = gScene.getWorldMatrix(instanceID);
    float3 posW = mul(float4(vIn.pos, 1.f), worldMat).xyz;
    vsOut.posW = posW;
    vsOut.posH = float4(posW, 1.0f);
    vsOut.normalW = mul(vIn.unpack().normal, gScene.getInverseTransposeWorldMatrix(instanceID));
    vsOut.instanceID = instanceID;

    return vsOut;
}

// Hull Shader Patch Constant Fucntion
HS_PatchConstantDataOut calcPatchConstants() // uint patchID : SV_PrimitiveID
{
    HS_PatchConstantDataOut data;

    data.edgeTessFactor[0] = 1.0f;
    data.edgeTessFactor[1] = 1.0f;
    data.edgeTessFactor[2] = 1.0f;
    data.insideTessFactor = 1.0f;

    return data;
}

[domain("tri")]
[partitioning("integer")]
[outputtopology("triangle_cw")] // should this be ccw?
[outputcontrolpoints(3)]
[patchconstantfunc("calcPatchConstants")]
HS_ControlPointsOut hsMain(InputPatch<VSOut, NUM_CONTROL_POINTS> ip, uint i : SV_OutputControlPointID/*, uint patchID : SV_PrimitiveID*/)
{
    HS_ControlPointsOut hsOut;

    // hsOut.posW = ip[i].posW;
    // hsOut.normalW = ip[i].normalW;
    hsOut.posH = ip[i].posH;
    // hsOut.tangentW = ip[i].tangentW;
    // hsOut.texC = ip[i].texC;
    // hsOut.instanceID = ip[i].instanceID;
    // hsOut.materialID = ip[i].materialID;

    return hsOut;
}

float3 interpolate3D(float3 v0, float3 v1, float3 v2, float3 uvw)
{
    return v0 * uvw.x + v1 * uvw.y + v2 * uvw.z;
}

[domain("tri")]
DSOut dsMain(HS_ControlPointsOut input, float3 uvw : SV_DomainLocation, const OutputPatch<HS_ControlPointsOut, NUM_CONTROL_POINTS> patch)
{
    DSOut dsOut;

    // Barycentric interpolation
    // dsOut.posW = interpolate3D(patch[0].posW, patch[1].posW, patch[2].posW, uvw);
    // dsOut.normalW = interpolate3D(patch[0].normalW, patch[1].normalW, patch[2].normalW, uvw);
    dsOut.posH = mul(float4(interpolate3D(patch[0].posH.xyz, patch[1].posH.xyz, patch[2].posH.xyz, uvw), 1.0f), gScene.camera.getViewProj());

    // dsOut.instanceID = patch[0].instanceID;
    // dsOut.materialID = patch[0].materialID;

    return dsOut;
}

// [maxvertexcount(1)]
// void gsOnePoint(triangle VertexOut gsIn[3], uint primitiveID: SV_PrimitiveID, inout PointStream<GSOut> outStream)
// {
//     // Triangle center position in world space
//     float3 triCenterW = (gsIn[0].posW + gsIn[1].posW + gsIn[2].posW) / 3.0f;
//     float3 triCenterNormal = (gsIn[0].normalW + gsIn[1].normalW + gsIn[2].normalW) / 3.0f;
//     float4 triCenterClip = mul(float4(triCenterW, 1.0f), gScene.camera.getViewProj());
//     float3 triCenterNdc = triCenterClip.xyz / triCenterClip.w;
//     uint2 pixelPos = uint2(gScreenSize * (triCenterNdc.xy * float2(0.5, -0.5) + float2(0.5f)));

//     // // Triangle area
//     // float triArea = 0.5f * length(cross(gsIn[0].posW, gsIn[1].posW));

//     // Compute the max length for center to all three vertices
//     float pointRadius = max(max(length(triCenterW - gsIn[0].posW), length(triCenterW - gsIn[1].posW)), length(triCenterW - gsIn[2].posW));

//     GSOut gsOut;
//     LightShadowData lightShadowData = gLightShadowDataBuffer[0];
//     float3 vertexView = mul(float4(triCenterW, 1.0f), lightShadowData.frontViewMat).xyz;
//     float3 vertexClip = paraboloidProjection(vertexView, gLightNearFar.x, gLightNearFar.y);
//     gsOut.posW = triCenterW;
//     gsOut.normalW = triCenterNormal;
//     gsOut.posH = float4(vertexClip, 1.0f);
//     outStream.Append(gsOut);
// }


[maxvertexcount(6)]
void gsMain(triangle VertexOut gsIn[3], uint primitiveID : SV_PrimitiveID, 
#if GS_OUT_STREAM == 0
    inout TriangleStream<GSOut> outStream
#elif GS_OUT_STREAM == 1
    inout PointStream<GSOut> outStream
#endif
)
{
    GSOut gsOut;

    // Create random number generator
    float3 posW = gsIn[0].posW;
    float4 posClip = mul(float4(posW, 1.0f), gScene.camera.getViewProj());
    float3 posNdc = posClip.xyz / posClip.w;
    uint2 pixelPos = uint2(gScreenSize * (posNdc.xy * float2(0.5, -0.5) + float2(0.5f)));
    SampleGenerator sg = SampleGenerator(pixelPos, 1 * 13);

#if GS_OUT_STREAM == 0
    // Pick a random light for each triangle
    uint lightID = uint(sampleNext1D(sg) * gTotalLightCount);
    // lightID = 0;
    LightShadowData lightShadowData = gLightShadowDataBuffer[lightID];
    for (uint d = 0; d < 2; d++)
    {
        float4x4 viewMat = lightShadowData.viewMats[d];
        gsOut.rtIndex = 2 * lightID + d;
        for (uint v = 0; v < 3; v++)
        {
            float3 posLightView = mul(float4(gsIn[v].posW, 1.0f), viewMat).xyz;
            float3 posLightClip = paraboloidProjection(posLightView, gLightNearFar.x, gLightNearFar.y);
            gsOut.posW = gsIn[v].posW;
            gsOut.posView = posLightView;
            gsOut.posH = float4(posLightClip, 1.0f);
            outStream.Append(gsOut);
        }
        outStream.RestartStrip();
    }
#elif GS_OUT_STREAM == 1
    for (uint v = 0; v < 3; v++)
    {
        // Pick a random light for each vertex
        uint lightID = uint(sampleNext1D(sg) * gTotalLightCount);
        // lightID = 0;
        LightShadowData lightShadowData = gLightShadowDataBuffer[lightID];

        for (uint d = 0; d < 2; d++)
        {
            float4x4 viewMat = lightShadowData.viewMats[d];
            float3 posLightView = mul(float4(gsIn[v].posW, 1.0f), viewMat).xyz;
            float3 posLightClip = paraboloidProjection(posLightView, gLightNearFar.x, gLightNearFar.y);
            
            gsOut.rtIndex = 2 * lightID + d;
            gsOut.posW = gsIn[v].posW;
            gsOut.posView = posLightView;
            gsOut.posH = float4(posLightClip, 1.0f);
            outStream.Append(gsOut);
        }
        outStream.RestartStrip();
    }
#endif
}

PSOut psMain(GSOut psIn)
{
    // printSetPixel(uint2(psIn.posH.xy));

    PSOut psOut;
    print("psIn.posH = ", psIn.posH);

    // Clip the pixel if located behind near plane (zView > 0.0f in RH), because we set posH.w = 1.0f in the GS, 
    // rasterizer cannot use posH.w to do the clipping like in perspective projection where posH.w = -z.
    clip(-(psIn.posView.z + gLightNearFar.x));

    // Equal to length(posView)
    float depthView = psIn.posH.z * (gLightNearFar.y - gLightNearFar.x) + gLightNearFar.x;

    print("psIn.posView = ", psIn.posView);
    // print("psIn.posParaProj = ", psIn.posParaProj);
    // print("depthView = ", depthView);

    psOut.ism = psIn.posH.z;
    psOut.depth = psIn.posH.z;

    return psOut;
}