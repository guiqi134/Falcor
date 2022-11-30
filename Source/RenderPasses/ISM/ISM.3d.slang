#include "HostDeviceSharedDefinitions.h"

import Scene.Raster;
import Utils.Sampling.TinyUniformSampleGenerator;
import Utils.Debug.PixelDebug;
import ISMHelpers;

#define NUM_CONTROL_POINTS 3

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

struct GSOut
{
    float3 posW : POSW;
    float3 posView : PosView;
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

#if GS_OUT_STREAM == 0 || GS_OUT_STREAM == 1
[maxvertexcount(6)]
#else
[maxvertexcount(2)]
#endif
void gsMain(triangle VertexOut gsIn[3], uint primitiveID : SV_PrimitiveID,
#if GS_OUT_STREAM == 0
    inout TriangleStream<GSOut> outStream
#else
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
        // outStream.RestartStrip();
    }
#else
    // Triangle center position in world space
    float3 triCenterW = (gsIn[0].posW + gsIn[1].posW + gsIn[2].posW) / 3.0f;

    uint lightID = uint(sampleNext1D(sg) * gTotalLightCount);
    LightShadowData lightShadowData = gLightShadowDataBuffer[lightID];

    // // Triangle area
    // float triArea = 0.5f * length(cross(gsIn[0].posW, gsIn[1].posW));

    // Compute the max length for center to all three vertices
    float pointRadius = max(max(length(triCenterW - gsIn[0].posW), length(triCenterW - gsIn[1].posW)), length(triCenterW - gsIn[2].posW));

    for (uint d = 0; d < 2; d++)
    {
        float4x4 viewMat = lightShadowData.viewMats[d];
        float3 posLightView = mul(float4(triCenterW, 1.0f), viewMat).xyz;
        float3 posLightClip = paraboloidProjection(posLightView, gLightNearFar.x, gLightNearFar.y);

        gsOut.rtIndex = 2 * lightID + d;
        gsOut.posW = triCenterW;
        gsOut.posView = posLightView;
        gsOut.posH = float4(posLightClip, 1.0f);
        outStream.Append(gsOut);
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
