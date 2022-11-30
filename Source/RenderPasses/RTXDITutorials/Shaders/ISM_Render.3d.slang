#include "../HostDeviceSharedDefinitions.h"

import Scene.Raster;
import Utils.Sampling.TinyUniformSampleGenerator;
import Utils.Debug.PixelDebug;
import Utils.Math.MathHelpers;
import ISMHelpers;

#define MAX_EXTRA_POINTS 16
#define MAX_VERTEX_COUNT 38
#define GS_OUT_STREAM 1
#define RANDOM_OPTION 1


typedef TinyUniformSampleGenerator SampleGenerator;

cbuffer ismCB
{
    uint gTotalLightsCount;
    uint gTotalIsmCount;
    uint gIsmPerLight;
    uint2 gScreenSize;
    float gBaseTriSize;
}

StructuredBuffer<LightShadowMapData> gLightShadowDataBuffer;
Buffer<uint> gIsmLightIndexBuffer;
RWBuffer<uint> gExtraPointsCountBuffer;
// RWBuffer<float4> gPointsBuffer;
// RWBuffer<uint> gCounterBuffer; // Counter for each light, size must larger than total light count

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
    float2 lightNearFar : LightNearFar;
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
[maxvertexcount(MAX_VERTEX_COUNT)] // 2 * (3 + 32)
#else
[maxvertexcount(2)]
#endif
void gsMain(triangle VertexOut gsIn[3], uint primitiveID : SV_PrimitiveID, uint instanceID : SV_GSInstanceID,
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

    /*
    Iusses:
        1. beacuse some triangles (~6) will share the same vertex, so if we use one vertex screen position, then
        approximately every 6 triangles will the same RNG. And all those points will be the same in those triangles.
        They will all go to the same light
        2. the screen position depends on camera. This is why we have so many flickering due to camera motion
    */

    // SampleGenerator sg = SampleGenerator(pixelPos, 1 * 13);

    SampleGenerator sg = SampleGenerator(uint2(instanceID + 1, primitiveID + 1), 1 * 13);

    // Get random range which is total ISM lights count (buffer size)
    uint bufferSize;
    gIsmLightIndexBuffer.GetDimensions(bufferSize);

#if GS_OUT_STREAM == 0
    // // Pick a random light for each triangle.
    // uint lightID = uint(sampleNext1D(sg) * gTotalLightsCount);
    // LightShadowMapData lightShadowData = gLightShadowDataBuffer[lightID];

    // // Make sure we don't pick the light that already has a (reusing) shadow map
    // for (uint i = 0; i < 9 && lightShadowData.shadowMapType != 0; i++)
    // {
    //     lightID = uint(sampleNext1D(sg) * gTotalLightsCount);
    //     lightShadowData = gLightShadowDataBuffer[lightID];
    // }

    // for (uint d = 0; d < gIsmPerLight; d++)
    // {
    //     float4x4 viewMat = lightShadowData.viewMats[d];
    //     gsOut.rtIndex = gIsmPerLight * lightID + d;
    //     for (uint v = 0; v < 3; v++)
    //     {
    //         float3 posLightView = mul(float4(gsIn[v].posW, 1.0f), viewMat).xyz;
    //         float3 posLightClip = paraboloidProjection(posLightView, lightShadowData.nearFarPlane.x, lightShadowData.nearFarPlane.y);
    //         gsOut.posW = gsIn[v].posW;
    //         gsOut.posView = posLightView;
    //         gsOut.lightNearFar = lightShadowData.nearFarPlane;
    //         gsOut.posH = float4(posLightClip, 1.0f);
    //         outStream.Append(gsOut);
    //     }
    //     outStream.RestartStrip();
    // }
#elif GS_OUT_STREAM == 1
    // Triangle area
    float triSize = 0.5f * length(cross(gsIn[0].posW, gsIn[1].posW));

    // Adaptively generate more points
    uint numPoints = 3 + clamp(0, MAX_EXTRA_POINTS, uint(triSize / gBaseTriSize));
    InterlockedAdd(gExtraPointsCountBuffer[numPoints - 3], 1);

    for (uint v = 0; v < numPoints; v++)
    {
// TODO: remove
#if RANDOM_OPTION == 0
        // Pick a random buffer index
        // -> If using this approach, there will be lots of flickering across frames
        uint bufferIdx = uint(sampleNext1D(sg) * bufferSize);
        // uint bufferIdx = uint(sampleNext1D(sg) * 8); // Only pick first 12 lights
        uint lightID = gIsmLightIndexBuffer[bufferIdx];
        LightShadowMapData lightShadowData = gLightShadowDataBuffer[lightID];
#else
        // Or directly pick a random light ID
        uint lightID = uint(sampleNext1D(sg) * gTotalLightsCount);
        LightShadowMapData lightShadowData = gLightShadowDataBuffer[lightID];
#endif

        for (uint d = 0; d < gIsmPerLight; d++)
        {
            // Directly get the world position for first 3 vertices and randomly interpolate inside the triangle for the rest
            float3 vertexPosW;
            if (v < 3)
            {
                vertexPosW = gsIn[v].posW;
            }
            else
            {
                // TODO: change to low discrepency sequence (like R2 sequence)
                float3 barycentrics = sample_triangle(sampleNext2D(sg));
                vertexPosW = barycentrics.x * gsIn[0].posW + barycentrics.y * gsIn[1].posW + barycentrics.z * gsIn[2].posW;
            }

            float4x4 viewMat = lightShadowData.viewMats[d];
            float3 posLightView = mul(float4(vertexPosW, 1.0f), viewMat).xyz;
            float3 posLightClip = paraboloidProjection(posLightView, lightShadowData.nearFarPlane.x, lightShadowData.nearFarPlane.y);

            gsOut.rtIndex = lightShadowData.shadowMapType == 2 ? lightShadowData.startIndex + d : gTotalIsmCount - 1;
            gsOut.posW = vertexPosW;
            gsOut.posView = posLightView;
            gsOut.lightNearFar = lightShadowData.nearFarPlane;
            gsOut.posH = float4(posLightClip, 1.0f);
            outStream.Append(gsOut);
        }
        outStream.RestartStrip();
    }
#else
    // // Triangle center position in world space
    // float3 triCenterW = (gsIn[0].posW + gsIn[1].posW + gsIn[2].posW) / 3.0f;

    // uint lightID = uint(sampleNext1D(sg) * gTotalLightsCount);
    // LightShadowMapData lightShadowData = gLightShadowDataBuffer[lightID];

    // // Make sure we don't pick the light that already has a shadow map
    // for (uint i = 0; i < 9 && lightShadowData.shadowMapType != 0; i++)
    // {
    //     lightID = uint(sampleNext1D(sg) * gTotalLightsCount);
    //     lightShadowData = gLightShadowDataBuffer[lightID];
    // }

    // // Compute the max length for center to all three vertices
    // float pointRadius = max(max(length(triCenterW - gsIn[0].posW), length(triCenterW - gsIn[1].posW)), length(triCenterW - gsIn[2].posW));

    // for (uint d = 0; d < gIsmPerLight; d++)
    // {
    //     float4x4 viewMat = lightShadowData.viewMats[d];
    //     float3 posLightView = mul(float4(triCenterW, 1.0f), viewMat).xyz;
    //     float3 posLightClip = paraboloidProjection(posLightView, lightShadowData.nearFarPlane.x, lightShadowData.nearFarPlane.y);

    //     gsOut.rtIndex = gIsmPerLight * lightID + d;
    //     gsOut.posW = triCenterW;
    //     gsOut.posView = posLightView;
    //     gsOut.lightNearFar = lightShadowData.nearFarPlane;
    //     gsOut.posH = float4(posLightClip, 1.0f);
    //     outStream.Append(gsOut);
    // }
#endif
}

PSOut psMain(GSOut psIn)
{
    // printSetPixel(uint2(psIn.posH.xy));

    PSOut psOut;
    float2 lightNearFar = psIn.lightNearFar;

    print("psIn.posH = ", psIn.posH);
    print("lightNearFar = ", lightNearFar);

    // Clip the pixel if located behind near plane (zView > 0.0f in RH), because we set posH.w = 1.0f in the GS,
    // rasterizer cannot use posH.w to do the clipping like in perspective projection where posH.w = -z.
    clip(-(psIn.posView.z + lightNearFar.x));

    // Equal to length(posView)
    float depthView = psIn.posH.z * (lightNearFar.y - lightNearFar.x) + lightNearFar.x;

    print("psIn.posView = ", psIn.posView);
    // print("psIn.posParaProj = ", psIn.posParaProj);
    // print("depthView = ", depthView);

    psOut.ism = psIn.posH.z;
    psOut.depth = psIn.posH.z;

    return psOut;
}

