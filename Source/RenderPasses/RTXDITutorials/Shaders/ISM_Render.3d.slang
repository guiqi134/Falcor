#include "../HostDeviceSharedDefinitions.h"
#include "Utils/Math/MathConstants.slangh"

import Scene.Raster;
import Utils.Sampling.TinyUniformSampleGenerator;
import Utils.Debug.PixelDebug;
import Utils.Math.MathHelpers;
import ISMHelpers;

#define MAX_EXTRA_POINTS 32
#define MAX_VERTEX_COUNT 35
#define GS_OUT_STREAM 1

#define TOTAL_LIGHTS 132


typedef TinyUniformSampleGenerator SampleGenerator;

cbuffer ismCB
{
    uint gTotalLightsCount;
    uint gTotalIsmCount;
    uint gIsmPerLight;
    uint2 gScreenSize;
    float gBaseTriSize;
    bool gAdaptiveISM;
    uint gIsmLightSamplingMode;
}

StructuredBuffer<LightShadowMapData> gLightShadowDataBuffer;
Buffer<float> gLightPdfBuffer;
Buffer<float> gLightCdfBuffer;
RWBuffer<uint> gExtraPointsCountBuffer;
// RWBuffer<float4> gPointsBuffer;
RWBuffer<uint> gCounterBuffer; // Counter for each light

struct VertexOut
{
    float3 posW : POSW;
    float3 normalW : NORMAL;

    nointerpolation GeometryInstanceID instanceID : INSTANCE_ID; ///< Geometry instance ID.
    float4 posH : SV_POSITION;
}

struct GSOut
{
    float3 posView : PosView;
    float lightNear : LightNear;
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

// Find interval using binary search -> std::upper_bound()
uint findInterval(Buffer<float> targetBuffer, float u)
{
    uint bufferSize;
    targetBuffer.GetDimensions(bufferSize);

    int first = 0;
    uint len = bufferSize;

    while (len > 0)
    {
        uint h = len >> 1;
        uint middle = first + h;
        if (u < gLightCdfBuffer[middle])
        {
            len = h;
        }
        else
        {
            first = (int)middle + 1;
            len -= h + 1;
        }
    }

    return clamp(first - 1, 0, bufferSize - 2);
}

uint findInterval(float cdfArray[TOTAL_LIGHTS + 1], uint size, float u)
{
    int first = 0;
    uint len = size;

    while (len > 0)
    {
        uint h = len >> 1;
        uint middle = first + h;
        if (u < cdfArray[middle])
        {
            len = h;
        }
        else
        {
            first = (int)middle + 1;
            len -= h + 1;
        }
    }

    return clamp(first - 1, 0, size - 2);
}

#if GS_OUT_STREAM == 0 || GS_OUT_STREAM == 1
[maxvertexcount(MAX_VERTEX_COUNT)] // 3 + MAX_EXTRA_POINTS
#else
[maxvertexcount(2)]
#endif
void gsMain(triangle VertexOut gsIn[3], uint primitiveID : SV_PrimitiveID, uint gsInstanceID : SV_GSInstanceID,
#if GS_OUT_STREAM == 0
    inout TriangleStream<GSOut> outStream
#else
    inout PointStream<GSOut> outStream
#endif
)
{
    GSOut gsOut;
    uint instanceID = gsIn[0].instanceID.index;

    printSetPixel(uint2(instanceID, primitiveID));

    // Create random number generator
    float3 posW = gsIn[0].posW;
    float4 posClip = mul(float4(posW, 1.0f), gScene.camera.getViewProj());
    float3 posNdc = posClip.xyz / posClip.w;
    uint2 pixelPos = uint2(gScreenSize * (posNdc.xy * float2(0.5, -0.5) + float2(0.5f)));

    print("gsIn[0].instanceID = ", gsIn[0].instanceID.index);

    /*
    Iusses:
        1. beacuse some triangles (~6) will share the same vertex, so if we use one vertex screen position, then
        approximately every 6 triangles will the same RNG. And all those points will be the same in those triangles.
        They will all go to the same light
        2. the screen position depends on camera. This is why we have so many flickering due to camera motion
    */
    // SampleGenerator sg = SampleGenerator(pixelPos, 1 * 13);

    // TODO: Change to per-instance RNG?
    SampleGenerator sg = SampleGenerator(uint2(instanceID + 1, primitiveID + 1), 1 * 13);
    SampleGenerator sgFixed = SampleGenerator(uint2(1, primitiveID + 1), 1 * 13);

    // Compute the distance PDF and then joint PDF
    float jointPdfArray[TOTAL_LIGHTS];
    float jointCdfArray[TOTAL_LIGHTS + 1];
    float jointSum = 0.0f;
    float3 triCenterPosW = (gsIn[0].posW + gsIn[1].posW + gsIn[2].posW) / 3.0f;

    if (gAdaptiveISM)
    {
        for (uint i = 0; i < gTotalLightsCount; i++)
        {
            LightShadowMapData lightShadowData = gLightShadowDataBuffer[i];

            float invDistSquare = 1.0 / pow(length(lightShadowData.centerPosW - triCenterPosW), 2);
            float jointPdf = lightShadowData.shadowMapType == 2 ? gLightPdfBuffer[i] * invDistSquare : 0.0f;
            // float jointPdf = gLightPdfBuffer[i] * invDistSquare;
            jointPdfArray[i] = jointPdf;
            jointSum += jointPdf;
        }

        // Normlize the joint PDF
        for (uint i = 0; i < gTotalLightsCount; i++)
        {
            jointPdfArray[i] /= jointSum;
        }

        // Build joint CDF
        float sum = 0.0f;
        jointCdfArray[0] = 0.0f;
        for (uint i = 1; i < gTotalLightsCount + 1; i++)
        {
            sum += jointPdfArray[i - 1];
            jointCdfArray[i] = sum;
        }
    }

    // Per-primitve base random
    float u = sampleNext1D(sg); // Note: for same primitive, this is fixed in every frame
    uint perPrimLightID = 0;

    // Adaptively generate more points.
    float triSize = 0.5f * length(cross(gsIn[0].posW, gsIn[1].posW));
    uint numPoints = 0;
    if (gIsmLightSamplingMode == 0)
    {
        numPoints = 3 + clamp(0, MAX_EXTRA_POINTS, uint(triSize / gBaseTriSize));
    }
    else
    {
        if (gAdaptiveISM)
        {
            perPrimLightID = findInterval(jointCdfArray, gTotalLightsCount + 1, u);
        }
        else
        {
            perPrimLightID = uint(u * gTotalLightsCount);
            // for (uint i = 0; i < 4; i++)
            // {
            //     if (gLightShadowDataBuffer[perPrimLightID].shadowMapType == 2) break;
            //     u = sampleNext1D(sg);
            //     perPrimLightID = uint(u * gTotalLightsCount);
            // }
        }

        float3 lightCenter = gLightShadowDataBuffer[perPrimLightID].centerPosW;
        float3 toLight = lightCenter - triCenterPosW;
        float distanceSquare = pow(length(toLight), 2);
        float3 triNormalW = normalize((gsIn[0].normalW + gsIn[1].normalW + gsIn[2].normalW) / 3.0f);
        float projectedSize = triSize * abs(dot(triNormalW, toLight)) / distanceSquare; // solid angle measure = A * cos / r^2 (r = distance to light)

        // How many points (texels) we need for a unit projected area
        uint ismSize = gLightShadowDataBuffer[perPrimLightID].shadowMapSize;
        uint numPointsUnitProjSize = uint(ismSize * ismSize / (2.0f * M_PI));

        numPoints = 3 + clamp(0, MAX_EXTRA_POINTS, uint(projectedSize * numPointsUnitProjSize * 0.12f));
    }

    InterlockedAdd(gExtraPointsCountBuffer[numPoints - 3], 1);

    // Render each point
    for (uint v = 0; v < numPoints; v++)
    {
        uint lightID = 0;

        // Per-point base or per-primitive base
        if (gIsmLightSamplingMode == 0)
        {
            u = sampleNext1D(sg);

            // Directly pick a random light ID
            if (gAdaptiveISM)
            {
                lightID = findInterval(jointCdfArray, gTotalLightsCount + 1, u);
            }
            else
            {
                lightID = uint(u * gTotalLightsCount);
                // for (uint i = 0; i < 4; i++)
                // {
                //     if (gLightShadowDataBuffer[lightID].shadowMapType == 2) break;
                //     u = sampleNext1D(sg);
                //     lightID = uint(u * gTotalLightsCount);
                // }
            }
        }
        else
        {
            lightID = perPrimLightID;
        }

        InterlockedAdd(gCounterBuffer[lightID], 1);

        LightShadowMapData lightShadowData = gLightShadowDataBuffer[lightID];

        // Directly get the world position for first 3 vertices and randomly interpolate inside the triangle for the rest
        float3 vertexPosW;
        if (v < 3)
        {
            vertexPosW = gsIn[v].posW;
        }
        else
        {
            // TODO: change to low discrepency sequence (like R2 sequence) and pre-computed those barycentrics
            // Note: we need to use different sg for sampling triangle to avoid getting different values
            float3 barycentrics = sample_triangle(sampleNext2D(sgFixed));
            print("barycentrics = ", barycentrics);
            vertexPosW = barycentrics.x * gsIn[0].posW + barycentrics.y * gsIn[1].posW + barycentrics.z * gsIn[2].posW;
            // print("vertexPosW = ", vertexPosW);
        }

        // Assign the point to the face
        float3 lightCenterW = lightShadowData.centerPosW;
        uint d = vertexPosW.y <= lightCenterW.y ? 0 : 1;
        float4x4 viewMat = lightShadowData.viewMats[3 - d];
        float3 posLightView = mul(float4(vertexPosW, 1.0f), viewMat).xyz;
        float3 posLightClip = paraboloidProjection(posLightView, lightShadowData.nearFarPlane.x, lightShadowData.nearFarPlane.y);

        gsOut.rtIndex = lightShadowData.ismArrayStartIndex + d;
        gsOut.posView = posLightView;
        gsOut.lightNear = lightShadowData.nearFarPlane.x;
        gsOut.posH = float4(posLightClip, 1.0f);
        outStream.Append(gsOut);
    }
    outStream.RestartStrip();
}

PSOut psMain(GSOut psIn)
{
    // printSetPixel(uint2(psIn.posH.xy));

    PSOut psOut;
    float lightNear = psIn.lightNear;
    lightNear = 0.001f;

    // Clip the pixel if located behind near plane (In RH system, "behind" means z value larger than nearZ = -nearPlane), because we set posH.w = 1.0f in the GS,
    // rasterizer cannot use posH.w to do the clipping like in perspective projection where posH.w = -z.
    clip(-(psIn.posView.z + lightNear)); // note: clip() function discards value < 0

    psOut.ism = psIn.posH.z;
    psOut.depth = psIn.posH.z;

    return psOut;
}

