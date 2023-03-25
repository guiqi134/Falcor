#include "rtxdi/RtxdiMath.hlsli"

import Utils.Debug.PixelDebug;
import Scene.Scene;
import Utils.Math.MathHelpers;
import Utils.Sampling.TinyUniformSampleGenerator;
import ISMHelpers;

typedef TinyUniformSampleGenerator SampleGenerator;

cbuffer presampleCB
{
    uint gTotalPointSamples;
    uint gTotalNonEmissiveTriCount;
    uint gLaunchWidth;
    uint gTotalIsmCount;
    uint gTotalHighResIsmCount;
    uint gFirstSpotLightID;
}

RWBuffer<uint> gNumPointsForLight;

[numthreads(16, 16, 1)]
void presampleLight(uint3 threadID: SV_DispatchThreadID)
{
    uint globalThreadID = threadID.y * gLaunchWidth + threadID.x;
    if (globalThreadID >= gTotalPointSamples) return;

    SampleGenerator sg = SampleGenerator(threadID.xy, 5 * 13);

    // Sample random face and then cast to light ID
    uint globalFaceID = uint(gTotalIsmCount * sampleNext1D(sg));
    uint2 localLightIDAndFaceIdx = getLocalLightIdAndFaceIdx(globalFaceID, gFirstSpotLightID);
    uint selectedLightID = localLightIDAndFaceIdx.x;

    InterlockedAdd(gNumPointsForLight[selectedLightID], 1);
}

enum class TriangleSampleType : uint
{
    TriangleThreeVertices = 0,
    AllTriangles = 1,
    OnlySmallTriangles = 2
}

Texture2D<float> gTriAreaPDFTexture;
Texture2D<float> gTriAreaPDFTexOnlyForSmallTri;
Buffer<uint2> gNonEmisInstTriOffsetBuffer;
Buffer<uint> gLightNumPointsOffsetBuffer;
Buffer<float> gSampleTypeCdfBuffer;
RWStructuredBuffer<PackedIsmPointSample> gPointsBuffer;

RWBuffer<uint> gDebugLightID;
RWBuffer<uint> gTypeCountBuffer;

[numthreads(16, 16, 1)]
void presampleTriangle(uint3 threadID : SV_DispatchThreadID)
{
    printSetPixel(threadID.xy);

    uint pointsBufferIdx = threadID.y * gLaunchWidth + threadID.x;
    if (pointsBufferIdx >= gTotalPointSamples) return;

    IsmPointSample pointSample;

    // Create RNG for each point sample
    SampleGenerator sg = SampleGenerator(threadID.xy, 5 * 13);

    // Decide which triangle sampling type we should use
    uint type = findInterval(gSampleTypeCdfBuffer, sampleNext1D(sg));
    uint globalTriangleIdx;
    if (type == uint(TriangleSampleType::TriangleThreeVertices))
    {
        globalTriangleIdx = uint(sampleNext1D(sg) * gTotalNonEmissiveTriCount);
    }
    else if (type == uint(TriangleSampleType::AllTriangles))
    {
        // Sample triangle PDF to get a triangle index
        uint2 texSize;
        gTriAreaPDFTexture.GetDimensions(texSize.x, texSize.y);
        uint2 texelPos;
        float pdf;
        samplePdfMipmap(sg, gTriAreaPDFTexture, texSize, texelPos, pdf);
        globalTriangleIdx = RTXDI_ZCurveToLinearIndex(texelPos);
    }
    else
    {
        uint2 texSize;
        gTriAreaPDFTexOnlyForSmallTri.GetDimensions(texSize.x, texSize.y);
        uint2 texelPos;
        float pdf;
        samplePdfMipmap(sg, gTriAreaPDFTexOnlyForSmallTri, texSize, texelPos, pdf);
        globalTriangleIdx = RTXDI_ZCurveToLinearIndex(texelPos);
    }

    // Get instanceID using binary search
    uint id = findInterval(gNonEmisInstTriOffsetBuffer, globalTriangleIdx);
    GeometryInstanceID instanceID = { gNonEmisInstTriOffsetBuffer[id].y };
    pointSample.instanceID = instanceID.index;

    // Get the three vertices of current triangle (without world transform)
    uint triangleIndex = globalTriangleIdx - gNonEmisInstTriOffsetBuffer[id].x;
    float3 triVertices[3];
    uint3 vtxIndices = gScene.getIndices(instanceID, triangleIndex);
    for (uint i = 0; i < 3; i++)
        triVertices[i] = gScene.vertices[vtxIndices[i]].position;

    // Random position on triangle
    if (type != uint(TriangleSampleType::TriangleThreeVertices))
    {
        float3 barycentrics = sample_triangle(sampleNext2D(sg));
        pointSample.pos = barycentrics.x * triVertices[0] + barycentrics.y * triVertices[1] + barycentrics.z * triVertices[2];
    }
    else
    {
        pointSample.pos = triVertices[uint(3 * sampleNext1D(sg))];
    }

    // Find which light section the thread is in
    uint lightID = findInterval(gLightNumPointsOffsetBuffer, pointsBufferIdx);
    pointSample.selectedLightID = lightID;
    gPointsBuffer[pointsBufferIdx] = packIsmPointSample(pointSample);

    gDebugLightID[pointsBufferIdx] = lightID;
    InterlockedAdd(gTypeCountBuffer[type], 1);
}
