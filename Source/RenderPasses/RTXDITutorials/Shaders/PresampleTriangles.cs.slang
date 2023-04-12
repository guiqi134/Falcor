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
    uint gNumTriAreaPDFTextures;
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

// Below is for presampling triangle
#ifndef NUM_AREA_TEXTURES
#define NUM_AREA_TEXTURES 3
#endif

#define RTXDI_TEX2D_LOAD(tex, pos, lod) tex.Load(int3(pos, lod))

Texture2D<float2> gTriAreaPDFTexture[NUM_AREA_TEXTURES];
Buffer<uint2> gNonEmisInstTriOffsetBuffer;
Buffer<uint> gLightNumPointsOffsetBuffer;
Buffer<float> gTexCdfBuffer;
Buffer<float> gTexCdfByAreaBuffer;
RWStructuredBuffer<PackedIsmPointSample> gPointsBuffer;
RWBuffer<uint> gPointToGlobalTriIdxBuffer;
RWBuffer<uint> gTriPointsCountBuffer;
RWBuffer<float> gTriMaxDistBuffer;

RWBuffer<uint> gTypeCountBuffer;

uint samplePdfMipmap(
    inout TinyUniformSampleGenerator rng,
    Texture2D<float2> pdfTexture, // full mip chain starting from unnormalized sampling pdf in mip 0
    uint2 pdfTextureSize)        // dimensions of pdfTexture at mip 0; must be 16k or less
{
    int lastMipLevel = max(0, int(floor(log2(max(pdfTextureSize.x, pdfTextureSize.y)))) - 1);

    uint2 position = uint2(0, 0);
    float pdf = 1.0;
    for (int mipLevel = lastMipLevel; mipLevel >= 0; mipLevel--)
    {
        position *= 2;

        float4 samples;
        samples.x = max(0, RTXDI_TEX2D_LOAD(pdfTexture, int2(position.x + 0, position.y + 0), mipLevel).x);
        samples.y = max(0, RTXDI_TEX2D_LOAD(pdfTexture, int2(position.x + 0, position.y + 1), mipLevel).x);
        samples.z = max(0, RTXDI_TEX2D_LOAD(pdfTexture, int2(position.x + 1, position.y + 0), mipLevel).x);
        samples.w = max(0, RTXDI_TEX2D_LOAD(pdfTexture, int2(position.x + 1, position.y + 1), mipLevel).x);

        float weightSum = samples.x + samples.y + samples.z + samples.w;
        if (weightSum <= 0)
        {
            pdf = 0;
            break;
        }

        samples /= weightSum;

        float rnd = sampleNext1D(rng);

        int2 selectedOffset;

        if (rnd < samples.x)
        {
            pdf *= samples.x;
        }
        else
        {
            rnd -= samples.x;

            if (rnd < samples.y)
            {
                position += uint2(0, 1);
                pdf *= samples.y;
            }
            else
            {
                rnd -= samples.y;

                if (rnd < samples.z)
                {
                    position += uint2(1, 0);
                    pdf *= samples.z;
                }
                else
                {
                    position += uint2(1, 1);
                    pdf *= samples.w;
                }
            }
        }
    }

    uint globalTriangleIdx = uint(RTXDI_TEX2D_LOAD(pdfTexture, int2(position.x, position.y), 0).y);
    return globalTriangleIdx;
}

[numthreads(16, 16, 1)]
void presampleTriangle(uint3 threadID : SV_DispatchThreadID)
{
    printSetPixel(threadID.xy);

    uint pointsBufferIdx = threadID.y * gLaunchWidth + threadID.x;
    if (pointsBufferIdx >= gTotalPointSamples) return;

    IsmPointSample pointSample;

    // Create RNG for each point sample
    SampleGenerator sg = SampleGenerator(threadID.xy, 5 * 13);

    // Decide which pdf texture to use and sample triangle PDF to get a triangle index
    // uint whichPdfTex = findInterval(gTexCdfBuffer, sampleNext1D(sg));
    uint whichPdfTex = uint(gNumTriAreaPDFTextures * sampleNext1D(sg));
    uint2 texSize;
    gTriAreaPDFTexture[whichPdfTex].GetDimensions(texSize.x, texSize.y);
    uint globalTriangleIdx = samplePdfMipmap(sg, gTriAreaPDFTexture[whichPdfTex], texSize);

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
    float3 barycentrics = sample_triangle(sampleNext2D(sg));
    pointSample.pos = barycentrics.x * triVertices[0] + barycentrics.y * triVertices[1] + barycentrics.z * triVertices[2];

    // Store info for computing point radius later
    // gPointToGlobalTriIdxBuffer[pointsBufferIdx] = globalTriangleIdx;
    // InterlockedAdd(gTriPointsCountBuffer[globalTriangleIdx], 1);
    // float3 triCenter = (triVertices[0] + triVertices[1] + triVertices[2]) / 3.0f;
    // float maxDist = max(max(length(triVertices[0] - triCenter), length(triVertices[1] - triCenter)), length(triVertices[2] - triCenter));
    // gTriMaxDistBuffer[globalTriangleIdx] = maxDist;

    // Find which light section the thread is in
    uint lightID = findInterval(gLightNumPointsOffsetBuffer, pointsBufferIdx);
    pointSample.selectedLightID = lightID;

    gPointsBuffer[pointsBufferIdx] = packIsmPointSample(pointSample);

    InterlockedAdd(gTypeCountBuffer[whichPdfTex], 1);
}
