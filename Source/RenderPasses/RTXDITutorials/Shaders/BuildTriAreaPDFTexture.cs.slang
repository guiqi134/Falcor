import Scene.Scene;
import Utils.Debug.PixelDebug;
import ISMHelpers;

cbuffer CB
{
    uint gTotalNonEmisTriCount;
    uint gLaunchWidth;
}

Buffer<uint2> gNonEmisInstTriOffsetBuffer;
RWBuffer<uint> gTriAreaSum;
RWBuffer<float> gAllTriAreas;

[numthreads(16, 16, 1)]
void triAreaSum(uint3 threadID: SV_DispatchThreadID)
{
    uint globalThreadID = threadID.y * gLaunchWidth + threadID.x;
    if (globalThreadID >= gTotalNonEmisTriCount) return;
    uint id = findInterval(gNonEmisInstTriOffsetBuffer, globalThreadID);
    GeometryInstanceID instanceID = { gNonEmisInstTriOffsetBuffer[id].y };

    // Get the area of current triangle
    uint triangleIndex = globalThreadID - gNonEmisInstTriOffsetBuffer[id].x;
    float triArea;
    gScene.getFaceNormalAndAreaW(instanceID, triangleIndex, triArea);
    gAllTriAreas[globalThreadID] = triArea;
}

#include "rtxdi/RtxdiMath.hlsli" // Node: this file is in the original location, not my copied one
import Utils.Sampling.TinyUniformSampleGenerator;
typedef TinyUniformSampleGenerator SampleGenerator;

Buffer<float2> gSubTriAreas;
RWTexture2D<float2> gTriAreaPDFTexture;

cbuffer pdfCB
{
    uint gNumTriangles;
}

[numthreads(16, 16, 1)]
void pdfTexForAllTris(uint3 threadID : SV_DispatchThreadID)
{
    printSetPixel(threadID.xy);

    uint globalThreadID = threadID.y * gLaunchWidth + threadID.x;
    if (globalThreadID >= gNumTriangles) return;

    uint2 pdfTexPos = RTXDI_LinearIndexToZCurve(globalThreadID);

    // Get the information of current non-emissmive triangle
    gTriAreaPDFTexture[pdfTexPos] = gSubTriAreas[globalThreadID];
}
