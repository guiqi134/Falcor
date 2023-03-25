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
RWBuffer<float> gTriAreas;

// void interlockedAddFloat(float value)
// {
//     uint i_val = asuint(value);
//     uint tmp0 = 0;
//     uint tmp1;

//     [allow_uav_condition]
//     while (true)
//     {
//         InterlockedCompareExchange(gTriAreaSum[0], tmp0, i_val, tmp1);
//         if (tmp1 == tmp0)
//             break;
//         tmp0 = tmp1;
//         i_val = asuint(value + asfloat(tmp1));
//     }
// }

[numthreads(16, 16, 1)]
void triAreaSum(uint3 threadID: SV_DispatchThreadID)
{
    uint globalThreadID = threadID.y * gLaunchWidth + threadID.x;
    if (globalThreadID >= gTotalNonEmisTriCount) return;
    uint id = findInterval(gNonEmisInstTriOffsetBuffer, globalThreadID);
    GeometryInstanceID instanceID = { gNonEmisInstTriOffsetBuffer[id].y };

    // Accumulate the area of current triangle
    uint triangleIndex = globalThreadID - gNonEmisInstTriOffsetBuffer[id].x;
    float triArea;
    gScene.getFaceNormalAndAreaW(instanceID, triangleIndex, triArea);
    gTriAreas[globalThreadID] = triArea;
    // interlockedAddFloat(triArea);
}

#include "rtxdi/RtxdiMath.hlsli" // Node: this file is in the original location, not my copied one
import Utils.Sampling.TinyUniformSampleGenerator;
typedef TinyUniformSampleGenerator SampleGenerator;

cbuffer pdfCB
{
    float gExactTriAreaSum;
}

RWTexture2D<float> gTriAreaPDFTexture;

[numthreads(16, 16, 1)]
void pdfTexForAllTris(uint3 threadID : SV_DispatchThreadID)
{
    printSetPixel(threadID.xy);

    uint globalThreadID = threadID.y * gLaunchWidth + threadID.x;

    // Discard the thread if we go out of range
    if (globalThreadID >= gTotalNonEmisTriCount) return;

    uint2 pdfTexPos = RTXDI_LinearIndexToZCurve(globalThreadID);

    // Get the information of current non-emissmive triangle
    float triArea = gTriAreas[globalThreadID];
    gTriAreaPDFTexture[pdfTexPos] = triArea / gExactTriAreaSum;
}
