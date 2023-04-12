import Scene.Scene;
import Utils.Debug.PixelDebug;
import ISMHelpers;

cbuffer CB
{
    uint gLaunchWidth;
    uint gTotalPointSamples;
}

Buffer<uint> gPointToGlobalTriIdxBuffer;
Buffer<uint> gTriPointsCountBuffer;
Buffer<float> gTriMaxDistBuffer;
RWStructuredBuffer<PackedIsmPointSample> gPointsBuffer;

[numthreads(16, 16, 1)]
void main(uint3 threadID: SV_DispatchThreadID)
{
    uint pointsBufferIdx = threadID.y * gLaunchWidth + threadID.x;
    if (pointsBufferIdx >= gTotalPointSamples) return;

    IsmPointSample pointSample = unpackIsmPointSample(gPointsBuffer[pointsBufferIdx]);
    uint globalTriangleIdx = gPointToGlobalTriIdxBuffer[pointsBufferIdx];

    uint numPoints = gTriPointsCountBuffer[globalTriangleIdx];
    // gPointsBuffer[pointsBufferIdx].radius = numPoints != 0 ? 3.0 * gTriMaxDistBuffer[globalTriangleIdx] / float(numPoints) : 0.0f;
}
