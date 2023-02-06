import Utils.Debug.PixelDebug;

Texture2DArray<float> gInput;
RWTexture2DArray<float> gOutput;

cbuffer PushCB
{
    uint gPushMode;
    float gDepthDiffThreshold;
}

static const uint2 offsets[4] = {
    uint2(0, 0), uint2(0, 1),
    uint2(1, 0), uint2(1, 1)
};

// To replace the value in finer level using coarse level value, it needs to meet the condition:
// 1. Valid depth
// 2. coarse level depth < finer level depth
[numthreads(16, 16, 1)]
void main(uint3 threadID : SV_DispatchThreadID)
{
    // printSetPixel(threadID.xy);
    print("gDepthDiffThreshold = ", gDepthDiffThreshold);

    uint texArrayIndex = threadID.z;
    uint2 currPixelCoarsePos = threadID.xy / 2;
    uint2 currPixelCoarseOffset = currPixelCoarsePos % 2;
    uint2 leftTopCoarsePos = currPixelCoarsePos - currPixelCoarseOffset;
    float currPixelDepth = gOutput[uint3(threadID.xy, texArrayIndex)];
    float computedDepth = 0.0f;

    print("currPixelCoarsePos = ", currPixelCoarsePos);
    print("leftTopCoarsePos = ", leftTopCoarsePos);
    print("currPixelCoarseOffset = ", currPixelCoarseOffset);

    // Brute-force method: Just copy the repective value in the coarser level to the pixel in the finer level
    if (gPushMode == 0)
    {
        computedDepth = gInput[uint3(currPixelCoarsePos, texArrayIndex)];
    }

    // Interpolation method: Get the left-most top-most pixel on the coarser level, and then get all the 4 pixels based on it. Do the interpolation using
    // these four coarser pixels.
    // Each coarser level 2*2 region covers 4*4 region in the finer level
    if (gPushMode == 1)
    {
        // Adjust weights according to current pixel position in coarser level
        float4 weights = float4(9.0f, 3.0f, 3.0f, 1.0f);
        if (all(currPixelCoarseOffset == uint2(0)))
        {
            weights = weights.xyzw;
        }
        else if (all(currPixelCoarseOffset == uint2(1, 0)))
        {
            weights = weights.yxwz;
        }
        else if (all(currPixelCoarseOffset == uint2(0, 1)))
        {
            weights = weights.ywxz;
        }
        else
        {
            weights = weights.zywx;
        }

        float weightSum = 0.0f;
        for (uint i = 0; i < 4; i++)
        {
            float coarseDepth = gInput[uint3(leftTopCoarsePos + offsets[i], texArrayIndex)];
            if (coarseDepth == 1.0f) continue;
            computedDepth += weights[i] * gInput[uint3(leftTopCoarsePos + offsets[i], texArrayIndex)];
            weightSum += weights[i];
        }

        print("weightSum = ", weightSum);

        if (weightSum == 0.0f)
            computedDepth = 1.0f;
        else
            computedDepth /= weightSum;
    }

    print("computedDepth = ", computedDepth);
    print("currPixelDepth = ", currPixelDepth);

    // Check if the interpolated depth value is valid or occluded
    if (computedDepth != 1.0f && computedDepth < currPixelDepth /*&& (currPixelDepth - computedDepth) <= gDepthDiffThreshold*/)
    {
        gOutput[uint3(threadID.xy, texArrayIndex)] = computedDepth;
    }

}
