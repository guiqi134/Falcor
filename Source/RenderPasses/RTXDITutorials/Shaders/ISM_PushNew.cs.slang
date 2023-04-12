import Utils.Debug.PixelDebug;

cbuffer PushCB
{
    uint gPushMode;
    float2 gLightNearFarPlane;
}

Texture2DArray<float> gIsmInput;
RWTexture2DArray<float> gIsmOutput;
Texture2DArray<float> gRadiusNormalInput;
Texture2DArray<float2> gDispVecInput;

// Redundent variables
RWTexture2DArray<float> gRadiusNormalOuput;
RWTexture2DArray<float2> gDispVecOutput;

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

    // Get texel sizes
    float3 coarserTexSize;
    gIsmInput.GetDimensions(coarserTexSize.x, coarserTexSize.y, coarserTexSize.z);
    float coarserTexelSize = 1.0f / coarserTexSize.x;
    float3 finerTexSize;
    gIsmOutput.GetDimensions(finerTexSize.x, finerTexSize.y, finerTexSize.z);
    float finerTexelSize = 1.0f / finerTexSize.x;

    uint texArrayIndex = threadID.z;
    uint2 currPixelCoarsePos = threadID.xy / 2;
    uint2 currPixelCoarseOffset = currPixelCoarsePos % 2;
    uint2 leftTopCoarsePos = currPixelCoarsePos - currPixelCoarseOffset;
    float currPixelDepth = gIsmOutput[uint3(threadID.xy, texArrayIndex)];
    float computedDepth = 0.0f;

    print("currPixelCoarsePos = ", currPixelCoarsePos);
    print("leftTopCoarsePos = ", leftTopCoarsePos);
    print("currPixelCoarseOffset = ", currPixelCoarseOffset);

    // Brute-force method: Just copy the repective value in the coarser level to the pixel in the finer level
    if (gPushMode == 0)
    {
        computedDepth = gIsmInput[uint3(currPixelCoarsePos, texArrayIndex)];
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
            uint3 coarsePixelPos = uint3(leftTopCoarsePos + offsets[i], texArrayIndex);
            float coarseDepth = gIsmInput[coarsePixelPos];

            if (coarseDepth != 1.0f)
            {
                // Check if the finer pixel is covered in the coarser circle filter
                float2 centerCoarseCoord = coarserTexelSize * coarsePixelPos.xy + gDispVecInput[coarsePixelPos];
                float2 finerPixelCenterCoord = finerTexelSize * (float2(threadID.xy) + float2(0.5f));
                float difference = distance(centerCoarseCoord, finerPixelCenterCoord);

                // How to compare the above difference in texture-space with the radius defined in object-space???

                computedDepth += weights[i] * coarseDepth;
                weightSum += weights[i];
            }
        }

        print("weightSum = ", weightSum);

        if (weightSum == 0.0f)
        {
            computedDepth = 1.0f;
        }
        else
        {
            computedDepth /= weightSum;
        }
    }

    print("computedDepth = ", computedDepth);
    print("currPixelDepth = ", currPixelDepth);

    // Check if the interpolated depth value is valid or occluded
    if (computedDepth != 1.0f && (computedDepth + gRadiusNormalInput[uint3(currPixelCoarsePos, texArrayIndex)]) < currPixelDepth)
    {
        gIsmOutput[uint3(threadID.xy, texArrayIndex)] = computedDepth;
    }

}
