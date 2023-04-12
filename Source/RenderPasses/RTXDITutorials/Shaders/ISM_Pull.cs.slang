import Utils.Debug.PixelDebug;
import ISMHelpers;

cbuffer PullCB
{
    float gDepthDiffThreshold;
    float2 gLightNearFarPlane;
}

Texture2DArray<float> gIsmInput;
RWTexture2DArray<float> gIsmOutput;

static const uint2 offset[4] = {
    uint2(0, 0), uint2(0, 1),
    uint2(1, 0), uint2(1, 1)
};

// Two conditions must meet for pixel's depth value to be used in averaging:
// 1. It needs to contain a vaild depth
// 2. Their depth should close to each other
[numthreads(16, 16, 1)]
void main(uint3 threadID : SV_DispatchThreadID)
{
    // printSetPixel(threadID.xy);

    uint texArrayIndex = threadID.z;
    float pixelDepths[4];
    float frontmostDepth = 1.0f;
    uint validPixelCount = 4;
    float depthSum = 0.0f;

    // float2 texDims;
    // gIsmInput.GetDimensions(texDims.x, texDims.y);
    // gIsmOutput.GetDimensions(texDims.x, texDims.y);
    // print("input texDims = ", texDims);
    // print("output texDims = ", texDims);

    // Get pixels values and the frontmost pixel. Also check if is valid
    for (uint i = 0; i < 4; i++)
    {
        uint2 inputPixelPos = 2 * threadID.xy + offset[i];
        float pixelDepth = gIsmInput[uint3(inputPixelPos, texArrayIndex)];
        frontmostDepth = min(frontmostDepth, pixelDepth);

        if (pixelDepth == 1.0f)
        {
            validPixelCount--;
            pixelDepths[i] = -1.0f;
        }
        else
        {
            pixelDepths[i] = pixelDepth;
        }
    }

    // print("validPixelCount = ", validPixelCount);

    // Do the occlusion check. TODO: later change to use the radius
    for (uint i = 0; i < 4; i++)
    {
        if (pixelDepths[i] == -1.0f) continue;

        float currPixelDepth = pixelDepths[i];
        float currPixelLinearDepth = abs(linearizeDepth(currPixelDepth, gLightNearFarPlane.x, gLightNearFarPlane.y));
        float frontmostLienarDepth = abs(linearizeDepth(frontmostDepth, gLightNearFarPlane.x, gLightNearFarPlane.y));
        print("currPixelLinearDepth = ", currPixelLinearDepth);
        print("frontmostLienarDepth = ", frontmostLienarDepth);
        print("gDepthDiffThreshold = ", gDepthDiffThreshold);

        if (currPixelLinearDepth <= frontmostLienarDepth + gDepthDiffThreshold)
        {
            depthSum += currPixelDepth;
        }
        else
        {
            validPixelCount--;
        }
    }

    // print("validPixelCount = ", validPixelCount);

    float depthAvgOut = 1.0f;
    if (validPixelCount != 0)
    {
        depthAvgOut = depthSum / (float)validPixelCount;
    }
    // print("depthAvgOut = ", depthAvgOut);

    gIsmOutput[uint3(threadID.xy, texArrayIndex)] = depthAvgOut;
}
