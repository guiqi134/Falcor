import Utils.Debug.PixelDebug;
import ISMHelpers;

cbuffer PullCB
{
    float2 gLightNearFarPlane;
}

Buffer<uint2> gSortedLightsBuffer;
StructuredBuffer<LightShadowMapData> gLightShadowDataBuffer;

Texture2DArray<float> gIsmInput;
RWTexture2DArray<float> gIsmOutput;
Texture2DArray<float> gRadiusNormalInput;
RWTexture2DArray<float> gRadiusNormalOuput;
Texture2DArray<float2> gDispVecInput; // This is for computing the correct displaced center from texel center
RWTexture2DArray<float2> gDispVecOutput;

// LT -> LB -> RB -> RT
static const uint2 offset[4] = {
    uint2(0, 0), uint2(0, 1),
    uint2(1, 0), uint2(1, 1)
};

static const float2 coarserCenterToFinerOffset[4] = {
    float2(-0.5f, -0.5f), float2(-0.5f, 0.5f),
    float2(0.5f, -0.5f), float2(0.5f, 0.5f)
};

struct IsmPixelData
{
    float depth;
    bool isValid;
    float radius;
    float2 dispVec;
};

// Two conditions must meet for pixel's depth value to be used in averaging:
// 1. It needs to contain a vaild depth
// 2. Their depth range should intersect with the frontmost depth range
[numthreads(16, 16, 1)]
void main(uint3 threadID : SV_DispatchThreadID)
{
    // printSetPixel(threadID.xy);

    // Get texel sizes
    float3 coarserTexSize;
    gIsmOutput.GetDimensions(coarserTexSize.x, coarserTexSize.y, coarserTexSize.z);
    float coarserTexelSize = 1.0f / coarserTexSize.x;
    float3 finerTexSize;
    gIsmInput.GetDimensions(finerTexSize.x, finerTexSize.y, finerTexSize.z);
    float finerTexelSize = 1.0f / finerTexSize.x;

    uint texArrayIndex = threadID.z;
    IsmPixelData finerPixels[4];
    float2 frontmostLinearDepth = gLightNearFarPlane.y; // range from [depth, depth + 2r]
    uint validPixelCount = 4;

    // Get pixels values and the frontmost pixel. Also check if is valid
    for (uint i = 0; i < 4; i++)
    {
        // Get the ipos of input pixel and read its data
        uint3 inputPixelPos = uint3(2 * threadID.xy + offset[i], texArrayIndex);
        finerPixels[i].depth = gIsmInput[inputPixelPos];
        if (finerPixels[i].depth != 1.0f)
        {
            finerPixels[i].isValid = true;
            finerPixels[i].radius = gRadiusNormalInput[inputPixelPos];
            finerPixels[i].dispVec = gDispVecInput[inputPixelPos].xy;

            // Compute the frontmost depth range
            float finerPixelLinearDepth = linearizeDepth(finerPixels[i].depth, gLightNearFarPlane.x, gLightNearFarPlane.y);
            if (finerPixelLinearDepth < frontmostLinearDepth.x)
            {
                frontmostLinearDepth.x = finerPixelLinearDepth;
                frontmostLinearDepth.y = finerPixelLinearDepth + 2.0f * finerPixels[i].radius;
            }
        }
        else
        {
            finerPixels[i].isValid = false;
            validPixelCount--;
        }
    }

    print("validPixelCount = ", validPixelCount);

    // Aggregate the finer pixels to coarser one
    float depthSum = 0.0f;
    float radiusSum = 0.0f;
    float3 projNormalSum = 0.0f;
    float2 dispVecSum = 0.0f;
    for (uint i = 0; i < 4; i++)
    {
        if (finerPixels[i].isValid)
        {
            // Occlusion check
            if (linearizeDepth(finerPixels[i].depth, gLightNearFarPlane.x, gLightNearFarPlane.y) <= frontmostLinearDepth.y)
            {
                depthSum += finerPixels[i].depth;
                radiusSum += finerPixels[i].radius;
                dispVecSum += finerTexelSize * coarserCenterToFinerOffset[i] + finerPixels[i].dispVec;
            }
            else
            {
                validPixelCount--;
            }
        }
    }

    print("validPixelCount = ", validPixelCount);

    if (validPixelCount != 0)
    {
        gIsmOutput[uint3(threadID.xy, texArrayIndex)] = depthSum / float(validPixelCount);
        gRadiusNormalOuput[uint3(threadID.xy, texArrayIndex)].x = radiusSum / float(validPixelCount);
        gDispVecOutput[uint3(threadID.xy, texArrayIndex)] = dispVecSum / float(validPixelCount);
    }
}
