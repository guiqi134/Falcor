import Utils.Debug.PixelDebug;

Texture2D<float4> gInputEmission; // Emissive color from the G-buffer
RWTexture2D<float4> gOutputEmission;

static const float gaussianKernel5x5[25] = {
    1, 4, 7, 4, 1,
    4, 16, 26, 16, 4,
    7, 26, 41, 26, 7,
    4, 16, 26, 16, 4,
    1, 4, 7, 4, 1
}; // divide by 273

static const float gaussianKernel9x9_2std[81] = {
    0.00072876, 0.00174820, 0.00326606, 0.00475209, 0.00538482, 0.00475209, 0.00326606, 0.00174820, 0.00072876,
    0.00174820, 0.00419370, 0.00783487, 0.01139966, 0.01291751, 0.01139966, 0.00783487, 0.00419370, 0.00174820,
    0.00326606, 0.00783487, 0.01463746, 0.02129738, 0.02413309, 0.02129738, 0.01463746, 0.00783487, 0.00326606,
    0.00475209, 0.01139966, 0.02129738, 0.03098750, 0.03511344, 0.03098750, 0.02129738, 0.01139966, 0.00475209,
    0.00538482, 0.01291751, 0.02413309, 0.03511344, 0.03978874, 0.03511344, 0.02413309, 0.01291751, 0.00538482,
    0.00475209, 0.01139966, 0.02129738, 0.03098750, 0.03511344, 0.03098750, 0.02129738, 0.01139966, 0.00475209,
    0.00326606, 0.00783487, 0.01463746, 0.02129738, 0.02413309, 0.02129738, 0.01463746, 0.00783487, 0.00326606,
    0.00174820, 0.00419370, 0.00783487, 0.01139966, 0.01291751, 0.01139966, 0.00783487, 0.00419370, 0.00174820,
    0.00072876, 0.00174820, 0.00326606, 0.00475209, 0.00538482, 0.00475209, 0.00326606, 0.00174820, 0.00072876
};

static const int kernelRadius = 4;

[numthreads(16, 16, 1)]
void main(uint3 threadId: SV_DispatchThreadID)
{
    printSetPixel(threadId.xy);

    print("Bloom Effect -----------------------", 1);

    // Find our pixel location (on screen and in our reservoir list)
    const uint2 launchIdx = threadId.xy;
    float3 outputColor = 0.0f;
    uint kernelSize = kernelRadius * 2 + 1;
    for (int xOffset = -kernelRadius; xOffset <= kernelRadius; xOffset++)
    {
        for (int yOffset = -kernelRadius; yOffset <= kernelRadius; yOffset++)
        {
            int2 offsetPixel = launchIdx + int2(xOffset, yOffset);
            uint weightIdx = kernelSize * (yOffset + kernelRadius) + xOffset + kernelRadius;
            // float weight = gaussianKernel5x5[weightIdx] / 273.0f;
            float weight = gaussianKernel9x9_2std[weightIdx];
            outputColor += all(offsetPixel >= 0) ? gInputEmission[offsetPixel].rgb * weight : 0.0f;

            print("offsetPixel = ", offsetPixel);
            print("weightIdx = ", weightIdx);
            print("weight = ", weight);
        }
    }

    print("input emission = ", gInputEmission[launchIdx]);
    print("outputColor = ", outputColor);

    gOutputEmission[launchIdx].rgb = outputColor;
}

Texture2D<float4> gInputBloom;
RWTexture2D<float4> gColor;

[numthreads(16, 16, 1)]
void blending(uint3 threadId: SV_DispatchThreadID)
{
    printSetPixel(threadId.xy);
    print("Blending bloom -----------------------", 1);

    // Find our pixel location (on screen and in our reservoir list)
    gColor[threadId.xy] += gInputBloom[threadId.xy];
}
