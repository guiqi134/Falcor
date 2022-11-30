import Utils.Debug.PixelDebug;

Texture2DArray<float> gIsmInput;
RWTexture2D<float4> gIsmOutput;

cbuffer VisCB
{
    float gLightNear;
    float gLightFar;
    float gMipLevel;
    float gArrayIndex;
    uint2 gInputSize;
    uint2 gOutputSize;
    SamplerState gPointSampler;
    SamplerState gLinearSampler;
}

float paraboloidProjToView(float depth, float near, float far)
{
    return depth * (far - near) + near;
}

float prespectiveProjToView(float depthNDC, float near, float far)
{
    return (near * far) / (depthNDC * (far - near) - far);
}

// Visualize single ISM texture
[numthreads(16, 16, 1)]
void singleIsmCS(uint3 threadID : SV_DispatchThreadID)
{
    printSetPixel(threadID.xy);

    float3 location = float3(float2(threadID.xy) / float2(gInputSize), gArrayIndex);
    float sampleDepth = gIsmInput.SampleLevel(gPointSampler, location, gMipLevel);
    float depthView = paraboloidProjToView(sampleDepth, gLightNear, gLightFar);
    gIsmOutput[threadID.xy] = float4(float3(abs(depthView) / 5.0f), 1.0f);

    print("location = ", location);
    print("sampleDepth = ", sampleDepth);
    print("depthView = ", depthView);
}

// Visualize the entire ISM texture array
[numthreads(16, 16, 1)]
void allIsmCS(uint3 threadID: SV_DispatchThreadID)
{
    uint texArrayIndex = threadID.z;
    uint2 posInIsm = threadID.xy;

    // Get the index for current ISM texture on the output texture
    const uint rowSize = gOutputSize.x / gInputSize.x;
    uint2 ismOffset = uint2(texArrayIndex % rowSize, texArrayIndex / rowSize);

    // Map the position in ISM to output texture
    uint2 posInOutputTex = posInIsm + ismOffset * gInputSize;

    // Get the depth value from input ISM texture
    float3 location = float3(posInIsm / float2(gInputSize), (float)texArrayIndex);
    float sampleDepth = gIsmInput.SampleLevel(gPointSampler, location, 0);
    float depthView = paraboloidProjToView(sampleDepth, gLightNear, gLightFar);
    gIsmOutput[posInOutputTex] = float4(float3(abs(depthView) / 5.0f), 1.0f);

}
