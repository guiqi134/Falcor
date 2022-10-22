import Utils.Debug.PixelDebug;

Texture2DArray<float> gIsmInput;
RWTexture2D<float4> gSingleIsmOutput;

cbuffer VisCB
{
    float gLightNear;
    float gLightFar;
    float gMipLevel;
    float gArrayIndex;
    uint2 gThreadDim;
    SamplerState gPointSampler;
    SamplerState gLinearSampler;
}

float paraboloidProjToView(float depth, float near, float far)
{
    return depth * (far - near) + near;
}

// Visualize signle ISM texture
[numthreads(16, 16, 1)]
void main(uint3 threadID : SV_DispatchThreadID)
{
    printSetPixel(threadID.xy);

    float3 location = float3(float2(threadID.xy) / float2(gThreadDim), gArrayIndex);
    float sampleDepth = gIsmInput.SampleLevel(gPointSampler, location, gMipLevel);
    float depthView = paraboloidProjToView(sampleDepth, gLightNear, gLightFar);
    gSingleIsmOutput[threadID.xy] = float4(float3(abs(depthView) / 5.0f), 1.0f);

    print("location = ", location);
    print("sampleDepth = ", sampleDepth);
    print("depthView = ", depthView);
}

// Visualize the entire ISM texture array