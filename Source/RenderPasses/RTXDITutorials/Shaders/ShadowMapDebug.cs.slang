import Utils.Debug.PixelDebug;

cbuffer debugCB
{
    float gLightNear;
    float gLightFar;
    uint gShadowMapSize;
    SamplerState gPointSampler;
}

Texture2DArray<float> gShadowMaps;
RWTexture2D gDebugTexture;

float linearizeDepth(float depthNDC, float near, float far)
{
    return (near * far) / (depthNDC * (far - near) - far);
}

[numthreads(16, 16, 1)]
void main(uint3 threadId: SV_DispatchThreadID)
{
    printSetPixel(threadId.xy);

    float depth = gShadowMaps[uint3(threadId.xy, 5)];
    print("depth = ", depth);
    float depthCS = linearizeDepth(depth, gLightNear, gLightFar);
    float4 debugColor = float4(float3(abs(depthCS) / 5.0f), 1.0f);

    gDebugTexture[threadId.xy] = debugColor;
}