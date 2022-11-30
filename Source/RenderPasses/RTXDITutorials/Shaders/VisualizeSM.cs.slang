#include "../HostDeviceSharedDefinitions.h"

import Utils.Debug.PixelDebug;

Texture2DArray<float> gIsmInput;
StructuredBuffer<LightShadowMapData> gLightShadowDataBuffer;
Buffer<uint> gIsmLightIndexBuffer;
RWTexture2D<float4> gIsmOutput;

cbuffer VisCB
{
    float gMipLevel;
    uint gArrayIndex;
    uint2 gInputSize;
    uint2 gOutputSize;
    SamplerState gPointSampler;
    SamplerState gLinearSampler;
}

float paraboloidProjToView(float depth, float near, float far)
{
    return depth * (far - near) + near;
}

// Visualize single ISM texture
[numthreads(16, 16, 1)]
void singleIsmCS(uint3 threadID : SV_DispatchThreadID)
{
    printSetPixel(threadID.xy);

    uint lightID = gIsmLightIndexBuffer[gArrayIndex / 2];
    LightShadowMapData lightShadowData = gLightShadowDataBuffer[lightID];

    float3 location = float3(float2(threadID.xy) / float2(gInputSize), (float)gArrayIndex);
    float sampleDepth = gIsmInput.SampleLevel(gPointSampler, location, gMipLevel);
    float depthView = paraboloidProjToView(sampleDepth, lightShadowData.nearFarPlane.x, lightShadowData.nearFarPlane.y);
    gIsmOutput[threadID.xy] = float4(float3(abs(depthView) / lightShadowData.nearFarPlane.y), 1.0f);

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

    uint lightID = gIsmLightIndexBuffer[texArrayIndex / 2];
    LightShadowMapData lightShadowData = gLightShadowDataBuffer[lightID];

    // Get the index for current ISM texture on the output texture
    const uint rowSize = gOutputSize.x / gInputSize.x;
    uint2 ismOffset = uint2(texArrayIndex % rowSize, texArrayIndex / rowSize);

    // Map the position in ISM to output texture
    uint2 posInOutputTex = posInIsm + ismOffset * gInputSize;

    // Get the depth value from input ISM texture
    float3 location = float3(posInIsm / float2(gInputSize), (float)texArrayIndex);
    float sampleDepth = gIsmInput.SampleLevel(gPointSampler, location, 0);
    float depthView = paraboloidProjToView(sampleDepth, lightShadowData.nearFarPlane.x, lightShadowData.nearFarPlane.y);
    gIsmOutput[posInOutputTex] = float4(float3(abs(depthView) / lightShadowData.nearFarPlane.y), 1.0f);

    if (lightShadowData.shadowMapType != 2)
    {
        // gIsmOutput[posInOutputTex] = float4(abs(depthView) / lightShadowData.nearFarPlane.y, 0.0f, 0.0f, 1.0f);
        gIsmOutput[posInOutputTex] = float4(1.0f, 0.0f, 0.0f, 1.0f);
    }
}

// Or visualize all ISMs based on light ID
[numthreads(16, 16, 1)]
void allIsmCS2(uint3 threadID: SV_DispatchThreadID)
{
    uint lightIsmIndex = threadID.z;
    uint2 posInIsm = threadID.xy;

    uint lightID = lightIsmIndex / 2;
    LightShadowMapData lightShadowData = gLightShadowDataBuffer[lightID];

    // Get the index for current ISM texture on the output texture
    const uint rowSize = gOutputSize.x / gInputSize.x;
    uint2 ismOffset = uint2(lightIsmIndex % rowSize, lightIsmIndex / rowSize);

    // Map the position in ISM to output texture
    uint2 posInOutputTex = posInIsm + ismOffset * gInputSize;

    // Get the depth value from input ISM texture
    if (lightShadowData.shadowMapType == 2)
    {
        float3 location = float3(posInIsm / float2(gInputSize), lightShadowData.startIndex + lightIsmIndex % 2);
        float sampleDepth = gIsmInput.SampleLevel(gPointSampler, location, 0);
        float depthView = paraboloidProjToView(sampleDepth, lightShadowData.nearFarPlane.x, lightShadowData.nearFarPlane.y);
        gIsmOutput[posInOutputTex] = float4(float3(abs(depthView) / lightShadowData.nearFarPlane.y), 1.0f);

    }
    else
    {
        // gIsmOutput[posInOutputTex] = float4(abs(depthView) / lightShadowData.nearFarPlane.y, 0.0f, 0.0f, 1.0f);
        gIsmOutput[posInOutputTex] = float4(1.0f, 0.0f, 0.0f, 1.0f);
    }
}
