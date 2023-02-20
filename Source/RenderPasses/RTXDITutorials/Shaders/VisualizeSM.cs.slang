#include "../HostDeviceSharedDefinitions.h"

import Utils.Debug.PixelDebug;

#ifndef NUM_INPUT_TEXTURE_ARRAYS
#define NUM_INPUT_TEXTURE_ARRAYS 3
#endif

Texture2DArray<float> gInputSortedSMs[NUM_INPUT_TEXTURE_ARRAYS];
Texture2DArray<float4> gInputSortedMotionTex[NUM_INPUT_TEXTURE_ARRAYS];

StructuredBuffer<LightShadowMapData> gLightShadowDataBuffer;
Buffer<uint> gIsmLightIndexBuffer;
RWTexture2D<float4> gOutput;

cbuffer VisCB
{
    float gMipLevel;
    uint2 gLightFaceIndex;
    uint2 gInputSize;
    uint2 gOutputSize;
    uint gIsmPerLight;
    bool gIsmLinearProj;
    uint gShadowMapsPerLight;
    uint gSortingRules;
    SamplerState gPointSampler;
    SamplerState gLinearSampler;
}

float projToView(float depth, float near, float far)
{
    return gIsmLinearProj ? (near * far) / (depth * (far - near) - far) : depth * (far - near) + near;
}

float persProjToView(float depthNDC, float near, float far)
{
    return (near * far) / (depthNDC * (far - near) - far);
}

// Visualize single PSM
[numthreads(16, 16, 1)]
void singlePsmCS(uint3 threadID : SV_DispatchThreadID)
{
    // printSetPixel(threadID.xy);

    LightShadowMapData lightShadowData = gLightShadowDataBuffer[gLightFaceIndex.x];
    LightFaceData lightFaceData = lightShadowData.lightFaceData[gLightFaceIndex.y];

    // Visualize PSM
    if (lightFaceData.psmTexArrayIdx == -1)
    {
        gOutput[threadID.xy] = float4(1.0f);
    }
    else
    {
        float3 location = float3(float2(threadID.xy) / float2(gInputSize), lightFaceData.psmTexArrayIdx);
        float sampleDepth = gInputSortedSMs[lightFaceData.whichPsmTexArray].SampleLevel(gPointSampler, location, gMipLevel);
        float depthView = persProjToView(sampleDepth, lightShadowData.nearFarPlane.x, lightShadowData.nearFarPlane.y);
        gOutput[threadID.xy] = float4(float3(abs(depthView) / lightShadowData.nearFarPlane.y), 1.0f);
    }
}

// Visualize single PSM
void singleIsmCS(uint3 threadID: SV_DispatchThreadID)
{
    LightShadowMapData lightShadowData = gLightShadowDataBuffer[gLightFaceIndex.x];
    LightFaceData lightFaceData = lightShadowData.lightFaceData[gLightFaceIndex.y];

    // Visualize ISM
    float3 location = float3(float2(threadID.xy) / float2(gInputSize), lightFaceData.ismTexArrayIdx);
    float sampleDepth = gInputSortedSMs[lightFaceData.whichIsmTexArray].SampleLevel(gPointSampler, location, gMipLevel);
    float depthView = projToView(sampleDepth, lightShadowData.nearFarPlane.x, lightShadowData.nearFarPlane.y);
    gOutput[threadID.xy] = float4(float3(abs(depthView) / lightShadowData.nearFarPlane.y), 1.0f);
}

// Visualize PSM's motion texture
void singleMotionTexCS(uint3 threadID: SV_DispatchThreadID)
{
    LightShadowMapData lightShadowData = gLightShadowDataBuffer[gLightFaceIndex.x];
    LightFaceData lightFaceData = lightShadowData.lightFaceData[gLightFaceIndex.y];

    float3 location = float3(float2(threadID.xy) / float2(gInputSize), lightFaceData.psmTexArrayIdx);
    float4 mvec = gInputSortedMotionTex[lightFaceData.whichPsmTexArray].SampleLevel(gPointSampler, location, gMipLevel);
    gOutput[threadID.xy] = abs(mvec);
}

// Visualize the entire ISM texture array
[numthreads(16, 16, 1)]
void allIsmCS(uint3 threadID: SV_DispatchThreadID)
{
    uint ismTexGlobalIdx = threadID.z;
    uint2 posInIsm = threadID.xy;

    uint lightOrGlobalFaceIdx = gIsmLightIndexBuffer[gSortingRules == (uint)SortingRules::LightFaces ? ismTexGlobalIdx : ismTexGlobalIdx / gIsmPerLight];
    uint localLightIdx = gSortingRules == (uint)SortingRules::LightFaces ? lightOrGlobalFaceIdx / gIsmPerLight : lightOrGlobalFaceIdx;
    uint faceIdx = gSortingRules == (uint)SortingRules::LightFaces ? lightOrGlobalFaceIdx % gIsmPerLight : ismTexGlobalIdx % gIsmPerLight;
    LightShadowMapData lightShadowData = gLightShadowDataBuffer[localLightIdx];
    LightFaceData lightFaceData = lightShadowData.lightFaceData[faceIdx];
    uint lightShadowMapType = lightShadowData.lightFaceData[gSortingRules == (uint)SortingRules::LightFaces ? faceIdx : 0].shadowMapType;

    // Get the index for current ISM texture on the output texture
    const uint rowSize = gOutputSize.x / gInputSize.x;
    uint2 ismOffset = uint2(ismTexGlobalIdx % rowSize, ismTexGlobalIdx / rowSize);

    // Map the position in ISM to output texture
    uint2 posInOutputTex = posInIsm + ismOffset * gInputSize;

    // Get the depth value from input ISM texture
    float3 location = float3(posInIsm / float2(gInputSize), ismTexGlobalIdx);
    float sampleDepth = gInputSortedSMs[lightFaceData.whichIsmTexArray].SampleLevel(gPointSampler, location, 0);
    float depthView = projToView(sampleDepth, lightShadowData.nearFarPlane.x, lightShadowData.nearFarPlane.y);
    gOutput[posInOutputTex] = float4(float3(abs(depthView) / lightShadowData.nearFarPlane.y), 1.0f);

    if (lightShadowMapType != 2)
    {
        gOutput[posInOutputTex] = float4(1.0f, 0.0f, 0.0f, 1.0f);
    }
}

// Or visualize all ISMs based on light ID
[numthreads(16, 16, 1)]
void allIsmCS2(uint3 threadID: SV_DispatchThreadID)
{
    uint lightIsmIndex = threadID.z;
    uint2 posInIsm = threadID.xy;

    uint localLightIdx = lightIsmIndex / gIsmPerLight;
    uint faceIdx = lightIsmIndex % gIsmPerLight;
    LightShadowMapData lightShadowData = gLightShadowDataBuffer[localLightIdx];
    LightFaceData lightFaceData = lightShadowData.lightFaceData[faceIdx];
    uint lightShadowMapType = lightShadowData.lightFaceData[gSortingRules == (uint)SortingRules::LightFaces ? faceIdx : 0].shadowMapType;

    // Get the index for current ISM texture on the output texture
    const uint rowSize = gOutputSize.x / gInputSize.x;
    uint2 ismOffset = uint2(lightIsmIndex % rowSize, lightIsmIndex / rowSize);

    // Map the position in ISM to output texture
    uint2 posInOutputTex = posInIsm + ismOffset * gInputSize;

    // Get the depth value from input ISM texture
    if (lightShadowMapType == 2)
    {
        float3 location = float3(posInIsm / float2(gInputSize), lightFaceData.ismTexArrayIdx);
        float sampleDepth = gInputSortedSMs[lightFaceData.whichIsmTexArray].SampleLevel(gPointSampler, location, 0);
        float depthView = projToView(sampleDepth, lightShadowData.nearFarPlane.x, lightShadowData.nearFarPlane.y);
        gOutput[posInOutputTex] = float4(float3(abs(depthView) / lightShadowData.nearFarPlane.y), 1.0f);

    }
    else
    {
        // gOutput[posInOutputTex] = float4(abs(depthView) / lightShadowData.nearFarPlane.y, 0.0f, 0.0f, 1.0f);
        gOutput[posInOutputTex] = float4(1.0f, 0.0f, 0.0f, 1.0f);
    }
}
