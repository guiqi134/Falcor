// #include "../HostDeviceSharedDefinitions.h"

import Utils.Debug.PixelDebug;
import ISMHelpers;

#ifndef NUM_INPUT_TEXTURE_ARRAYS
#define NUM_INPUT_TEXTURE_ARRAYS 3
#endif

Texture2DArray<float> gInputSortedSMs[NUM_INPUT_TEXTURE_ARRAYS];
Texture2DArray<float4> gInputSortedMotionTex[NUM_INPUT_TEXTURE_ARRAYS];

StructuredBuffer<LightShadowMapData> gLightShadowDataBuffer;
Buffer<uint2> gSortedLightsBuffer;
Buffer<uint> gHighResIsmFaceIdxBuffer;
Buffer<uint> gReusingLightIndexBuffer;
RWTexture2D<float4> gOutput;

cbuffer VisCB
{
    float gMipLevel;
    uint2 gLightFaceIndex;
    uint2 gInputSize;
    uint2 gOutputSize;
    uint gIsmPerLight;
    uint gSortingRules;
    uint gShadowMapType;
    uint gFirstSpotLightID;
    SamplerState gPointSampler;
    SamplerState gLinearSampler;
}

float projToView(float depth, float near, float far)
{
    return (near * far) / (depth * (far - near) - far);
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
    if (lightFaceData.shadowMapType != 1 || lightFaceData.psmTexArrayIdx == -1)
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

// Visualize single ISM
void singleIsmCS(uint3 threadID: SV_DispatchThreadID)
{
    printSetPixel(threadID.xy);

    LightShadowMapData lightShadowData = gLightShadowDataBuffer[gLightFaceIndex.x];
    LightFaceData lightFaceData = lightShadowData.lightFaceData[gLightFaceIndex.y];

    print("gLightFaceIndex = ", gLightFaceIndex);
    print("gShadowMapType = ", gShadowMapType);
    print("lightShadowData.nearFarPlane = ", lightShadowData.nearFarPlane);

    // Visualize ISM
    float3 location = float3(float2(threadID.xy) / float2(gInputSize), 0.0f);
    location.z = float(gShadowMapType == uint(ShadowMapType::LowIsm) ? lightFaceData.ismTexArrayIdx : lightFaceData.highResIsmTexArrayIdx);
    print("location = ", location);
    uint whichTexArray = gShadowMapType == uint(ShadowMapType::LowIsm) ? lightFaceData.whichIsmTexArray : lightFaceData.whichHighResIsmTexArray;
    float sampleDepth = gInputSortedSMs[whichTexArray].SampleLevel(gPointSampler, location, gMipLevel);
    float depthView = projToView(sampleDepth, lightShadowData.nearFarPlane.x, lightShadowData.nearFarPlane.y);
    print("whichTexArray = ", whichTexArray);
    print("sampleDepth = ", sampleDepth);
    print("depthView = ", depthView);
    gOutput[threadID.xy] = float4(float3(abs(depthView) / lightShadowData.nearFarPlane.y), 1.0f);

    if (gShadowMapType == uint(ShadowMapType::HighIsm) && lightFaceData.shadowMapType != uint(ShadowMapType::HighIsm))
    {
        gOutput[threadID.xy] = float4(1.0f);
    }

    print("color = ", gOutput[threadID.xy]);

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

    // TODO: this has bugs after changing to point lights, fix this!!!
    uint lightOrGlobalFaceIdx = gHighResIsmFaceIdxBuffer[(gSortingRules != (uint)SortingRules::Light && gSortingRules != (uint)SortingRules::Light_OccludedPixels) ? ismTexGlobalIdx : ismTexGlobalIdx / gIsmPerLight];
    uint2 localLightIDAndFaceIdx = getLocalLightIdAndFaceIdx(lightOrGlobalFaceIdx, gFirstSpotLightID);
    uint localLightID = localLightIDAndFaceIdx.x;
    uint faceIdx = localLightIDAndFaceIdx.y;
    LightShadowMapData lightShadowData = gLightShadowDataBuffer[localLightID];
    LightFaceData lightFaceData = lightShadowData.lightFaceData[faceIdx];
    uint lightShadowMapType = lightShadowData.lightFaceData[(gSortingRules != (uint)SortingRules::Light && gSortingRules != (uint)SortingRules::Light_OccludedPixels) ? faceIdx : 0].shadowMapType;

    // Get the index for current ISM texture on the output texture
    const uint rowSize = gOutputSize.x / gInputSize.x;
    uint2 ismOffset = uint2(ismTexGlobalIdx % rowSize, ismTexGlobalIdx / rowSize);

    // Map the position in ISM to output texture
    uint2 posInOutputTex = posInIsm + ismOffset * gInputSize;

    // Get the depth value from input ISM texture
    float3 location = float3(posInIsm / float2(gInputSize), ismTexGlobalIdx);
    uint whichTexArray = gShadowMapType == uint(ShadowMapType::LowIsm) ? lightFaceData.whichIsmTexArray : lightFaceData.whichHighResIsmTexArray;
    float sampleDepth = gInputSortedSMs[whichTexArray].SampleLevel(gPointSampler, location, 0);
    float depthView = projToView(sampleDepth, lightShadowData.nearFarPlane.x, lightShadowData.nearFarPlane.y);
    gOutput[posInOutputTex] = float4(float3(abs(depthView) / lightShadowData.nearFarPlane.y), 1.0f);

    if (lightShadowMapType == uint(ShadowMapType::PSM))
    {
        gOutput[posInOutputTex] = float4(1.0f, 0.0f, 0.0f, 1.0f);
    }
    else if (lightShadowMapType == uint(ShadowMapType::LowIsm))
    {
        gOutput[posInOutputTex] = float4(0.0f, 1.0f, 0.0f, 1.0f);
    }
    else if (lightShadowMapType != gShadowMapType)
    {
        gOutput[posInOutputTex] = float4(0.0f, 0.0f, 1.0f, 1.0f);
    }
}

// Or visualize all ISMs based on light ID
[numthreads(16, 16, 1)]
void allIsmCS2(uint3 threadID: SV_DispatchThreadID)
{
    printSetPixel(threadID.zx);

    uint lightIsmIndex = threadID.z;
    uint2 posInIsm = threadID.xy;

    uint2 localLightIDAndFaceIdx = getLocalLightIdAndFaceIdx(lightIsmIndex, gFirstSpotLightID);
    uint localLightID = localLightIDAndFaceIdx.x;
    uint faceIdx = localLightIDAndFaceIdx.y;
    LightShadowMapData lightShadowData = gLightShadowDataBuffer[localLightID];
    LightFaceData lightFaceData = lightShadowData.lightFaceData[faceIdx];
    uint lightShadowMapType = lightShadowData.lightFaceData[(gSortingRules != (uint)SortingRules::Light && gSortingRules != (uint)SortingRules::Light_OccludedPixels) ? faceIdx : 0].shadowMapType;

    // Get the index for current ISM texture on the output texture
    const uint rowSize = gOutputSize.x / gInputSize.x;
    uint2 ismOffset = uint2(lightIsmIndex % rowSize, lightIsmIndex / rowSize);

    // Map the position in ISM to output texture
    uint2 posInOutputTex = posInIsm + ismOffset * gInputSize;

    // Get the depth value from input ISM texture
    if (lightShadowMapType == gShadowMapType)
    {
        float3 location = float3(posInIsm / float2(gInputSize), 0.0);
        location.z = float(gShadowMapType == uint(ShadowMapType::LowIsm) ? lightFaceData.ismTexArrayIdx : lightFaceData.highResIsmTexArrayIdx);
        uint whichTexArray = gShadowMapType == uint(ShadowMapType::LowIsm) ? lightFaceData.whichIsmTexArray : lightFaceData.whichHighResIsmTexArray;
        float sampleDepth = gInputSortedSMs[whichTexArray].SampleLevel(gPointSampler, location, 0);
        float depthView = projToView(sampleDepth, lightShadowData.nearFarPlane.x, lightShadowData.nearFarPlane.y);
        gOutput[posInOutputTex] = float4(float3(abs(depthView) / lightShadowData.nearFarPlane.y), 1.0f);
    }
    else if (lightShadowMapType == uint(ShadowMapType::PSM))
    {
        // gOutput[posInOutputTex] = float4(abs(depthView) / lightShadowData.nearFarPlane.y, 0.0f, 0.0f, 1.0f);
        gOutput[posInOutputTex] = float4(1.0f, 0.0f, 0.0f, 1.0f);
    }
    else if (lightShadowMapType == uint(ShadowMapType::HighIsm))
    {
        gOutput[posInOutputTex] = float4(0.0f, 1.0f, 0.0f, 1.0f);
    }
    else
    {
        gOutput[posInOutputTex] = float4(0.0f, 0.0f, 1.0f, 1.0f);
    }
}

[numthreads(16, 16, 1)]
void allSortedPsmCS(uint3 threadID : SV_DispatchThreadID)
{
    // printSetPixel(threadID.xy);

    uint psmTexGlobalIdx = threadID.z;
    uint2 posInIsm = threadID.xy;

    uint globalFaceID = gReusingLightIndexBuffer[psmTexGlobalIdx];
    uint2 localLightIDAndFaceIdx = getLocalLightIdAndFaceIdx(globalFaceID, gFirstSpotLightID);
    uint localLightID = localLightIDAndFaceIdx.x;
    uint faceIdx = localLightIDAndFaceIdx.y;
    LightShadowMapData lightShadowData = gLightShadowDataBuffer[localLightID];
    LightFaceData lightFaceData = lightShadowData.lightFaceData[faceIdx];
    uint lightShadowMapType = lightFaceData.shadowMapType;

    // Get the index for current ISM texture on the output texture
    const uint rowSize = gOutputSize.x / gInputSize.x;
    uint2 psmOffset = uint2(psmTexGlobalIdx % rowSize, psmTexGlobalIdx / rowSize);

    // Map the position in ISM to output texture
    uint2 posInOutputTex = posInIsm + psmOffset * gInputSize;

    // Get the depth value from input ISM texture
    float3 location = float3(posInIsm / float2(gInputSize), psmTexGlobalIdx);
    uint whichTexArray = lightFaceData.whichPsmTexArray;
    float sampleDepth = gInputSortedSMs[whichTexArray].SampleLevel(gPointSampler, location, 0);
    float depthView = projToView(sampleDepth, lightShadowData.nearFarPlane.x, lightShadowData.nearFarPlane.y);
    gOutput[posInOutputTex] = float4(float3(abs(depthView) / lightShadowData.nearFarPlane.y), 1.0f);
}
