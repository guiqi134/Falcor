import Scene.Scene;
import Utils.Debug.PixelDebug;
import ISMHelpers;

cbuffer CB
{
    bool gEnableEmissiveLightSampling;
    uint gFirstSpotLightID;
    uint gShadowMapsPerPointLight;
    float gAlphaBlendingFactor;
    uint gHistoryMax;
    uint gBoundaryIdx;
    uint gIterationCount;
    bool gCompareToBoundary;
    uint gSortingRules;
    uint gFlickerReduction;
}

RWStructuredBuffer<LightShadowMapData> gLightShadowDataBuffer;

// Updating light shadow map data for the parts changed in scene animation
StructuredBuffer<LightShadowMapData> gNewLightShadowDataBuffer;

[numthreads(256, 1, 1)]
void updateCenterCS(uint3 threadId: SV_DispatchThreadID)
{
    // printSetPixel(threadId.xy);

    // Update light center
    if (gEnableEmissiveLightSampling)
    {
        gLightShadowDataBuffer[threadId.x].centerPosW = gNewLightShadowDataBuffer[threadId.x].centerPosW;
    }
    else
    {
        uint falcorLightID = gLightShadowDataBuffer[threadId.x].falcorLightID;
        gLightShadowDataBuffer[threadId.x].centerPosW = gScene.getLight(falcorLightID).posW;
    }

    // Since center changes, we also need to update light view matrix
    float3 newCenter = gLightShadowDataBuffer[threadId.x].centerPosW;
    for (uint face = 0; face < gLightShadowDataBuffer[threadId.x].numShadowMaps; face++)
    {
        float4x4 prevViewMat = gLightShadowDataBuffer[threadId.x].lightFaceData[face].viewMat;
        float4x4 viewRotation = float4x4(prevViewMat[0], prevViewMat[1], prevViewMat[2], float4(0, 0, 0, 1));
        float4x4 viewNewTranslation = translation(-newCenter.x, -newCenter.y, -newCenter.z);
        gLightShadowDataBuffer[threadId.x].lightFaceData[face].viewMat = mul(viewNewTranslation, viewRotation);
    }
}

RWBuffer<float> gBoundaryVarianceBuffer;
Buffer<uint2> gSortedLightsBuffer; // only need top N light/face index

[numthreads(256, 1, 1)]
void generalUpdates(uint3 threadId: SV_DispatchThreadID)
{
    printSetPixel(threadId.xy);

    print("gIterationCount =", gIterationCount);

    uint2 localLightIDAndFaceIdx = getLocalLightIdAndFaceIdx(gSortedLightsBuffer[threadId.x].x, gFirstSpotLightID);
    uint localLightID = localLightIDAndFaceIdx.x;
    uint faceIdx = localLightIDAndFaceIdx.y;
    uint frequency = gSortedLightsBuffer[threadId.x].y;

    print("localLightIDAndFaceIdx = ", localLightIDAndFaceIdx);

    gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx].currRanking = threadId.x;
    gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx].accumulatedFrames += 1;
    // gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx].psmAge += 1;
    gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx].frequency = frequency;

    // Reset light's average direction
    // gLightShadowDataBuffer[localLightID].avgDirection = float3(0.0f);

    // Update mean and stdDev
    if (gFlickerReduction == (uint)FlickerReduction::VarianceCheck && gIterationCount > 0)
    {
        float n = gIterationCount;
        // float n = min(gHistoryMax, gIterationCount);
        float prevMean = gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx].mean;
        float prevVar = gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx].varianceUnbiased;

        print("n = ", n);
        print("prevMean = ", prevMean);
        print("prev std = ", sqrt(prevVar));

        float x = frequency;
        float mean = 0.0f;
        // float variance = 0.0f;
        float varianceUnbiased = 0.0f;

        // Weighted mean and variance
        mean = (1 - gAlphaBlendingFactor) * prevMean + gAlphaBlendingFactor * x;
        varianceUnbiased = n > 1 ? (1 - gAlphaBlendingFactor) * prevVar + gAlphaBlendingFactor * (x - prevMean) * (x - mean) : 0.0;

        gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx].mean = mean;
        gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx].varianceUnbiased = varianceUnbiased;

        print("x = ", x);
        print("mean = ", mean);
        print("unbiased std = ", sqrt(varianceUnbiased));
        print("boundary freq = ", gSortedLightsBuffer[gBoundaryIdx].y);

        // Also update the boundary variance
        if (gCompareToBoundary && threadId.x == gBoundaryIdx)
        {
            gBoundaryVarianceBuffer[2] = x;

            float prevBoundaryMean = gBoundaryVarianceBuffer[0];
            float prevBoundaryVar = gBoundaryVarianceBuffer[1];

            // Weighted mean and variance
            gBoundaryVarianceBuffer[0] = (1 - gAlphaBlendingFactor) * prevBoundaryMean + gAlphaBlendingFactor * x;
            gBoundaryVarianceBuffer[1] = n > 1 ? (1 - gAlphaBlendingFactor) * prevBoundaryVar + gAlphaBlendingFactor * (x - prevBoundaryMean) * (x - gBoundaryVarianceBuffer[0]) : 0.0;

            print("boundary mean = ", gBoundaryVarianceBuffer[0]);
            print("boundary unbiased std = ", sqrt(gBoundaryVarianceBuffer[1]));
        }
    }
}

[numthreads(256, 1, 1)]
void updatePrevRanking(uint3 threadId: SV_DispatchThreadID)
{
    uint2 localLightIDAndFaceIdx = getLocalLightIdAndFaceIdx(threadId.x, gFirstSpotLightID);
    uint localLightID = localLightIDAndFaceIdx.x;
    uint faceIdx = localLightIDAndFaceIdx.y;
    gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx].prevRanking = gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx].currRanking;
}
