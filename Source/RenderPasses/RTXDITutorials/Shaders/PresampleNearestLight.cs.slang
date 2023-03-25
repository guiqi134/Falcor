#include "Utils/Math/MathConstants.slangh"

import Utils.Debug.PixelDebug;
import Scene.Scene;
import Utils.Sampling.TinyUniformSampleGenerator;
import ISMHelpers;

typedef TinyUniformSampleGenerator SampleGenerator;

cbuffer nearestCB
{
    uint gFirstNearestPointSample;
    uint gNumNearestPointSamples;
    uint gLaunchWidth;
    uint gTotalLightsCount;
}

StructuredBuffer<LightShadowMapData> gLightShadowDataBuffer;
RWStructuredBuffer<PackedIsmPointSample> gPointsBuffer;

[numthreads(16, 16, 1)]
void main(uint3 threadID: SV_DispatchThreadID)
{
    uint pointsBufferIdx = gFirstNearestPointSample + (threadID.y * gLaunchWidth + threadID.x);
    if (pointsBufferIdx >= gFirstNearestPointSample + gNumNearestPointSamples) return;

    // Compute the nearest light across all lights
    IsmPointSample pointSample = unpackIsmPointSample(gPointsBuffer[pointsBufferIdx]);
    GeometryInstanceID instanceID = { pointSample.instanceID };
    float3 posW = mul(float4(pointSample.pos, 1.0f), gScene.getWorldMatrix(instanceID)).xyz;

    float nearestDist = FLT_MAX;
    uint nearestLightID = 0;
    for (uint i = 0; i < gTotalLightsCount; i++)
    {
        LightShadowMapData lightShadowData = gLightShadowDataBuffer[i];
        // if (lightShadowData.isDynamic) continue;

        float3 lightPosW = lightShadowData.centerPosW;
        float dist = length(posW - lightPosW);
        if (dist < nearestDist)
        {
            nearestDist = dist;
            nearestLightID = i;
        }
    }
    pointSample.selectedLightID = nearestLightID;

    gPointsBuffer[pointsBufferIdx] = packIsmPointSample(pointSample);
}
