#include "../HostDeviceSharedDefinitions.h"
#include "Utils/Math/MathConstants.slangh"

import Scene.Scene;
import Utils.Sampling.TinyUniformSampleGenerator;
import Utils.Math.MathHelpers;
import Utils.Debug.PixelDebug;
import ISMHelpers;

typedef TinyUniformSampleGenerator SampleGenerator;

#ifndef MAX_EXTRA_POINTS
#define MAX_EXTRA_POINTS 32
#endif

#ifndef NUM_ISM_TEXTURE_ARRAYS
#define NUM_ISM_TEXTURE_ARRAYS 3
#endif

cbuffer ismCB
{
    float gBaseTriSize;
    uint gTotalLightOrIsmCount;
    uint gTotalLightsCount;
    bool gIsmLinearProjection;
    uint gIsmSize;
    uint gLaunchWidth;
    uint gTotalTriangleCount;
    uint gPassIndex;
    uint gTotalPasses;
    uint gSortingRules;
    uint gIsmPerLight;
}

StructuredBuffer<LightShadowMapData> gLightShadowDataBuffer;
Buffer<uint> gTriOffsetBuffer;
RWBuffer<uint> gExtraPointsCountBuffer;
RWBuffer<uint> gCounterBuffer; // Counter for each light or face

// All light's ISMs in light ranking order
RWTexture2DArray<uint> gSortedLightsISMsUint[NUM_ISM_TEXTURE_ARRAYS];

[numthreads(16, 16, 1)]
void renderIsmCS(uint3 threadID: SV_DispatchThreadID)
{
    uint globalThreadID = threadID.y * gLaunchWidth + threadID.x;

    // Discard the thread if we go out of range
    if (globalThreadID >= gTotalTriangleCount)
        return;

    // printSetPixel(threadID.xy);

    // Get instanceID using binary search
    uint id = findInterval(gTriOffsetBuffer, globalThreadID);
    GeometryInstanceID instanceID = { id };
    print("instanceID = ", id);

    // Get triangle index
    uint triangleIndex = globalThreadID - gTriOffsetBuffer[id];
    SampleGenerator sg = SampleGenerator(uint2(instanceID.index, triangleIndex), 1 * 13);
    SampleGenerator sgFixed = SampleGenerator(uint2(1, triangleIndex), 1 * 13);
    print("triangleIndex = ", triangleIndex);

    // Get the information of current triangle
    float3 triVertices[3];
    float triArea;
    gScene.getVertexPositionsW(instanceID, triangleIndex, triVertices);
    float3 triNormalW = gScene.getFaceNormalAndAreaW(instanceID, triangleIndex, triArea);
    print("triArea = ", triArea);

    // Compute how many points to generate
    uint numPoints = 3 + clamp(uint(triArea / gBaseTriSize), 0, MAX_EXTRA_POINTS);
    InterlockedAdd(gExtraPointsCountBuffer[numPoints - 3], 1);
    print("numPoints = ", numPoints);

    // Render each point.
    // TODO: this can be seperate into another CS, not sure if this is going to improve the performance.
    //  -> Maybe perform a sorting to make the point buffer more coherent in terms of light ID
    for (uint v = 0; v < numPoints; v++)
    {
        // uint lightOrGlobalIsmPerPass = gTotalLightOrIsmCount / gTotalPasses;
        // uint lightOrGlobalIsmIdx = uint(lightOrGlobalIsmPerPass * (sampleNext1D(sg) + gPassIndex));
        uint lightCountPerPass = gTotalLightsCount / gTotalPasses;
        uint localLightIdx = uint(lightCountPerPass * (sampleNext1D(sg) + gPassIndex));

        if (localLightIdx >= gTotalLightsCount)
            continue;
        InterlockedAdd(gCounterBuffer[localLightIdx], 1);

        // uint localLightIdx = gSortingRules == (uint)SortingRules::LightFaces ? lightOrGlobalIsmIdx / gIsmPerLight : lightOrGlobalIsmIdx;
        // uint ismIdx = gSortingRules == (uint)SortingRules::LightFaces ? lightOrGlobalIsmIdx % gIsmPerLight : 0u;
        LightShadowMapData lightShadowData = gLightShadowDataBuffer[localLightIdx];

        // Cull the point if it is backfacing towards the light. Pick any point on the triangle and check the cosine value
        if (dot(normalize(lightShadowData.centerPosW - triVertices[0]), triNormalW) < 0.0f)
            continue;

        // Directly get the world position for first 3 vertices and randomly interpolate inside the triangle for the rest
        float3 vertexPosW;
        if (v < 3)
        {
            vertexPosW = triVertices[v];
        }
        else
        {
            // TODO: change to low discrepency sequence (like R2 sequence) and pre-computed those barycentrics
            float3 barycentrics = sample_triangle(sampleNext2D(sgFixed)); // Note: we need to use different sg for sampling triangle to avoid getting different values
            vertexPosW = barycentrics.x * triVertices[0] + barycentrics.y * triVertices[1] + barycentrics.z * triVertices[2];
        }

        // Project the point to the light face
        float3 lightCenterW = lightShadowData.centerPosW;
        uint ismIdx;
        float3 posLightView;
        float3 posLightNDC;
        if (gIsmLinearProjection)
        {
            float3 lightToPoint = vertexPosW - lightCenterW;
            ismIdx = getFaceIndex(normalize(lightToPoint));
            float4x4 lightView = lightShadowData.lightFaceData[ismIdx].viewMat;
            posLightView = mul(float4(vertexPosW, 1.0f), lightView).xyz;
            float4 posLightClipping = mul(float4(posLightView, 1.0f), lightShadowData.persProjMat);
            posLightNDC = posLightClipping.xyz / posLightClipping.w;
        }
        else
        {
            ismIdx = vertexPosW.y <= lightCenterW.y ? 0 : 1;
            float4x4 viewMat = lightShadowData.lightFaceData[3 - ismIdx].viewMat;
            posLightView = mul(float4(vertexPosW, 1.0f), viewMat).xyz;
            posLightNDC = paraboloidProjection(posLightView, lightShadowData.nearFarPlane.x, lightShadowData.nearFarPlane.y);
        }
        float3 ismProjCoord = float3(posLightNDC.xy * float2(0.5f, -0.5f) + 0.5f, posLightNDC.z);

        // Cull the point if it is behind the near plane. Note: view direction is -z
        if (posLightView.z > -lightShadowData.nearFarPlane.x) continue;

        // Transform to pixel position in ISM texture
        uint2 ismTexPixelPos = uint2(ismProjCoord.xy * gIsmSize);
        float depth = ismProjCoord.z;

        // Perform depth testing using atomic operation
        uint3 ismTexArrayLoc = uint3(ismTexPixelPos, lightShadowData.lightFaceData[ismIdx].ismTexArrayIdx);
        uint originDepth;
        InterlockedMin(gSortedLightsISMsUint[lightShadowData.lightFaceData[ismIdx].whichIsmTexArray][ismTexArrayLoc], asuint(depth), originDepth);
    }
}

cbuffer convertCB
{
    uint gMaxIsmPerArray;
}

RWTexture2DArray<float> gSortedLightsISMs[3];

[numthreads(16, 16, 1)]
void uint2FloatCS(uint3 threadID : SV_DispatchThreadID)
{
    uint3 ismTexArrayLoc = uint3(threadID.xy, threadID.z % gMaxIsmPerArray);
    uint whichIsmTexArray = uint(threadID.z / gMaxIsmPerArray);
    gSortedLightsISMs[whichIsmTexArray][ismTexArrayLoc] = asfloat(gSortedLightsISMsUint[whichIsmTexArray][ismTexArrayLoc]);
}
