// #include "../HostDeviceSharedDefinitions.h"
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

// TODO: define on CPU side
#ifndef NUM_ISM_TEXTURE_ARRAYS
#define NUM_ISM_TEXTURE_ARRAYS 3
#endif

#ifndef DEBUG_PRINT_MODE
#define DEBUG_PRINT_MODE 0
#endif

cbuffer ismCB
{
    float gBaseTriSize;
    uint gTotalLightsCount;
    uint gTotalIsmCount;
    uint gLaunchWidth;
    uint gTotalTriangleCount;
    uint gSortingRules;
    int gDebugLightID;
    // Below are parameters for high ISM
    uint gTotalHighResIsmCount;
    uint gFirstSpotLightID;

    // Overloaded parameters
    bool gUseHighResISM;
    uint gIsmSize;
}

StructuredBuffer<LightShadowMapData> gLightShadowDataBuffer;
Buffer<uint> gInstTriOffsetBuffer;
Buffer<uint> gHighResIsmFaceIdxBuffer;

RWBuffer<uint> gExtraPointsCountBuffer;
RWBuffer<uint> gPerLightCountingBuffer;
RWBuffer<uint> gPerFaceCountingBuffer;

// Low-res or High-res ISMs in light ranking order
RWTexture2DArray<uint> gSortedLightsISMsUint[NUM_ISM_TEXTURE_ARRAYS];
RWTexture2DArray<float4> gSortedIsmRadiusNormal[NUM_ISM_TEXTURE_ARRAYS];

[numthreads(16, 16, 1)]
void renderIsmCS(uint3 threadID: SV_DispatchThreadID)
{
    uint globalThreadID = threadID.y * gLaunchWidth + threadID.x;

    // Discard the thread if we go out of range
    if (globalThreadID >= gTotalTriangleCount)
        return;

    // printSetPixel(threadID.xy);
    print("ISM Render Pass 11111111111111111111111111111111111111", 1);

    // Get instanceID using binary search
    uint id = findInterval(gInstTriOffsetBuffer, globalThreadID);
    GeometryInstanceID instanceID = { id };
    print("instanceID = ", id);

    // Get triangle index
    uint triangleIndex = globalThreadID - gInstTriOffsetBuffer[id];
    SampleGenerator sg = SampleGenerator(uint2(instanceID.index, triangleIndex), 1 * 13);
    SampleGenerator sgFixed = SampleGenerator(uint2(1, triangleIndex), 1 * 13);
    print("triangleIndex = ", triangleIndex);

    // Get the information of current triangle
    float3 triVertices[3];
    float triArea;
    gScene.getVertexPositionsW(instanceID, triangleIndex, triVertices);
    float3 triNormalW = gScene.getFaceNormalAndAreaW(instanceID, triangleIndex, triArea);
    print("triArea = ", triArea);

    // Cull emissive triangles
    uint materialID = gScene.getMaterialID(instanceID);
    if (gScene.materials.isBasicMaterial(materialID))
    {
        float16_t3 emissive = gScene.materials.getBasicMaterialData(materialID).emissive;

        // Some objects have emissive textures (e.g. the train in Subway), so we cannot just ignore them
        if (!gScene.materials.isEmissiveTextured(materialID) && any(emissive != float16_t3(0)))
            return;
    }

    // Compute how many points to generate
    uint numPoints = 3 + clamp(uint(triArea / gBaseTriSize), 0, MAX_EXTRA_POINTS);
#if DEBUG_PRINT_MODE
    InterlockedAdd(gExtraPointsCountBuffer[numPoints - 3], 1);
#endif
    print("numPoints = ", numPoints);

    // // Approximate the radius for all the points
    // //  -> Actually we should compute it in light space, but since there is no scaling applied from world -> light space, it's ok
    // //     to compute the length in world space
    // float3 triCenter = (triVertices[0] + triVertices[1] + triVertices[2]) / 3.0f;
    // float maxDist = max(max(length(triVertices[0] - triCenter), length(triVertices[1] - triCenter)), length(triVertices[2] - triCenter));
    // float radius = 3.0 * maxDist / float(numPoints);

    // Render each point.
    // TODO: this can be seperate into another CS, not sure if this is going to improve the performance.
    //  -> Maybe perform a sorting to make the point buffer more coherent in terms of light ID
    for (uint v = 0; v < numPoints; v++)
    {
        uint localLightID;
        uint faceIdx;
        if (gUseHighResISM)
        {
            uint highResIsmArrayIdx = uint(gTotalHighResIsmCount * sampleNext1D(sg));
            uint globalFaceID = gHighResIsmFaceIdxBuffer[highResIsmArrayIdx];
            uint2 localLightIDAndFaceIdx = getLocalLightIdAndFaceIdx(globalFaceID, gFirstSpotLightID);
            localLightID = localLightIDAndFaceIdx.x;
            faceIdx = localLightIDAndFaceIdx.y;
        }
        else
        {
            uint globalFaceID = uint(gTotalIsmCount * sampleNext1D(sg));
            uint2 localLightIDAndFaceIdx = getLocalLightIdAndFaceIdx(globalFaceID, gFirstSpotLightID);
            localLightID = localLightIDAndFaceIdx.x;
        }
        print("localLightID = ", localLightID);

        LightShadowMapData lightShadowData = gLightShadowDataBuffer[localLightID];

        // Cull the point if it is backfacing towards the light. Pick any point on the triangle and check the cosine value
        // if (dot(normalize(lightShadowData.centerPosW - triVertices[0]), triNormalW) < 0.0f) continue;
#if DEBUG_PRINT_MODE
        InterlockedAdd(gPerLightCountingBuffer[localLightID], 1);
#endif

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

        // Only perspective projection is going to affect the length
        float3 projTriVertices[3];
        for (uint i = 0; i < 3; i++)
        {
            float4 clipTriVertices = mul(float4(triVertices[i], 1.0f), lightShadowData.persProjMat);
            projTriVertices[i] = clipTriVertices.xyz / clipTriVertices.w;
        }
        float3 projTriCenter = (projTriVertices[0] + projTriVertices[1] + projTriVertices[2]) / 3.0f;
        float maxDist = max(max(length(projTriVertices[0] - projTriCenter), length(projTriVertices[1] - projTriCenter)), length(projTriVertices[2] - projTriCenter));
        float radiusNDC = 3.0 * maxDist / float(numPoints);

        // print("vertexPosW = ", vertexPosW);

        // Project the point to the light face
        float3 lightCenterW = lightShadowData.centerPosW;
        float3 lightToPoint = vertexPosW - lightCenterW;
        if (!gUseHighResISM) faceIdx = getClosestFace(normalize(lightToPoint), lightShadowData);
        float4x4 lightView = lightShadowData.lightFaceData[faceIdx].viewMat;
        float3 posLightView = mul(float4(vertexPosW, 1.0f), lightView).xyz;
        float4 posLightClipping = mul(float4(posLightView, 1.0f), lightShadowData.persProjMat);
        float3 posLightNDC = posLightClipping.xyz / posLightClipping.w;

        // Also project point (triangle) normal to light view space
        float4 normalLightClipping = mul(mul(float4(triNormalW, 0.0f), lightShadowData.lightFaceData[faceIdx].normalViewMat), lightShadowData.normalPersProjMat);
        float3 normalLightNDC = normalLightClipping.xyz / normalLightClipping.w;

        float3 ismProjCoord = float3(posLightNDC.xy * float2(0.5f, -0.5f) + 0.5f, posLightNDC.z);

        print("ismProjCoord = ", ismProjCoord);
        print("faceIdx = ", faceIdx);
        print("gIsmSize = ", gIsmSize);
        print("posLightView = ", posLightView);
        // print("lightShadowData.nearFarPlane = ", lightShadowData.nearFarPlane);

        // Cull the point if it is behind the near plane. Note: view direction is -z
        if (posLightView.z > -lightShadowData.nearFarPlane.x) continue;

#if DEBUG_PRINT_MODE
        // Count if it is a valid point
        uint globalIsmID = localLightID < gFirstSpotLightID ? kShadowMapsPerPointLight * localLightID + faceIdx :
            kShadowMapsPerPointLight * gFirstSpotLightID + (localLightID - gFirstSpotLightID);
        InterlockedAdd(gPerFaceCountingBuffer[globalIsmID], 1);
#endif

        // Transform to pixel position in ISM texture
        uint2 ismTexPixelPos = uint2(ismProjCoord.xy * gIsmSize);
        float depth = ismProjCoord.z;

        // Perform depth testing using atomic operation
        uint3 ismTexArrayLoc = uint3(ismTexPixelPos, 0);
        ismTexArrayLoc.z = gUseHighResISM ? lightShadowData.lightFaceData[faceIdx].highResIsmTexArrayIdx : lightShadowData.lightFaceData[faceIdx].ismTexArrayIdx;
        uint whichTexArray = gUseHighResISM ? lightShadowData.lightFaceData[faceIdx].whichHighResIsmTexArray : lightShadowData.lightFaceData[faceIdx].whichIsmTexArray;
        uint originDepth;
        print("ismTexArrayLoc = ", ismTexArrayLoc);
        InterlockedMin(gSortedLightsISMsUint[whichTexArray][ismTexArrayLoc], asuint(depth), originDepth);

        print("asuint(depth) = ", asuint(depth));
        print("originDepth = ", originDepth);

        // Wait until all point's depth is tested
        DeviceMemoryBarrier();

        // Write additional data for the frontmost point
        if (asuint(depth) <= gSortedLightsISMsUint[whichTexArray][ismTexArrayLoc])
        {
            gSortedIsmRadiusNormal[whichTexArray][ismTexArrayLoc] = float4(radiusNDC, normalLightNDC.xy, triNormalW.z);
        }
    }
}
