// #include "../HostDeviceSharedDefinitions.h"
#include "Utils/Math/MathConstants.slangh"

import Utils.Debug.PixelDebug;
import ISMHelpers;

cbuffer rotateCB
{
}

RWStructuredBuffer<LightShadowMapData> gLightShadowDataBuffer;

static const float3 faceUpVectors[6] = {
    float3(0.0f, 1.0f, 0.0f), float3(0.0f, 1.0f, 0.0f), float3(0.0f, 0.0f, 1.0f),
    float3(0.0f, 0.0f, -1.0f), float3(0.0f, 1.0f, 0.0f), float3(0.0f, 1.0f, 0.0f)
};

// Note: DX is row-major, skew Symmetric matrix should be transposed from math formula
inline float3x3 skewSymmetricMat(float3 v)
{
    return float3x3(
        0.0, v.z, -v.y,
        -v.z, 0.0, v.x,
        v.y, -v.x, 0.0
    );
}

// v1 and v2 have to be unit vector. Return rotation from v1 to v2
float4x4 calcRotationFromTwoVectors(float3 v1, float3 v2)
{
    float3 k = cross(v1, v2);
    float s = length(k);
    float c = dot(v1, v2);

    // Clamp the rotation if it's too large
    // c = max(c, cos(M_PI / 180.0f));

    print("c = ", c);

    // Apply Rodrigues's rotation formula
    float3x3 skewSymmMat = skewSymmetricMat(k);
    float3x3 R = identity3x3 + skewSymmMat + (1.0f / (1.0f + c)) * mul(skewSymmMat, skewSymmMat);

    printMatrix3x3(R, "R = ");

    return float4x4(
        R._11, R._12, R._13, 0.0,
        R._21, R._22, R._23, 0.0,
        R._31, R._32, R._33, 0.0,
        0.0, 0.0, 0.0, 1.0
    );
}

[numthreads(256, 1, 1)]
void main(uint3 threadID: SV_DispatchThreadID)
{
    // printSetPixel(threadID.xy);

    uint lightID = threadID.x;
    LightShadowMapData lightShadowData = gLightShadowDataBuffer[lightID];
    float3 lightPosW = lightShadowData.centerPosW;

    // Compute the weighted average direction
    float alpha = 0.1f;
    float3 lightAvgDir = (1.0f - alpha) * lightShadowData.accumAvgDirection + alpha * lightShadowData.currAvgDirection;
    // float3 lightAvgDir = lightShadowData.currAvgDirection;

    print("light center = ", lightShadowData.centerPosW);

    // Update average direction
    gLightShadowDataBuffer[lightID].accumAvgDirection = lightAvgDir;
    gLightShadowDataBuffer[lightID].currAvgDirection = float3(0.0f);

    // Debugging
    for (uint face = 0; face < lightShadowData.numShadowMaps; face++)
    {
        LightFaceData lightFaceData = lightShadowData.lightFaceData[face];
        print("frequency = ", lightFaceData.frequency);
        print("ranking = ", lightFaceData.currRanking);
        print("viewDirection = ", lightFaceData.viewDirection);
        print("dot with +x = ", dot(lightShadowData.lightFaceData[0].viewDirection, lightFaceData.viewDirection));

        float4x4 viewMat = lightFaceData.viewMat;
        float3 checkVector = mul(float4(lightPosW, 1.0f), viewMat).xyz;
        print("checkVector = ", checkVector);
    }

    // Early out if there is no a "significant" important face for this light
    float importance = length(lightAvgDir);
    print("importance = ", importance);
    if (importance < 0.001f) return;

    // Get the closest face of this average direction.
    lightAvgDir = normalize(lightAvgDir);
    print("lightAvgDir = ", lightAvgDir);
    uint closestFaceIdx = getClosestFace(lightAvgDir, lightShadowData);
    float3 closestFaceDir = normalize(lightShadowData.lightFaceData[closestFaceIdx].viewDirection);

    print("closestFaceIdx = ", closestFaceIdx);
    print("cos(M_PI / 4.0f - 0.1f) = ", cos(M_PI / 4.0f - 0.1f));

    // No need to rotate if the angle is too small or angle is close to 45 degree
    if (dot(closestFaceDir, lightAvgDir) >= cos(0.1f)) return;
    // if (dot(closestFaceDir, lightAvgDir) <= cos(M_PI / 4.0f - 0.01f)) return;

    // Compute the rotation between closest face direction and average direction
    // Song: do we need to clamp this rotation if rotation angle is too large (cosine between them is small)?
    float4x4 rotationFromPrev = calcRotationFromTwoVectors(closestFaceDir, lightAvgDir);

    print("closestFaceDir before = ", closestFaceDir);
    print("closestFaceDir after = ", normalize(mul(float4(closestFaceDir, 0.0f), rotationFromPrev).xyz));

    // float3 testV1 = normalize(float3(1, 1, 0));
    // float3 testV2 = normalize(float3(0, 1, 1));
    // float4x4 rot = calcRotationFromTwoVectors(testV1, testV2);
    // float3 compV2 = normalize(mul(float4(testV1, 0.0f), rot).xyz);
    // print("testV2 = ", testV2);
    // print("compV2 = ", compV2);


    // Also compute another rotation from the face base direction. This is later used for rotate the shading point
    // float3 closestFaceBaseDir = float3(0.0f);
    // closestFaceBaseDir[closestFaceIdx / 2u] = pow(-1, closestFaceIdx % 2u);
    // float4x4 rotationFromBase = calcRotationFromTwoVectors(closestFaceBaseDir, weightedAvgDir);

    // Update face direction and view matrix
    //  -> Song: Do we need to recompute the view matrix in light world space since rotation and view matrix of each face is
    //     computed in different coordinate basis?
    for (uint face = 0; face < lightShadowData.numShadowMaps; face++)
    {
        LightFaceData lightFaceData = lightShadowData.lightFaceData[face];
        print("", face);

        print("viewDirection before= ", gLightShadowDataBuffer[lightID].lightFaceData[face].viewDirection);
        float3 newViewDirection = normalize(mul(float4(lightFaceData.viewDirection, 0.0f), rotationFromPrev).xyz);
        gLightShadowDataBuffer[lightID].lightFaceData[face].viewDirection = newViewDirection;
        print("viewDirection after = ", newViewDirection);


        // Decompose view matrix into rotation and translation matrix and only update the view rotation part
        float4x4 prevViewMat = lightFaceData.viewMat;
        float4x4 viewTranslation = translation(-lightPosW.x, -lightPosW.y, -lightPosW.z);
        float4x4 newViewMat = float4x4(prevViewMat[0], prevViewMat[1], prevViewMat[2], float4(0.0, 0.0, 0.0, 1.0));
        // printMatrix4x4(newViewMat, "newViewMat = ");
        newViewMat = mul(newViewMat, transpose(rotationFromPrev));
        newViewMat = mul(viewTranslation, newViewMat);
        float4x4 newViewMat3 = lookAtRH(lightShadowData.centerPosW, lightShadowData.centerPosW + newViewDirection, faceUpVectors[face]);

        gLightShadowDataBuffer[lightID].lightFaceData[face].viewMat = newViewMat3;


        if (face < 3)
        {
            // printMatrix4x4(prevViewMat, "prevViewMat = ");
            // printMatrix4x4(viewTranslation, "viewTranslation = ");
            // printMatrix4x4(newViewMat, "newViewMat = ");
            // printMatrix4x4(newViewMat2, "newViewMat2 = ");
            printMatrix4x4(newViewMat3, "newViewMat3 = ");
        }
    }


}
