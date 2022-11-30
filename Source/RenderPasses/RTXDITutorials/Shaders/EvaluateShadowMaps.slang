#include "../HostDeviceSharedDefinitions.h"

import Scene.Scene;
import Utils.Debug.PixelDebug;
import ISMHelpers;

// Reference: https://github.com/TheRealMJP/BakingLab/blob/1a043117506ac5b5bcade5c86d808485f3c70b12/SampleFramework11/v1.02/Graphics/Textures.h#L124
uint getFaceIndex(float3 direction)
{
    // Assert_(texData.NumSlices == 6);

    float maxComponent = max(max(abs(direction.x), abs(direction.y)), abs(direction.z));
    uint faceIdx = 0;
    float2 uv = float2(direction.y, direction.z);

    if (direction.x == maxComponent)
    {
        faceIdx = 0;
        uv = float2(-direction.z, -direction.y) / direction.x;
    }
    else if (-direction.x == maxComponent)
    {
        faceIdx = 1;
        uv = float2(direction.z, -direction.y) / -direction.x;
    }
    else if (direction.y == maxComponent)
    {
        faceIdx = 2;
        uv = float2(direction.x, direction.z) / direction.y;
    }
    else if (-direction.y == maxComponent)
    {
        faceIdx = 3;
        uv = float2(direction.x, -direction.z) / -direction.y;
    }
    else if (direction.z == maxComponent)
    {
        faceIdx = 4;
        uv = float2(direction.x, -direction.y) / direction.z;
    }
    else if (-direction.z == maxComponent)
    {
        faceIdx = 5;
        uv = float2(-direction.x, -direction.y) / -direction.z;
    }

    uv = uv * float2(0.5f, 0.5f) + float2(0.5f, 0.5f);
    return faceIdx;
}

bool evaluateShadowMaps(float3 posW, float3 normalW, LightShadowMapData lightData, const Texture2DArray<float> shadowMaps, SamplerState linearSampler, float bias,
    uint visMode = 1, uint offset = 0)
{
    // Check the shading is in which face
    float3 lightPosW = lightData.centerPosW;
    float3 lightToShadingW = posW - lightPosW;
    uint face = getFaceIndex(normalize(lightToShadingW));

    // Compute depth bias
    float3 L = normalize(lightPosW - posW);
    float NdotL = dot(normalW, L);
    float depthBias = bias * saturate(1.0 - NdotL) + bias;

    // Project shading point to the selected light space
    float4x4 lightViewProjMat = lightData.viewProjMat[face];
    float4 posLightClipping = mul(float4(posW, 1.0f), lightViewProjMat);
    float3 posLightNDC = posLightClipping.xyz / posLightClipping.w;
    float3 projCoord = float3(posLightNDC.xy * float2(0.5f, -0.5f) + 0.5f, posLightNDC.z - depthBias);

    // Sample the target face's shadow map
    uint texArrayIndex = visMode == 1 ? lightData.startIndex + face : offset + face;
    float shadowMapDepth = shadowMaps.SampleLevel(linearSampler, float3(projCoord.xy, texArrayIndex), 0);

    return shadowMapDepth < projCoord.z ? false : true;
}

bool evaluateISM(float3 posW, float3 normalW, LightShadowMapData lightData, const Texture2DArray<float> ismTextureArray, SamplerState linearSampler, float bias)
{
    float2 lightNearFar = lightData.nearFarPlane;

    // Check the shading is in which hemisphere
    float3 lightPosW = lightData.centerPosW;
    int direction = sign(lightPosW.y - posW.y);
    uint ismOffset = direction == 1 ? 0 : 1;

    // Compute depth bias
    float3 L = normalize(lightPosW - posW);
    float NdotL = dot(normalW, L);
    float depthBias = bias * saturate(1.0 - NdotL) + bias;

    // Paraboloid project shading point
    float4x4 viewMat = lightData.viewMats[ismOffset];
    float3 shadingPosLightView = mul(float4(posW, 1.0f), viewMat).xyz;
    float3 shadingPosLightNdc = paraboloidProjection(shadingPosLightView, lightNearFar.x, lightNearFar.y);
    float3 projCoord = float3(shadingPosLightNdc.xy * float2(0.5f, -0.5f) + 0.5f, shadingPosLightNdc.z - depthBias);

    print("ismOffset = ", ismOffset);
    print("projCoord = ", projCoord);

    // Sample the target face's shadow map
    float shadowMapDepth = ismTextureArray.SampleLevel(linearSampler, float3(projCoord.xy, lightData.startIndex + ismOffset), 0);

    return shadowMapDepth < projCoord.z ? false : true;
}


// Help functions for our method

// Given the reservoir's (triangle) light index, map it to mesh light or point light index
int toLocalLightIndex(uint reservoirLightIdx, uint totalLigthMeshCount, uint firstPointLight, uint environmentLightIndex)
{
    int lightID = -1;
    if (reservoirLightIdx < firstPointLight)
    {
        // Get the light mesh ID based on triangle index
        uint triIndex = reservoirLightIdx;
        lightID = (int)gScene.lightCollection.getLightIdx(triIndex);
    }
    else if (reservoirLightIdx < environmentLightIndex)
    {
        uint pointLightIndex = reservoirLightIdx - firstPointLight;
        lightID = int(totalLigthMeshCount + pointLightIndex);
    }

    return lightID;
}


