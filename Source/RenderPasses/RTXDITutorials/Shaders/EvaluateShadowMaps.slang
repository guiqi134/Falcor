import Utils.Debug.PixelDebug;

#include "../HostDeviceSharedDefinitions.h"

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

bool evaluateShadowMaps(float3 posW, float3 normalW, LightMeshData lightData, const Texture2DArray<float> shadowMaps, SamplerState linearSampler, float bias,
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