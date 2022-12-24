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

// Reference: Adaptive Depth Bias for Shadow Maps, Dou 2014
bool adaptiveDepthBias(float3 normalW, float3 wsFragPos, uint shadowMapSize, uint face, uint mipLevel, float constEpsilon, const Texture2DArray<float> shadowMaps, SamplerState sampler,
    LightShadowMapData lightData)
{
    // Note: Actually the light rays can be precomputed.
    // The light ray can therefore be obtained by looking up shadow map
    // texel center, followed by a texture lookup.

    /** Define tangent plane **/
    // Light space frag normal
    // float4 normalLightView = mul(float4(normalW, 0.0f), inverse(transpose(lightParams.lightView)));
    float4 normalLightView = mul(float4(normalW, 0.0f), lightData.viewMats[face]); // Because lightView is orthgonal matrix if we don't account for translation
    float3 n = normalize(normalLightView.xyz);

    /** Locate shadow map texel center **/
    // Transform from eye-space to shadow map texture coordinates
    float4 lsFragPos = mul(float4(wsFragPos, 1.0f), lightData.viewMats[face]);
    float4 projCoord = mul(lsFragPos, lightData.persProjMat);
    projCoord /= projCoord.w;
    projCoord.y *= -1; // DirectX flips the Y axis in viewport scale matrix
    projCoord = float4(0.5 * projCoord.xy, projCoord.z, 0.0f) + float4(0.5, 0.5, 0.0, 0.0);

    // Locate corresponding light space shadow map grid center
    uint2 index = (uint2)floor(float2(projCoord.xy * shadowMapSize));
    float texelSize = 1.0 / shadowMapSize;
    // Normalized coordinate in [0,1]
    float2 nlsGridCenter = texelSize * (index + float2(0.5)); // Normalized eye space grid center --- [0,1]
    float viewBound = 0.5f * lightData.lightFrustumSize; // TODO: remove *0.5f ???
    // Unnormalized coordinate in [-lightLeft,lightLeft]
    float2 lsGridCenter = viewBound * (2.0 * nlsGridCenter - float2(1.0));

    print("lightData.lightFrustumSize = ", lightData.lightFrustumSize);
    print("index = ", index);
    print("nlsGridCenter = ", nlsGridCenter);
    print("lsGridCenter = ", lsGridCenter);

    /** Define light ray **/
    // Light ray direction in light space
    float nearPlane = lightData.nearFarPlane.x;
    float farPlane = lightData.nearFarPlane.y;
    float3 lsGridLineDir = normalize(float3(lsGridCenter, -nearPlane)); // Light space grid line direction

    /** Plane ray intersection **/
    // Locate the potential occluder for the shading fragment
    float ls_t_hit = dot(n, lsFragPos.xyz) / dot(n, lsGridLineDir);
    float3 ls_hit_p = ls_t_hit * lsGridLineDir;

    print("ls_t_hit = ", ls_t_hit);
    print("ls_hit_p = ", ls_hit_p);

    /** Compute Adaptive Epsilon **/
    // Normalized depth value in shadow map
    float SMDepth = shadowMaps.SampleLevel(sampler, float3(projCoord.xy, face), mipLevel);
    // A and B are computed bnased on light near and far planes.
    // They can be retrieved directly from light projection matrix
    float A = lightData.persProjMat[2][2];
    float B = lightData.persProjMat[3][2];

    // Change projection matrix calculation from DirectX to OpenGL
    A += nearPlane / (nearPlane - farPlane);
    B *= 2;

    // Constant part is (sceneScale * K).
    //  -> this formula is equal to Eq.3 in the paper
    float adaptiveEpsilon = 0.5 * pow(1.0 - A - 2.0 * SMDepth, 2) * constEpsilon / B; // this will be a negative value

    // Use the intersection point as new look up point
    float4 lsPotentialoccluder = mul(float4(ls_hit_p, 1.0), lightData.persProjMat);
    lsPotentialoccluder = lsPotentialoccluder / lsPotentialoccluder.w; // DirectX depth is alreay in [0, 1]

    // This min operation already includes subtracting the optimal depth bias in the paper.
    // It is equal to: shading point depth - abs(shading point depth - shadow map depth)
    float actualDepth = min(lsPotentialoccluder.z, projCoord.z);
    bool isVisible = SMDepth < actualDepth + adaptiveEpsilon ? false : true;

    // isVisible = abs(dot(n, normalize(-lsFragPos.xyz))) < 0.1f ? false : isVisible;
    // isVisible = dot(normalLightView.xyz, lsFragPos.xyz) > 0.0001f ? false : isVisible;
    // isVisible = any(clamp(projCoord.xyz, 0.0, 1.0) != projCoord.xyz) ? false : isVisible;

    print("face = ", face);
    print("mipLevel = ", mipLevel);
    print("shadowMapSize = ", shadowMapSize);
    print("SMDepth = ", SMDepth);
    print("projCoord = ", projCoord);
    print("lsPotentialoccluder = ", lsPotentialoccluder);
    print("adaptiveEpsilon = ", adaptiveEpsilon);
    print("constEpsilon = ", constEpsilon);
    print("actualDepth = ", actualDepth);
    print("cos = ", dot(n, normalize(-lsFragPos.xyz)));

    return isVisible;
}

bool evaluateShadowMaps(float3 posW, float3 normalW, LightShadowMapData lightData, const Texture2DArray<float> shadowMaps, SamplerState linearSampler, float bias,
    uint gDepthBiasMode, float constEpsilon, uint visMode = 1, uint offset = 0)
{
    // Get the max resolution of the texture
    uint2 maxShadowMapSize;
    uint elements;
    shadowMaps.GetDimensions(maxShadowMapSize.x, maxShadowMapSize.y, elements);
    uint mipLevel = (uint)log2(maxShadowMapSize.x / lightData.shadowMapSize);

    // Check the shading is in which face
    float3 lightPosW = lightData.centerPosW;
    float3 lightToShadingW = posW - lightPosW;
    uint face = getFaceIndex(normalize(lightToShadingW));

    // Compute depth bias
    float3 L = normalize(lightPosW - posW);
    float NdotL = dot(normalW, L);
    bool isVisible = true;
    if (gDepthBiasMode == (uint)ShadowDepthBias::SlopeScale)
    {
        float depthBias = 0.0f;
        if (mipLevel <= 1)
            depthBias = max(2.0 * bias * (1.0 - NdotL), bias);
        else if (mipLevel <= 3)
            depthBias = max(2.0 * bias * (1.0 - NdotL) * pow(2.0, mipLevel), bias);
        else
            depthBias = max(2.0 * bias * (1.0 - NdotL) * pow(2.1, mipLevel + 1), bias);

        // Project shading point to the selected light space
        float4x4 lightViewProjMat = lightData.viewProjMats[face];
        float4 posLightClipping = mul(float4(posW, 1.0f), lightViewProjMat);
        float3 posLightNDC = posLightClipping.xyz / posLightClipping.w;
        float3 projCoord = float3(posLightNDC.xy * float2(0.5f, -0.5f) + 0.5f, posLightNDC.z);

        // Sample the target face's shadow map
        float shadowMapDepth = shadowMaps.SampleLevel(linearSampler, float3(projCoord.xy, face), mipLevel);

        isVisible = shadowMapDepth < projCoord.z - depthBias ? false : true;
    }
    else if (gDepthBiasMode == (uint)ShadowDepthBias::Dou2014)
    {
        isVisible = adaptiveDepthBias(normalW, posW, lightData.shadowMapSize, face, mipLevel, constEpsilon, shadowMaps, linearSampler, lightData);
    }


    return isVisible;

}

bool evaluateISM(float3 posW, float3 normalW, LightShadowMapData lightData, const Texture2DArray<float> ismTextureArray, SamplerState linearSampler, float bias)
{
    float2 lightNearFar = lightData.nearFarPlane;

    // Check the shading is in which hemisphere
    float3 lightPosW = lightData.centerPosW;
    int direction = sign(lightPosW.y - posW.y);
    uint ismOffset = direction == 1 ? 0 : 1;

    // Compute depth bias. TODO: also adaptively based on pull & push levels
    float3 L = normalize(lightPosW - posW);
    float NdotL = dot(normalW, L);
    float depthBias = max(4.0 * bias * (1.0 - NdotL), bias);

    // Paraboloid project shading point
    float4x4 viewMat = lightData.viewMats[3 - ismOffset];
    float3 shadingPosLightView = mul(float4(posW, 1.0f), viewMat).xyz;
    float3 shadingPosLightNdc = paraboloidProjection(shadingPosLightView, lightNearFar.x, lightNearFar.y);
    float3 projCoord = float3(shadingPosLightNdc.xy * float2(0.5f, -0.5f) + 0.5f, shadingPosLightNdc.z - depthBias);

    print("ismOffset = ", ismOffset);
    print("projCoord = ", projCoord);

    // Sample the target face's shadow map
    float shadowMapDepth = ismTextureArray.SampleLevel(linearSampler, float3(projCoord.xy, lightData.ismArrayStartIndex + ismOffset), 0);

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


