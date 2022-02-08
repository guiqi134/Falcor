#include "Utils/Math/MathConstants.slangh"

import Scene.Raster;
import Scene.Shading;
import Scene.Material.MaterialData;
import Experimental.Scene.Material.StandardMaterial;
import Experimental.Scene.Lights.LightHelpers;
import Utils.Sampling.SampleGenerator;
import Utils.Debug.PixelDebug;

import Helpers;

cbuffer PerFrameCB
{
    float4x4 gLightSpaceMat;
    float4x4 gLightView;
    float3 gLightPos;
    uint2 gShadowMapDim;
    bool gEnableShadow;
    float nearZ;
    float farZ;
    uint gFrameIndex;
}

Texture2D<float4> gShadowMap;
SamplerComparisonState gSamplerCmp;
SamplerState gSampler;

float LinearizeDepth(float depth)
{
    return (nearZ * farZ) / (farZ + depth * (nearZ - farZ));
}

float shadowMapPCF(float3 projCoords, float2 filterRadiusUV, float2 poissonOffset, float2 rotation)
{
    float sum = 0.0f;
    float2 t = frac(projCoords.xy * gShadowMapDim + 0.5f); // Interpolation percentage in x, y

    for (int i = 0; i < PCF_SAMPLES; i++)
    {
        // Use hardware PCF
        float2 offset = (POISSON_SAMPLES_64[i] + poissonOffset) * filterRadiusUV;

        float2 selectedSample = POISSON_SAMPLES_64[i];
        selectedSample = float2(
            selectedSample.x * rotation.x - selectedSample.y * rotation.y,
            selectedSample.x * rotation.y + selectedSample.y * rotation.x
        );
        offset = selectedSample * filterRadiusUV;

        // Gathers the four texels: (-,+),(+,+),(+,-),(-,-)
        //float4 tmp = gShadowMap.Gather(gSampler, projCoords.xy + offset);
        //tmp.x = tmp.x < projCoords.z ? 0.0f : 1.0f;
        //tmp.y = tmp.y < projCoords.z ? 0.0f : 1.0f;
        //tmp.z = tmp.z < projCoords.z ? 0.0f : 1.0f;
        //tmp.w = tmp.w < projCoords.z ? 0.0f : 1.0f;
        //sum += lerp(lerp(tmp.w, tmp.z, t.x), lerp(tmp.x, tmp.y, t.x), t.y); // bilinear filter comparison results

        sum += gShadowMap.SampleCmpLevelZero(gSamplerCmp, projCoords.xy + offset, projCoords.z); // projCoords.z < sample value ? for all nearest texel

        /*float shadowMapDepth = gShadowMap.SampleLevel(gSampler, projCoords.xy + offset, 0).r;
        sum += shadowMapDepth < projCoords.z ? 0.0f : 1.0f;*/
    }

    print("shadow factor = ", sum / PCF_SAMPLES);
    return saturate(sum / PCF_SAMPLES);
}

void findBlocker(out float avgBlockerDepth, out float numBlockers, float3 projCoords, float zReceiver, float2 poissonOffset, float2 rotation)
{
    // This uses similar triangles to compute what area of the shadow map we should search (in light view space)
    float2 searchSize = float2(LIGHT_WORLD_SIZE * (zReceiver - NEAR_PLANE) / zReceiver);
    // Measure the searchSize according to the frustum size and clamp search size into [0, 1]
    searchSize /= LIGHT_FRUSTUM_SIZE;

    print("zReceiver = ", zReceiver);
    print("searchSize = ", searchSize);

    float blockerSum = 0;
    numBlockers = 0;
    for (int i = 0; i < BLOCKER_SEARCH_SAMPLES; i++)
    {
        float2 offset = (POISSON_SAMPLES_32[i] + poissonOffset) * searchSize;

        float2 selectedSample = POISSON_SAMPLES_64[i];
        selectedSample = float2(
            selectedSample.x * rotation.x - selectedSample.y * rotation.y,
            selectedSample.x * rotation.y + selectedSample.y * rotation.x
        );
        offset = selectedSample * searchSize;

        float shadowMapDepth = gShadowMap.SampleLevel(gSampler, projCoords.xy + offset, 0).r; 
        if (shadowMapDepth < projCoords.z)
        {
            blockerSum += abs(LinearizeDepth(shadowMapDepth));
            numBlockers++;
        }
    }

    avgBlockerDepth = blockerSum / numBlockers;
    print("avgBlockerDepth = ", avgBlockerDepth);
    print("numBlockers = ", numBlockers);
}

// Reference: https://developer.download.nvidia.com/whitepapers/2008/PCSS_Integration.pdf
// All calculation should be performed in light view space!
float shadowMapPCSS(float3 projCoords, float zView, float2 poissonOffset, float2 rotation)
{
    // 1. blocker search
    float avgBlockerDepth = 0;
    float numBlockers = 0;
    findBlocker(avgBlockerDepth, numBlockers, projCoords, zView, poissonOffset, rotation);
    print("projCoords = ", projCoords);

    // no occluders -> no shadow
    if (numBlockers < 1) return 1.0f;

    // 2. penumbra size
    float penumbraSize = LIGHT_WORLD_SIZE * (zView - avgBlockerDepth) / avgBlockerDepth; // depth in light view space

    // Mapping penumbra size to the shadow map (divide by frustum size) using similar triangle to determine the filter size
    // The similar triangle takes sample point in area light as upper vertex and penumbra size as bottom edge
    float2 filterRadiusUV = float2(penumbraSize * NEAR_PLANE / zView);
    filterRadiusUV /= LIGHT_FRUSTUM_SIZE;

    print("penumbraSize = ", penumbraSize);
    print("LIGHT_FRUSTUM_SIZE = ", LIGHT_FRUSTUM_SIZE);
    print("filterRadiusUV = ", filterRadiusUV);

    // 3. PCF filtering
    // filterRadiuUV = 0 -> hard shadow
    return shadowMapPCF(projCoords, filterRadiusUV, poissonOffset, rotation);
}

float4 psMain(VSOut vsOut, uint triangleIndex : SV_PrimitiveID) : SV_TARGET0
{
    float2 pixelPos = vsOut.posH.xy;
    printSetPixel(pixelPos);

    float4 color = float4(0, 0, 0, 1);
    float visibility = 1.0f;
    SampleGenerator sg = SampleGenerator.create(pixelPos, gFrameIndex);
    SampleGenerator sg1 = SampleGenerator.create(pixelPos, 0);

    // TODO: use random number from CPU and store them in a 3D texture. Use pixel world pos to index it.
    float2 poissonOffset = sampleNext2D(sg1) - 0.5f;
    float angle = sampleNext1D(sg1) * M_2PI;
    float2 rotation = float2(cos(angle), sin(angle));

    // Perform alpha test first
    float3 faceNormal = gScene.getFaceNormalW(vsOut.instanceID, triangleIndex);
    VertexData v = prepareVertexData(vsOut, faceNormal);
    if (alphaTest(v, gScene.materials[vsOut.materialID], gScene.materialResources[vsOut.materialID], 0.0f))
        discard;

    // Prepare all shading data
    float3 viewDir = normalize(gScene.camera.getPosition() - vsOut.posW);
    ShadingData sd = prepareShadingData(vsOut, triangleIndex, viewDir);
    StandardMaterial mtl;

    // Is emissive material
    if (dot(sd.emissive, sd.emissive) != 0.0f) return float4(sd.emissive, 1.0f);

    // numerical integration to sample the sphere light to get incident radiance
    float3 sum = 0.0f;
    LightData light = gScene.getLight(0);
    for (uint i = 0; i < _LIGHT_SAMPLES; i++)
    {
        AnalyticLightSample ls;
        sampleLight(sd.posW, light, sg, ls);
        if (dot(sd.N, ls.dir) < kMinCosTheta) continue;

        sum += mtl.eval(sd, ls.dir) * ls.Li;
    }
    color.rgb = sum / _LIGHT_SAMPLES;

    // Get visibility from shadow map using PCSS
    float3 toLight = gLightPos - sd.posW;
    float distSqr = max(dot(toLight, toLight), kMinLightDistSqr);
    float d = sqrt(distSqr);
    float3 L = toLight / d;
    if (gEnableShadow)
    {
        // Self-shadowing bias
        float bias = _DEPTH_BIAS < 0.0f ? max(0.01 * (1.0 - dot(sd.N, L)), 0.001) : _DEPTH_BIAS; // TODO: better bias selecting algorithm

        /*
        1. Since DX uses upper-left corner as the origin in screen space, we need to reverse y-axis
        2. In D3D the normalized device coordinates(x, y, z) form a 3D cube like so :
            x: -1 (left of screen) to 1 (right)
            y : -1 (bottom of screen) to 1 (top)
            z : 0 (near)to 1 (far)
        */
        float4 lightSpacePosH = mul(float4(vsOut.posW, 1.0f), gLightSpaceMat);
        float3 lightSpacePos = lightSpacePosH.xyz / lightSpacePosH.w;
        lightSpacePos.y *= -1;
        float3 projCoords = float3(lightSpacePos.xy * 0.5f + 0.5f, lightSpacePos.z - bias); // screen space

        // Depth is negative in -z direction, for length we need to use absolute value
        float zView = abs(mul(float4(vsOut.posW, 1.0f), gLightView).z);
        visibility = shadowMapPCSS(projCoords, zView, poissonOffset, rotation);
    }


    //color.rgb = mtl.eval(sd, L) * light.intensity / distSqr * visibility;
    color.rgb *= visibility;
    color.rgb += sd.emissive;
    color.a = sd.opacity;

    return color;
}
