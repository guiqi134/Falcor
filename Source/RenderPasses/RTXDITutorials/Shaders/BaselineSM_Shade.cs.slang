#include "RtxdiBridge.slangh"

import Scene.Shading;
import Scene.HitInfo;
import Rendering.Materials.Microfacet;
import Rendering.Materials.Fresnel;
import Rendering.Lights.EmissiveLightSampler;
import Rendering.Lights.EmissivePowerSampler;
import Rendering.Lights.EmissiveUniformSampler;
// import Utils.Sampling.SampleGenerator;

import EvaluateShadowMaps;

cbuffer BaselineCB
{
    uint gHeroLightsCount;
    uint gTotalLightMeshCount;
    EmissiveLightSampler gEmissiveSampler;
}

Texture2DArray<float> gHeroLightShadowMaps;
Texture2D<float4> gInputEmission; // Emissive color from the G-buffer
Texture2D<PackedHitInfo> gVbuffer;

RWTexture2D<float4> gOutputColor;

// Return a triangle light index and 1/pdf of sampling that light (for SAMPLE_UNIFORM)
uint SampleTrianglesUniformly(inout RAB_RandomSamplerState rng, int lightMeshID, out float invSourcePdf)
{
    // Randomly pick a triangle out of the local list with uniform probability.
    float uTriangle = RAB_GetNextRandom(rng);
    uint triangleIndex = 0;

    // Get the target light total triangle count and start offset
    if (lightMeshID != -1)
    {
        uint lightTriangleCount = gScene.lightCollection.meshData[lightMeshID].triangleCount;
        uint triangleIndexOffset = gScene.lightCollection.meshData[lightMeshID].triangleOffset;

        invSourcePdf = (float)lightTriangleCount;

        // Safety precaution as the result of the multiplication may be rounded to triangleCount even if uLight < 1.0 when triangleCount is large.
        triangleIndex = triangleIndexOffset + min((uint)(uTriangle * lightTriangleCount), lightTriangleCount - 1);
    }
    else
    {
        uint totalTriangleCount = gScene.lightCollection.triangleCount;
        invSourcePdf = (float)totalTriangleCount;
        triangleIndex = min((uint)(uTriangle * totalTriangleCount), totalTriangleCount - 1);
    }

    return triangleIndex;
}

float3 unShadowedShading(const RAB_LightInfo triLightInfo, const RAB_LightSample triLightSample, const float3 triLightEmission, const RAB_Surface surface)
{
    float3 shadingPosW = RAB_GetSurfaceWorldPos(surface);
    float3 shadingN = RAB_GetSurfaceNormal(surface);

    // Compute our geometry term
    float geomTerm = triLightSample.getJacobianAtoW();

    // Compute the BRDF terms for the target function.
    float D = surface.getNdf(triLightSample.NdotH());
    float G = surface.getMaskingTerm(triLightSample.NdotL(), surface.NdotV());
    float3 F = any(surface.getSpecularColor() <= 0.0f) ? float3(0.0f) :
            evalFresnelSchlick(surface.getSpecularColor(), float3(1), triLightSample.LdotH());

    // Compute the specular and diffuse lobes of the BRDF separately.
    float3 diffuseLobe = triLightSample.NdotL() * (surface.getDiffuseColor() * M_1_PI); // NdotL * albedo
    float3 specularLobe = F * D * G / (4.0f * surface.NdotV());                      // Standard microfacet model

    print("geomTerm = ", geomTerm);

    // Return our unshadowed color result, separate in diffuse and specular channels:
    //   -> [ L * BRDF * <geometry-term> ]
    float3 difColor = max(float3(0.0f), triLightEmission * diffuseLobe * geomTerm);
    float3 specColor = max(float3(0.0f), triLightEmission * specularLobe * geomTerm);

    return difColor + specColor;
}

[numthreads(16, 16, 1)]
void main(uint3 threadId : SV_DispatchThreadID)
{
    printSetPixel(threadId.xy);

    // Get the pixel location on screen
    const uint2 pixelPosition = threadId.xy;
    RAB_RandomSamplerState rng = RAB_InitRandomSampler(pixelPosition, 1);

    // Get the G-buffer for current pixel
    RAB_Surface surface = RAB_GetGBufferSurface((int2)pixelPosition, false);
    float3 shadingPosW = RAB_GetSurfaceWorldPos(surface);
    float3 shadingN = RAB_GetSurfaceNormal(surface);

    // Each pixel needs to evalute the shadowed path from N hero lights
    float3 outColor = 0.0f;
    for (uint i = 0; i < gHeroLightsCount; i++)
    {
        float3 difColor = 0.0f, specColor = 0.0f;
        uint lightMeshIndex = gSortedLightsBuffer[i].x;
        LightShadowMapData lightData = gLightShadowDataBuffer[lightMeshIndex];

        // Generate random sample from the lights
        // TODO: if we want to use pdf texture to sample, then one texture for each hero light should be created
        float invPdf = 0.0f;
        uint rndTriangleLight = SampleTrianglesUniformly(rng, (int)lightMeshIndex, invPdf); // sample in one lights

        // Unshadowed contribution use light sample to evaluate
        float2 uv = sampleNext2D(rng);
        RAB_LightInfo triLightInfo = RAB_LoadLightInfo(rndTriangleLight, false);
        RAB_LightSample triLightSample = RAB_SamplePolymorphicLight(triLightInfo, surface, uv);

        // We uniformly sampled our triangle UVs, so update our sampling PDF by 1/<triangle-area> to account for this.
        // Compare to the returned pdf from Falcor's sampleLight() method, the pdf here is for area and Falcor's is
        // for solid angle, so we need to add the Jacobian term (<geometry-term>) in Shade() function.
        invPdf = invPdf * triLightSample.getInverseSampleArea();
        triLightInfo.getLightArea();

        // Visibility use light center to evaluate
        float isVisible = 1.0f;
        isVisible = (float)evaluateShadowMaps(shadingPosW, shadingN, lightData, gHeroLightShadowMaps, gLinearSampler, gDepthBias, 2, 6 * i);

        outColor += isVisible * unShadowedShading(triLightInfo, triLightSample, triLightInfo.getSampleEmission(uv), surface) * invPdf;
    }

    // Also each pixel needs to add one random unshadowed path from other light?
    uint sampleCount = 0;
    float3 unshadowedPath = 0.0f;
    for (uint i = 0; i < sampleCount; i++)
    {
        float invPdf = 0.0f;
        uint rndTriangleLight = SampleTrianglesUniformly(rng, -1, invPdf); // sample in all lights

        // Reject samples that is in the hero lights. TODO: precompute other lights sampling list
        bool rejectSample = false;
        for (uint j = 0; j < gHeroLightsCount; j++)
        {
            rejectSample = gSortedLightsBuffer[j].x == gScene.lightCollection.getLightIdx(rndTriangleLight) ? true : false;
        }
        if (rejectSample) continue;

        // Get a candidate sample from some random UV on the selected emissive light
        float2 uv = sampleNext2D(rng);
        RAB_LightInfo triLightInfo = RAB_LoadLightInfo(rndTriangleLight, false);
        RAB_LightSample triLightSample = RAB_SamplePolymorphicLight(triLightInfo, surface, uv);

        // We uniformly sampled our triangle UVs, so update our sampling PDF by 1/<triangle-area> to account for this.
        // Compare to the returned pdf from Falcor's sampleLight() method, the pdf here is for area and Falcor's is
        // for solid angle, so we need to add the Jacobian term (<geometry-term>) in Shade() function.
        invPdf = invPdf * triLightInfo.getLightArea();

        // Shade the sample
        float3 unshadowedColor = unShadowedShading(triLightInfo, triLightSample, triLightInfo.getSampleEmission(uv), surface);

        // Doing Monte Carlo integration, so accumulate 1/N * sum[ f(x_i) / p(x_i) ]
        // outColor += unshadowedColor * invPdf * (1.0f / (float)sampleCount);
    }

    outColor += gInputEmission[pixelPosition].rgb;
    gOutputColor[pixelPosition] = float4(outColor.r, outColor.g, outColor.b, 1.0f);
}

// It is only under the assumption that (1) most surface is diffuse (2) mesh light is constant not textured, then
// we can use the light mesh center point to also approximate the unshadowed contribution
// {
//     float3 triLightEmission = gScene.materials.evalEmissive(gScene.lightCollection.meshData[lightMeshIndex].materialID, float2(0));
//     float3 lightCenter = gLightShadowDataBuffer[lightMeshIndex].centerPosW;
//     float invPdf = 1.0f; // we pick the light deterministically, the pdf is equal to 1.0
// }
