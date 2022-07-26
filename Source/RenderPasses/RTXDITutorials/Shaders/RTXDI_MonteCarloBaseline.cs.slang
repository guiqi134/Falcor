/***************************************************************************
 # Copyright (c) 2015-21, NVIDIA CORPORATION. All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions
 # are met:
 #  * Redistributions of source code must retain the above copyright
 #    notice, this list of conditions and the following disclaimer.
 #  * Redistributions in binary form must reproduce the above copyright
 #    notice, this list of conditions and the following disclaimer in the
 #    documentation and/or other materials provided with the distribution.
 #  * Neither the name of NVIDIA CORPORATION nor the names of its
 #    contributors may be used to endorse or promote products derived
 #    from this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS "AS IS" AND ANY
 # EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 # IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 # PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 # CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 # PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 # PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 # OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 # OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **************************************************************************/

/* This shader gives a baseline Monte Carlo integration, without doing any reservoir resampling.

   The reason it includes the RTXDI bridge is to ensure that light sampling and shading are
   identical between the comparisons, and to give you some insight into the RTXDI bridge 
*/

#include "RtxdiBridge.slangh"
#include "BaselineSampling.slangh"

import Utils.Geometry.GeometryHelpers;
import SSRT;

cbuffer CB
{
    uint gLightSamplingMode;  // Which sampling scheme should we use to pick our initial candidates?
    uint gNumTriSamples;      // How many samples should we take on emissive triangles?
    uint gNumEnvSamples;      // How many samples should we take on the environment map?
}

Texture2D<float4> gInputEmission;   // Emissive color from the G-buffer
RWTexture2D<float4> gOutputColor;   // Full output color (diffuse, specular, emissive, background)


// Shade a specific sample on a light at a given surface
float3 Shade(RAB_Surface surface, RAB_LightSample lightSample, float3 lightEmission)
{
    // Shoot a shadow ray to the selected light sample
    float3 rayOrigin = RAB_GetSurfaceWorldPos(surface);
    float4 rayDirLen = lightSample.getDirectionAndDistance();
    float isVisible = 1.0f;
    if (gVisibilityMode == 0)
        isVisible = traceShadowRay(rayOrigin, rayDirLen);
    else
        isVisible = ssrtUniformStepBinarySearch(rayOrigin, rayDirLen.xyz, rayDirLen.w);

    // Compute the BRDF terms for the target function.
    float  D = surface.getNdf(lightSample.NdotH());
    float  G = surface.getMaskingTerm(lightSample.NdotL(), surface.NdotV());
    float3 F = any(surface.getSpecularColor() <= 0.0f) ? float3(0.0f) :
        evalFresnelSchlick(surface.getSpecularColor(), float3(1), lightSample.LdotH());

    // Compute the specular and diffuse lobes of the BRDF separately.
    float3 diffuseLobe = lightSample.NdotL() * (surface.getDiffuseColor() * M_1_PI);  // NdotL * albedo 
    float3 specularLobe = F * D * G / (4.0f * surface.NdotV());                       // Standard microfacet model, with NdotLs cancelled

    // Return our shading result, <shadow-ray> * L * <geometry-term> * BRDF
    return isVisible * max(float3(0.0f), lightEmission * lightSample.getJacobianAtoW() * (diffuseLobe + specularLobe));
}


[numthreads(16, 16, 1)]
void main(uint3 threadId : SV_DispatchThreadID)
{
    printSetPixel(threadId.xy);

    // Find where we are on the screen 
    const uint2 launchIdx = threadId.xy;
    const RTXDI_ResamplingRuntimeParameters params = gRtxdiParams;
    uint2 pixelPosition = RTXDI_ReservoirToPixelPos(launchIdx, params);

    print("current pixel depth = ", gCameraSpaceZBuffer[launchIdx]);

    // Initialize our random number generator.  
    RAB_RandomSamplerState rng = RAB_InitRandomSampler(pixelPosition, 1);
    RAB_RandomSamplerState tileRng = RAB_InitRandomSampler(pixelPosition / RTXDI_TILE_SIZE_IN_PIXELS, 1);
 
    // Grab the current pixel's surface from our G-bufer
    RAB_Surface surface = RAB_GetGBufferSurface(pixelPosition, false);
    float3 surfacePos = RAB_GetSurfaceWorldPos(surface);

    // Grab and shade the requested number of samples.
    //
    // Note: This shader splits the rendering equation into two integrals and numerically integrates separately.
    //       One integral is lighting from the triangular emissives, one integral is lighting from the environment
    //       map.  Since (in our sample, currently) no single direction can have lighting from *both*, we can 
    //       trivially split them apart and add them back together afterwards.
    float3 totalColor = 0;

    // Integrate lighting from triangle emissives
    for (uint i = 0; i < gNumTriSamples; ++i)
    {
        // Pick a random light in the scene, and get the PDF of sampling it
        uint rndLight = 0;
        float invPdf = 0.0f;
        if (gLightSamplingMode == SAMPLE_UNIFORM)           rndLight = SampleTrianglesUniformly(rng, params, invPdf);
        if (gLightSamplingMode == SAMPLE_PDF_TEXTURE)       rndLight = SampleTrianglesViaPDFTexture(rng, invPdf);
        if (gLightSamplingMode == SAMPLE_PRECOMPUTED_TILES) rndLight = SampleTrianglesViaTiles(rng, tileRng, params, invPdf);

        // Get a candidate sample from some random UV on the selected emissive light
        float2 uv = GetRandomUV(rng);   // In [0..1) x [0..1)
        RAB_LightInfo   selectedLight   = RAB_LoadLightInfo(rndLight, false); // this is a triangle light not light sample
        RAB_LightSample candidateSample = RAB_SamplePolymorphicLight(selectedLight, surface, uv); // then we select a sample from this triangle emissive

        // We uniformly sampled our triangle UVs, so update our sampling PDF by 1/<triangle-area> to account for this.
        // Compare to the returned pdf from Falcor's sampleLight() method, the pdf here is for area and Falcor's is 
        // for solid angle, so we need to add the Jacobian term (<geometry-term>) in Shade() function.
        invPdf = invPdf * selectedLight.getLightArea();

        // Shade the sample
        float3 sampleColor = Shade(surface, candidateSample, selectedLight.getSampleEmission(uv));

        // Doing Monte Carlo integration, so accumulate 1/N * sum[ f(x_i) / p(x_i) ]
        totalColor += sampleColor * invPdf * (1.0f / gNumTriSamples);
    }
    
    // Integrate lighting from the environment map
    for (uint i = 0; (i < gNumEnvSamples) && (params.environmentLightPresent > 0); ++i)
    {
        // Grab a random UV sample on the environment light
        float2 uv = 0;
        float invPdf = 0.0f;
        if (gLightSamplingMode == SAMPLE_UNIFORM)           uv = SampleEnvironmentUniformly(rng, invPdf);
        if (gLightSamplingMode == SAMPLE_PDF_TEXTURE)       uv = SampleEnvironmentViaPDFTexture(rng, invPdf);
        if (gLightSamplingMode == SAMPLE_PRECOMPUTED_TILES) uv = SampleEnvironmentViaTiles(rng, tileRng, params, invPdf);

        // Get a candidate from this random UV on this environment map
        RAB_LightInfo lightInfo = RAB_LoadLightInfo(params.environmentLightIndex, false);
        RAB_LightSample candidateSample = RAB_SamplePolymorphicLight(lightInfo, surface, uv);

        // Shade the sample
        float3 sampleColor = Shade(surface, candidateSample, lightInfo.getSampleEmission(uv));

        // Doing Monte Carlo integration, so accumulate 1/N * sum[ f(x_i) / p(x_i) ]
        totalColor += sampleColor * invPdf * (1.0f / gNumEnvSamples);
    }

    // Add any directly-visible emssion & environment maps, then store our final color
    totalColor += gInputEmission[launchIdx].rgb;
    gOutputColor[launchIdx] = float4(totalColor.r, totalColor.g, totalColor.b, 1.0f);

    gDebug[launchIdx] = float4(float3(abs(gCameraSpaceZBuffer[launchIdx].b) / 5.0f), 1.0f);
}
