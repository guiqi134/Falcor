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

#include "RtxdiBridge.slangh"

Texture2D<float4> gInputEmission;     // Emissive color from the G-buffer
RWTexture2D<float4> gOutputColor;     // Full output color (diffuse, specular, emissive, background)


// Our per-pixel shading routine, given the RIS/ReSTIR reservoir, the current pixel surface, light sample,
//    and looked-up / computed emissivity for the sample.
void Shade(RTXDI_Reservoir reservoir, RAB_Surface surface, RAB_LightSample lightSample, float3 lightEmission,
           out float3 dif, out float3 spec)
{
    // Make sure this sample would actually contribute.
    if (!RTXDI_IsValidReservoir(reservoir)) return;

    // Shoot a shadow ray to the selected light sample
    float isVisible = traceShadowRay(RAB_GetSurfaceWorldPos(surface), lightSample.getDirectionAndDistance());

    // Compute the BRDF terms for the target function.
    float D = surface.getNdf(lightSample.NdotH());
    float G = surface.getMaskingTerm(lightSample.NdotL(), surface.NdotV());
    float3 F = any(surface.getSpecularColor() <= 0.0f) ? float3(0.0f) :
        evalFresnelSchlick(surface.getSpecularColor(), float3(1), lightSample.LdotH());

    // Compute the specular and diffuse lobes of the BRDF separately.
    float3 diffuseLobe = lightSample.NdotL() * (surface.getDiffuseColor() * M_1_PI);  // NdotL * albedo 
    float3 specularLobe = F * D * G / (4.0f * surface.NdotV());                       // Standard microfacet model 

    // Get the PDF of selecting this sample via RIS from the reservoir
    //   -> PLEASE NOTE:  When using the full RTXDI API (i.e., not the simplistic light sampler in
    //      shader "RTXDI_RISOnly_GenerateCandidates.cs.slang"), this line has another term, since
    //      RTXDI ignores part of the sampling PDF for initial candidates, and this gets refactored
    //      out to here to ensure correct PDFs.
    float invPdf = RTXDI_GetReservoirInvPdf(reservoir);

    // Compute our geometry term for the light sample
    float geomTerm = lightSample.getJacobianAtoW();

    // Return our result, separate in diffuse and specular channels:
    //   -> [ <shadow-ray> * L * <geometry-term> * BRDF] / <pdf-of-sampling>
    dif  = isVisible * max(float3(0.0f), lightEmission * invPdf * geomTerm * diffuseLobe);
    spec = isVisible * max(float3(0.0f), lightEmission * invPdf * geomTerm * specularLobe);
}


[numthreads(16, 16, 1)]
void main(uint3 threadId : SV_DispatchThreadID)
{
    // Find our pixel location (on screen and in our reservoir list)
    const uint2 launchIdx = threadId.xy;
    const RTXDI_ResamplingRuntimeParameters params = gRtxdiParams;
    uint2 pixelPosition = RTXDI_ReservoirToPixelPos(launchIdx, params);

    // Get our G-buffer for the current pixel
    RAB_Surface surface = RAB_GetGBufferSurface(pixelPosition, false);

    bool isValidSurface = RAB_IsSurfaceValid(surface);

    // Load the reservoir with the light sample we need to shade
    RTXDI_Reservoir reservoir = RTXDI_LoadReservoir(params, launchIdx, 0);
    uint lightIndex = RTXDI_GetReservoirLightIndex(reservoir);
    float2 lightUV  = RTXDI_GetReservoirSampleUV(reservoir);

    // Do shading at this pixel (if we have a valid G-buffer surface and light sample)
    float3 difColor = 0;    // Why split diffuse & specular color?  It makes a denoisers' job much easer!
    float3 specColor = 0;
    if (isValidSurface && RTXDI_IsValidReservoir(reservoir))
    {
        // Lookup the reservoir, get our light and specific light sample.
        RAB_LightInfo lightInfo = RAB_LoadLightInfo(lightIndex, false);
        RAB_LightSample lightSample = RAB_SamplePolymorphicLight(lightInfo, surface, lightUV);

        // Load the RGB emissive color for this particular light sample
        float3 lightEmission = lightInfo.getSampleEmission(lightUV);

        // Do shading
        Shade(reservoir, surface, lightSample, lightEmission, difColor, specColor);
    }

    // Add any directly-visible emssion & environment maps, then store our final color
    float3 outColor = difColor + specColor + gInputEmission[launchIdx].rgb;
    gOutputColor[launchIdx] = float4(outColor.r, outColor.g, outColor.b, 1.0f);
}
