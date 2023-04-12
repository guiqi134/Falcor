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

 /** A common shader used to shade the final, selected light sample for all
 *   of the RTXDI pipelines we demonstrate in our sample integration.
 */

#include "RtxdiBridge.slangh"

cbuffer ShadeCB
{
    uint gInputReservoirIndex; // Which reservoir contains the samples we should shade
    bool gShadingVisibility;
    uint gVisMode;
}

Texture2D<float4> gInputEmission;     // Emissive color from the G-buffer
RWTexture2D<float4> gOutputColor;     // Full output color (diffuse, specular, emissive, background)
Texture2D<PackedHitInfo> gVbuffer;

float Shade(RTXDI_Reservoir reservoir, RAB_Surface surface, RAB_LightSample lightSample, float3 lightEmission, uint lightIndex, uint2 launchIdx, bool shadingVisibility,
            RAB_RandomSamplerState rng, out float3 dif, out float3 spec)
{
    // Make sure this sample would actually contribute.
    if (!RTXDI_IsValidReservoir(reservoir)) return 0;

    // Where are we shading / computing a pdf?
    float3 origin = RAB_GetSurfaceWorldPos(surface);

    // Compute our geometry term for the light sample
    float geomTerm = lightSample.getJacobianAtoW();

    // Evaluate the visibility
    int localLightID = toLocalLightIndex(lightIndex, gTotalLightMeshCount, gRtxdiParams.firstPointLight, gRtxdiParams.environmentLightIndex, gFalcorLightIDtoOurs);
    uint bufferIndex = gScreenSize.x * launchIdx.y + launchIdx.x;
    float isVisible = 1.0f;
    if (geomTerm != 0.0f)
    {
        uint shadowOption;
        if (shadingVisibility)
            isVisible = (float)evaluateVisibility(bufferIndex, localLightID, surface, lightSample, gVisMode, rng, shadowOption);
    }

    print("lightIndex = ", lightIndex);
    print("localLightID = ", localLightID);

    // Compute the BRDF terms for the target function.
    float D = surface.getNdf(lightSample.NdotH());
    float G = surface.getMaskingTerm(lightSample.NdotL(), surface.NdotV());
    float3 F = any(surface.getSpecularColor() <= 0.0f) ? float3(0.0f) :
        evalFresnelSchlick(surface.getSpecularColor(), float3(1), lightSample.LdotH());

    // Compute the specular and diffuse lobes of the BRDF separately.
    float3 diffuseLobe = lightSample.NdotL() * (surface.getDiffuseColor() * M_1_PI);  // NdotL * albedo
    float3 specularLobe = F * D * G / (4.0f * surface.NdotV());                       // Standard microfacet model

    // Get the PDF of selecting this sample via RIS from the reservoir
    //  -> Multiplying the PDF by the light sample area accounts for the fact that RTXDI does not compute
    //     the sub-light sampling PDF during initial sampling.  By careful factoring, we can move it out
    //     of the sum to here.
    float invPdf = RTXDI_GetReservoirInvPdf(reservoir) * lightSample.getInverseSampleArea();

    // Return our result, separate in diffuse and specular channels:
    //   -> [ <shadow-ray> * L * BRDF * <geometry-term> ] / <pdf-of-ReSTIR-sampling>
    dif  = isVisible * max(float3(0.0f), lightEmission * invPdf * geomTerm * diffuseLobe);
    spec = isVisible * max(float3(0.0f), lightEmission * invPdf * geomTerm * specularLobe);

    return isVisible;
}


[numthreads(16, 16, 1)]
void main(uint3 threadId : SV_DispatchThreadID)
{
    printSetPixel(threadId.xy);

    print("33333333333333333333333", 3);

    // Find our pixel location (on screen and in our reservoir list)
    const uint2 launchIdx = threadId.xy;
    const RTXDI_ResamplingRuntimeParameters params = gRtxdiParams;
    uint2 pixelPosition = RTXDI_ReservoirToPixelPos(launchIdx, params);

    RAB_RandomSamplerState rng = RAB_InitRandomSampler(pixelPosition, 11);

    // Get our G-buffer for the current pixel
    RAB_Surface surface = RAB_GetGBufferSurface(pixelPosition, false);
    bool isValidSurface = RAB_IsSurfaceValid(surface);

    // Load the reservoir with the light sample we need to shade
    RTXDI_Reservoir reservoir = RTXDI_LoadReservoir(params, launchIdx, gInputReservoirIndex);
    uint lightIndex = RTXDI_GetReservoirLightIndex(reservoir);
    float2 lightUV  = RTXDI_GetReservoirSampleUV(reservoir);

    // Do shading at this pixel (if we have a valid G-buffer surface and light sample)
    float3 difColor = 0, specColor = 0;
    float visibility = 1.0f;
    if (isValidSurface && RTXDI_IsValidReservoir(reservoir))
    {
        // Lookup the selected light and the specific light sample on it.
        RAB_LightInfo lightInfo = RAB_LoadLightInfo(lightIndex, false);
        RAB_LightSample lightSample = RAB_SamplePolymorphicLight(lightInfo, surface, lightUV);

        // Load the RGB emissive color for this particular light sample
        float3 lightEmission = lightInfo.getSampleEmission(lightUV);

        // Do shading
        visibility = Shade(reservoir, surface, lightSample, lightEmission, lightIndex, launchIdx, gShadingVisibility, rng, difColor, specColor);
    }

    // Add any directly-visible emission & environment maps, then store our final color
    float3 outColor = difColor + specColor + gInputEmission[launchIdx].rgb;

    // // Store each pixel's light ID first for later face statistics computation
    // if (gVisMode != uint(Visibility::AllShadowRay) && gPlaceForRankingData == uint(PlacesForRankingData::AfterReusing) && !gDisableRankingUpdate)
    // {
    //     RTXDI_Reservoir rankingReservoir = RTXDI_LoadReservoir(params, launchIdx, gInputReservoirIndex + RTXDI_RANKING_RESERVOIR_OFFSET);
    //     if (!gSplitReservoirs) rankingReservoir = reservoir;
    //     storeLightSampleForRanking(rankingReservoir, params, isValidSurface, launchIdx, visibility);
    // }

    gOutputColor[launchIdx] = float4(outColor.r, outColor.g, outColor.b, 1.0f);

    // Visual debugging
    // if (false)
    // {
    //     HitInfo hit = HitInfo(gVbuffer[launchIdx]);
    //     const TriangleHit triangleHit = hit.getTriangleHit();

    //     uint emissiveTriIndex = gScene.lightCollection.getTriangleIndex(triangleHit.instanceID, triangleHit.primitiveIndex);
    //     print("emissive Triangle index = ", emissiveTriIndex);
    //     if (emissiveTriIndex != UINT32_MAX)
    //     {
    //         print("active triangle index = ", gScene.lightCollection.getActiveTriangleIndex(emissiveTriIndex));
    //         print("triangle mesh instance = ", gScene.lightCollection.getLightIdx(emissiveTriIndex));
    //         print("triangle flux = ", gScene.lightCollection.fluxData[emissiveTriIndex].flux);
    //     }

    //     int localLightID = toLocalLightIndex(lightIndex, gTotalLightMeshCount, gRtxdiParams.firstPointLight, gRtxdiParams.environmentLightIndex, gFalcorLightIDtoOurs);

    //     // print("localLightID = ", localLightID);
    //     // print("light triangle index = ", lightIndex);
    //     print("debug light center = ", gLightShadowDataBuffer[gDebugLightID].centerPosW);

    //     // Debugging mesh lights
    //     if (gDebugLightID < gTotalLightMeshCount)
    //     {
    //         uint debugLightMeshInstanceID = gScene.lightCollection.meshData[gDebugLightID].instanceID;

    //         // For pixel has emissive surface
    //         if (any(gInputEmission[launchIdx].rgb != float3(0.0f)))
    //         {

    //             if (hit.isValid() && hit.getType() == HitType::Triangle)
    //             {
    //                 // Turn off all other light's emission
    //                 if (triangleHit.instanceID.index != debugLightMeshInstanceID)
    //                     outColor = float3(0.0f);
    //             }
    //         }
    //         else
    //         {
    //             if (localLightID != gDebugLightID)
    //             {
    //                 outColor = float3(0.0f);
    //             }
    //         }
    //     }
    //     // Debugging point lights
    //     else
    //     {
    //         if (localLightID != gDebugLightID)
    //         {
    //             outColor = float3(0.0f);
    //         }
    //         else
    //         {
    //             outColor = difColor + specColor;
    //         }
    //     }
    // }
}
