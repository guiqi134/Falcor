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
    bool gDisplayLightSampling;
}

Texture2D<float4> gInputEmission;     // Emissive color from the G-buffer
RWTexture2D<float4> gOutputColor;     // Full output color (diffuse, specular, emissive, background)
Texture2D<PackedHitInfo> gVbuffer;

Buffer<float2> gThirdStageRankings;

// Shadow map textures
Texture2DArray<float> gTop1Texture;
Texture2DArray<float> gTop2Texture;
Texture2DArray<float> gTop3Texture;
Texture2DArray<float> gTop4Texture;


void Shade(RTXDI_Reservoir reservoir, RAB_Surface surface, RAB_LightSample lightSample, float3 lightEmission, bool hasSampleChanged,
           out float3 dif, out float3 spec)
{
    // Make sure this sample would actually contribute.
    if (!RTXDI_IsValidReservoir(reservoir)) return;

    // Where are we shading / computing a pdf?
    float3 origin = RAB_GetSurfaceWorldPos(surface);

    // Check if the selected light mesh is within the top N list
    float isVisible = 1.0f;
    isVisible = traceShadowRay(RAB_GetSurfaceWorldPos(surface), lightSample.getDirectionAndDistance());

    // Compute our geometry term for the light sample
    float geomTerm = lightSample.getJacobianAtoW();

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
}


[numthreads(16, 16, 1)]
void main(uint3 threadId : SV_DispatchThreadID)
{
    printSetPixel(threadId.xy);

    // Find our pixel location (on screen and in our reservoir list)
    const uint2 launchIdx = threadId.xy;
    const RTXDI_ResamplingRuntimeParameters params = gRtxdiParams;
    uint2 pixelPosition = RTXDI_ReservoirToPixelPos(launchIdx, params);

    // Get our G-buffer for the current pixel
    RAB_Surface surface = RAB_GetGBufferSurface(pixelPosition, false);
    bool isValidSurface = RAB_IsSurfaceValid(surface);

    // Load the reservoir with the light sample we need to shade
    RTXDI_Reservoir reservoir = RTXDI_LoadReservoir(params, launchIdx, gInputReservoirIndex);
    uint lightIndex = RTXDI_GetReservoirLightIndex(reservoir);
    float2 lightUV  = RTXDI_GetReservoirSampleUV(reservoir);

    bool hasSampleChanged = gHasSampleChangedInReusing[launchIdx].r;

    // Do shading at this pixel (if we have a valid G-buffer surface and light sample)
    float3 difColor = 0, specColor = 0;
    if (isValidSurface && RTXDI_IsValidReservoir(reservoir))
    {
        // Lookup the selected light and the specific light sample on it.
        RAB_LightInfo lightInfo = RAB_LoadLightInfo(lightIndex, false);
        RAB_LightSample lightSample = RAB_SamplePolymorphicLight(lightInfo, surface, lightUV);

        // Load the RGB emissive color for this particular light sample
        float3 lightEmission = lightInfo.getSampleEmission(lightUV);

        // Do shading
        Shade(reservoir, surface, lightSample, lightEmission, hasSampleChanged, difColor, specColor);
    }

    // Add any directly-visible emission & environment maps, then store our final color
    float3 outColor = difColor + specColor + gInputEmission[launchIdx].rgb;
    gOutputColor[launchIdx] = float4(outColor.r, outColor.g, outColor.b, 1.0f);

    // Debug shadow map
    // if (true)
    // {
    //     uint face = 3 // -Y
    // }

    // Visualize light mesh selection
    if (gDisplayLightSampling)
    {
        outColor = luminance(outColor);
        float4 currPixelResult = gSelectedLightSampleTextures[launchIdx];
        // uint meshCount = gScene.lightCollection.meshCount;

        uint firstLightID = gScene.lightCollection.meshData[27].instanceID;
        uint secondLightID = gScene.lightCollection.meshData[32].instanceID;
        uint thirdLightID = gScene.lightCollection.meshData[26].instanceID;
        uint fourthLightID = gScene.lightCollection.meshData[99].instanceID;

        print("firstLightID = ", firstLightID);

        if (any(gInputEmission[launchIdx].rgb != float3(0.0f)))
        {
            HitInfo hit = HitInfo(gVbuffer[launchIdx]);
            if (hit.isValid() && hit.getType() == HitType::Triangle)
            {
                // Extract a triangle hit from the V-buffer
                const TriangleHit triangleHit = hit.getTriangleHit();

                if (triangleHit.instanceID.index == firstLightID) outColor = float3(1.0, 0.0, 0.0);
                else if (triangleHit.instanceID.index == secondLightID) outColor = float3(0.0, 1.0, 0.0);
                else if (triangleHit.instanceID.index == thirdLightID) outColor = float3(0.0, 0.0, 1.0);
                else if (triangleHit.instanceID.index == fourthLightID) outColor = float3(1.0, 1.0, 0.0);
            }
        }
        else
        {
            outColor = float3(1.0f);
            if (currPixelResult.x == -1)
            {
                outColor = float3(0.0f);
            }
            else
            {
                if (currPixelResult.x == 27) outColor = float3(1.0, 0.0, 0.0);
                else if (currPixelResult.x == 32) outColor = float3(0.0, 1.0, 0.0);
                else if (currPixelResult.x == 26) outColor = float3(0.0, 0.0, 1.0);
                else if (currPixelResult.x == 99) outColor = float3(1.0, 1.0, 0.0);
            }
        }

        gOutputColor[launchIdx] = float4(outColor, 1.0f);
    }
}
