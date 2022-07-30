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

/** A common shader used to generate per-pixel candidates for all of the RTXDI
*   pipelines we demonstrate in our sample integration.
*/

#include "RtxdiBridge.slangh"

cbuffer SampleCB
{
    uint  gLocalSamples;           // How many candidate lights should RTXDI pick on emissive triangles / primitives
    uint  gEnvironmentSamples;     // How many candidate lights should RTXDI pick on the environment map
    uint  gOutputReservoirIndex;   // Which of our reservoirs should we use for output?
}


[numthreads(16, 16, 1)]
void main(uint3 threadId : SV_DispatchThreadID)
{
    printSetPixel(threadId.xy);

    // Find where we are on the screen and in the reservoir
    const uint2 launchIdx = threadId.xy;
    const RTXDI_ResamplingRuntimeParameters params = gRtxdiParams;
    uint2 pixelPosition = RTXDI_ReservoirToPixelPos(launchIdx, params);

    // Initialize our random sampler.  Since RTXDI generally uses light tiles, we need one
    // per-pixel RNG plus one that is consistent across all pixels that use the same tile.
    RAB_RandomSamplerState rng = RAB_InitRandomSampler(pixelPosition, 1);
    RAB_RandomSamplerState tileRng = RAB_InitRandomSampler(pixelPosition / RTXDI_TILE_SIZE_IN_PIXELS, 1);

    // Get data about the pixel we're shading from our G-buffer
    RAB_Surface surface = RAB_GetGBufferSurface(pixelPosition, false);

    // Set up how many candidate samples of various types we want to take.
    RTXDI_SampleParameters sampleParams = RTXDI_InitSampleParameters(
        0,                    // Number of samples to take from ReGIR.
        gLocalSamples,        // Number of samples to take from local (i.e., triangle or other primitive) lights?
        0,                    // Number of samples to take from infinite (i.e., directional) lights?
        gEnvironmentSamples,  // Number of samples to take from the environment map?
        0,                    // Number of BRDF-distributed samples to take
        0,                    // The BRDF cutoff
        0.001f);

    // Generate initial candidates using the RTXDI API
    // The pdf of reservoir's sample didn't include the pdf of sampling inside light's (triangle) area
    // so we need to multiply it in final shading
    RAB_LightSample lightSample;
    RTXDI_Reservoir reservoir = RTXDI_SampleLightsForSurface(rng, tileRng, surface,
        sampleParams, params, lightSample);

    // Stage 1: Do statistics after generating candidates
    float4 lightSampleData = float4(-1); // light mesh ID, triangle ID, UV
    if (RTXDI_IsValidReservoir(reservoir))
    {
        uint triIdx = RTXDI_GetReservoirLightIndex(reservoir);
        lightSampleData.x = gScene.lightCollection.getLightIdx(triIdx);
        lightSampleData.y = triIdx; 
        lightSampleData.zw = RTXDI_GetReservoirSampleUV(reservoir); 
    }

    gSelectedLightSampleTextures[uint2(launchIdx)] = lightSampleData;

    print("lightSampleData = ", lightSampleData);

    // Store data about our selected light in an intermediate reservoir for later reuse and shading.
    RTXDI_StoreReservoir(reservoir, params, launchIdx, gOutputReservoirIndex);
}
