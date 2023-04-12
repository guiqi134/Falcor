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
// #include "BaselineSampling.slangh"

cbuffer InitialCB
{
    uint gRankingImportance;
    uint gLocalSamples;
}


// Return a triangle light index and 1/pdf of sampling that light (for SAMPLE_UNIFORM)
uint SampleUniformly(inout RAB_RandomSamplerState rng, RTXDI_ResamplingRuntimeParameters params, out float invSourcePdf)
{
    // PDF is 1 / number-of-lights
    invSourcePdf = float(params.numLocalLights);

    // Randomly pick one of the triangle lights
    float rnd = RAB_GetNextRandom(rng);
    return min(uint(floor(rnd * params.numLocalLights)), params.numLocalLights - 1) + params.firstLocalLight;
}

// Return a triangle light index and 1/pdf of sampling that light (for SAMPLE_PDF_TEXTURE)
uint SampleEmissivePowerPDFTexture(inout RAB_RandomSamplerState rng, out float invSourcePdf)
{
    // Call RTXDI function to sample the lights and get the PDF
    uint2 texelPosition, textureSize;
    float pdf;
    gLocalLightPdfTexture.GetDimensions(textureSize.x, textureSize.y);
    RTXDI_SamplePdfMipmap(rng, gLocalLightPdfTexture, textureSize, texelPosition, pdf);
    invSourcePdf = 1.0f / pdf;

    // Map the texel selected to a light ID and return it
    return RTXDI_ZCurveToLinearIndex(texelPosition);
}

// Return a triangle light index and 1/pdf of sampling that light (for SAMPLE_PRECOMPUTED_TILES)
// This function needs a bit more data, so it can compute which tile we're currently in.
uint SampleViaTiles(inout RAB_RandomSamplerState rng, inout RAB_RandomSamplerState coherentRng,
                             RTXDI_ResamplingRuntimeParameters params, out float invSourcePdf)
{
    // Randomly select which light tile to use; find where it is in the buffer
    float tileRnd = RAB_GetNextRandom(coherentRng);
    uint tileIndex = uint(tileRnd * params.tileCount);
    uint risBufferBase = tileIndex * params.tileSize; // start index in RIS presampled light buffer

    // Randomly pick one light in the tile; look it up.
    float rnd = RAB_GetNextRandom(rng);
    uint risSample = min(uint(floor(rnd * params.tileSize)), params.tileSize - 1);
    uint2 presampledLight = gRisBuffer[risBufferBase + risSample];

    // Unpack the light index and its pdf from the tile structure.
    invSourcePdf = asfloat(presampledLight.y);
    return presampledLight.x & RTXDI_LIGHT_INDEX_MASK;
}

// This is a very simplistic, cut down version of the RTXDI function RTXDI_SampleLightsForSurface()
//  -> We define this function, rather than use the API to help you understand what the API function does
void SimpleLightSampler(inout RTXDI_Reservoir state, inout RAB_RandomSamplerState rng,
    inout RAB_RandomSamplerState coherentRng, RAB_Surface surface, uint numSamples, RTXDI_ResamplingRuntimeParameters params)
{
    if (params.numLocalLights == 0 || numSamples == 0) return;

    // Randomly select some number of initial candidate light samples, evaluate these samples'
    // PDFs (i.e., q(x) in the RIS estimator), evaluate our target functions at these samples
    // (i.e., p-hat(x) in the RIS estimator), and run them through a streaming resampled
    // importance sampling algorithm to select one, which is stored in our reservoir for later (re-)use.
    for (uint i = 0; i < numSamples; i++)
    {
        // We need to pick one of our emissive triangles randomly.  RTXDI really only uses SAMPLE_PRECOMPUTED_TILES mode
        uint rndLight = 0;
        float invPdf;
        if (gRankingImportance == uint(RankingImportance::onlyUniformRIS)) rndLight = SampleUniformly(rng, params, invPdf);
        if (gRankingImportance == uint(RankingImportance::onlyEmissivePowerRIS))
        {
            if ((bool)params.enableLocalLightImportanceSampling)
                rndLight = SampleViaTiles(rng, coherentRng, params, invPdf);
            else
                rndLight = SampleEmissivePowerPDFTexture(rng, invPdf);
        }

        // Get a candidate from some random UV on this selected emissive light
        float2 uv = float2(RAB_GetNextRandom(rng), RAB_GetNextRandom(rng));
        RAB_LightInfo lightInfo = RAB_LoadLightInfo(rndLight, false);
        RAB_LightSample candidateSample = RAB_SamplePolymorphicLight(lightInfo, surface, uv);

        // We also have analytic light, so it will always return 1.0f
        //   -> NOTE:  RTXDI does not do this in its sampling routines, which requires corrections elsewhere to account for it
        invPdf *= lightInfo.getLightArea();

        // Find the target function at the sample, stream it by the RIS reservoir
        float targetPdf = RAB_GetLightSampleTargetPdfForSurface(candidateSample, surface);
        RTXDI_StreamSample(state, rndLight, uv, RAB_GetNextRandom(rng), targetPdf, invPdf);
    }

    // This API call converts our weight-sum into a weight-average.
    RTXDI_FinalizeResampling(state, 1.0, state.M);

    // Since *all* our pixels use the same # of initial candidates, we can use a relative reservoir weight of 1
    state.M = 1;
}

[numthreads(16, 16, 1)]
void main(uint3 threadId : SV_DispatchThreadID)
{
    // Find where we are on the screen and in the reservoir
    const uint2 launchIdx = threadId.xy;
    const RTXDI_ResamplingRuntimeParameters params = gRtxdiParams;
    uint2 pixelPosition = RTXDI_ReservoirToPixelPos(launchIdx, params); // under normal cases, we can assume pixelPosition = launchIdx

    // Initialize our random number generators.  The second is only needed if we sample lights
    //    from (once-per-frame) precomputed tiles, i.e., gLightSamplingMode == SAMPLE_PRECOMPUTED_TILES
    RAB_RandomSamplerState rng = RAB_InitRandomSampler(pixelPosition, 1);
    RAB_RandomSamplerState tileRng = RAB_InitRandomSampler(pixelPosition / RTXDI_TILE_SIZE_IN_PIXELS, 1); // pixels in same tile will get same RNG

    // Grab the current pixel's surface from our G-bufer
    RAB_Surface surface = RAB_GetGBufferSurface(int2(pixelPosition), false);

    // Generate initial candidate samples on emissive triangles; select one via RIS
    //   -> Rather than calling the RTXDI API, RTXDI_SampleLightsForSurface(), we write our own
    //      variant to allow us to tweak it, and use one of three different sampling algorithms
    RTXDI_Reservoir analyticReservoir = RTXDI_EmptyReservoir();
    SimpleLightSampler(analyticReservoir, rng, tileRng, surface, gLocalSamples, params);

    // Merge these two reservoirs together.  By doing this reservoir merging, we don't have to account
    // for weighting triangle and environment map samples different based on how many of each we use.
    // Note, this can be done in-place, rather than using 3 different reservoirs, but this makes it clear what's happening
    //  -> Also note, triReservoir and envReservoir already finalized, so their weight-sum is weight-average
    RTXDI_Reservoir reservoir = RTXDI_EmptyReservoir();
    RTXDI_CombineReservoirs(reservoir, analyticReservoir, 0.5f, analyticReservoir.targetPdf);  // 0.5f as combining with an empty reservoir always succeeds

    // Do final normalization of our merged reservoir
    RTXDI_FinalizeResampling(reservoir, 1.0, 1.0); // Song: Why the third parameter is not 2?
    reservoir.M = 1;

    // Store this simple sample in our reservoir; pass it forward to the shader that will evaluate pixel colors
    RTXDI_StoreReservoir(reservoir, params, launchIdx, 0);

    if (gOnlyIsmForRanking)
        RTXDI_StoreReservoir(reservoir, params, launchIdx, RTXDI_RANKING_RESERVOIR_OFFSET);
}
