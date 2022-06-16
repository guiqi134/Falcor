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

#include "RTXDITutorialConstants.h"

using namespace Falcor;

 ////////////////////////////////////////////////////////////////////////////////////////////////////////
 // Filename / directory constants
 ////////////////////////////////////////////////////////////////////////////////////////////////////////

 // Where are our shaders located?
const std::string kShaderDirectory = "RenderPasses/RTXDITutorials/Shaders/";

// Example shaders that help set up various optional structures for RTXDI
const std::string kRTXDI_CreateLightResources = "RTXDI_BuildLightResourcesFromLightCollection.cs.slang";
const std::string kRTXDI_CreateEnvMapPDFTexture = "RTXDI_BuildEnvMapPDFTexture.cs.slang";
const std::string kRTXDI_PresamplePrimitiveLights = "RTXDI_PresampleLights.cs.slang";
const std::string kRTXDI_PresampleEnvironmentLights = "RTXDI_PresampleEnvironmentLights.cs.slang";
const std::string kRTXDI_PrepareSurfaceData = "RTXDI_PrepareSurfaceData.cs.slang";

// Example shader for doing a Monte Carlo integration baseline (i.e., shoot a shadow ray for each candidate; use all to shade)
const std::string kRTXDI_MonteCarloBaseline = "RTXDI_MonteCarloBaseline.cs.slang";

// Example shaders for a very simple RTXDI pipeline with only Talbot RIS, and no spatiotemporal reuse
const std::string kRTXDI_RISOnly_InitialSamples = "RTXDI_RISOnly_GenerateCandidates.cs.slang";
const std::string kRTXDI_RISOnly_Shade = "RTXDI_RISOnly_Shade.cs.slang";

// Example shaders to generate candidates and do final shading for general RTXDI pipelinese
const std::string kRTXDI_InitialSamples = "RTXDI_GenerateCandidates.cs.slang";
const std::string kRTXDI_InitialVisibility = "RTXDI_TestCandidateVisibility.cs.slang";
const std::string kRTXDI_Shade = "RTXDI_Shade.cs.slang";

// Example shaders doing ReSTIR with varying types of reuse
const std::string kRTXDI_SpatialReuseOnly = "RTXDI_SpatialReuse.cs.slang";
const std::string kRTXDI_TemporalReuseOnly = "RTXDI_TemporalReuse.cs.slang";
const std::string kRTXDI_SpatiotemporalReuse = "RTXDI_SpatiotemporalReuse.cs.slang";


////////////////////////////////////////////////////////////////////////////////////////////////////////
// GUI Constants
////////////////////////////////////////////////////////////////////////////////////////////////////////

// For our baselines (Talbot RIS & Monte Carlo integration), which method should we use to choose light samples?
const Gui::RadioButtonGroup kSampleList = {
    { kUniformSamplingMode,         "Uniform light sampling", false },
    { kPdfSamplingMode,             "PDF texture light sampling", false },
    { kPrecomputedTileSamplingMode, "Use precomputed light tiles", false },   // *Always* used if asking RTXDI to sample
};

// For RTXDI-based reuse, which bias correction method is used?
const Gui::DropdownList kBiasCorrection = {
    { 0, "None (i.e., 1/M weighting)"},                   // Cheapest & lowest quality
    { 1, "MIS without visibility rays"},                  // Still a bit biased
    { 2, "Pairwise MIS (better perf & quality)"},         // Uses pairwise MIS, better perf & quality, but still a bit biased (best tradeoff)
    { 3, "Unbiased (adds rays, cost, and often noise)"},  // Unbiased, i.e., converges to MC integration, but is more costly
};

// We have a bunch of different example RTXDI pipelines in different tutorials.  Our "combined" tutorials
// allows you to switch between them all dynamically, so we need a dropdown GUI element to control this
const Gui::DropdownList kDropList = {
    {kMonteCarloBaselineMode,  "Monte Carlo integration"},  // see runMonteCarloLighting()
    {kTalbotRISMode,           "Talbot RIS (very basic)"},  // see runBasicRISLighting()
    {kSpatialOnlyReuseMode,    "Spatial reuse only"},       // see runSpatialReuseOnly()
    {kTemporalOnlyReuseMode,   "Temporal reuse only"},      // see runTemporalReuseOnly()
    {kSpatiotemporalReuseMode, "Spatiotemporal reuse"},     // see runSpatioTemporalReuse()
};


////////////////////////////////////////////////////////////////////////////////////////////////////////
// GUI Tooltip Constants.  Long explanatory constants to help explain GUI options
////////////////////////////////////////////////////////////////////////////////////////////////////////

const std::string kSampleCountToolTip = "RTXDI allows specifying the number of initial candidates to select from "
    "each light type.  In the current setup, you need to pick at least one sample from each light type present "
    "in the scene in order for all lights to contribute.  Typically RTXDI bins together all area emissives "
    "of any geometry type (spheres, triangles, rects, capsules, etc.), though for these tutorials only "
    "support emissive triangles.";

const std::string kCandidateVisibilityToolTip = "Determines whether the single selected light candidate is tested "
    "for visibility (at each pixel) *before* the sample has a chance to be reused spatially or temporally. "
    "Occluded samples have their reseroirs zeroed out, so such a sample never has a chance to contribute "
    "to neighbors.  This is especially valuable in multi-room scenes, where occluded lights from a different "
    "room are also unlikely to light neighbors.";

const std::string kPerTileBuffersToolTip = "For scenes with thousands or millions of lights, memory incoherency and "
    "cache misses become a big performance limiter. To compensate, we can create a pre-packed buffer of light "
    "data for each presampled tile of lights.  This stores light geometry data in more coherent clusters that "
    "are likely to be accessed by adjacent pixels.  However, creating these buffers has a cost and can actually "
    "decrease memory coherence in simpler scenes; in this tutorial code, enabling this setting only gives a "
    "performance win in our test scenes with the most lights (e.g., the Amusement Park scene from our papers).  "
    "However, when integrating RTXDI into a different engine, the cost of your RAB_LoadLightInfo() function "
    "(required by the RTXDI bridge) may make enabling this setting a win even in simpler scenes.";

const std::string kSamplingTypeToolTip = "This tutorial does not (necessarily) rely on built-in RTXDI sampling "
    "routines, instead explicitly defining various more naive methods (e.g., uniform sampling) that we can " 
    "select.  This helps build confidence that the built-in routines are actually decent and performing correctly. "
    "'Uniform' sampling is extremely naive, but also easy to verify.  'PDF texture' sampling uses a PDF-mipmap "
    "created by RTXDI to sample proportional to light intensity in log(N) steps.  'Precomputed tiles' is the "
    "most performant approach, descibed in the HPG 2021 paper 'Rearchitecting Spatiotemporal Resampling for Production'.";

const std::string kNaiveSampleToolTip = "How many Monte Carlo samples do we take per-pixel on various types of lights "
    "in the scene.  Obviously, the cost is proportional to the number of these samples, and we trace one shadow ray to each "
    "selected sample.  This excessive number of shadow rays explains the (relatively) poor performance.";

const std::string kBaiscRISSampleToolTip = "How many per-pixel candidate samples for our RIS pass do we take on various types of lights.  "
    "The cost is proportional to the number of these samples, but we only trace one shadow ray to the single selected light, "
    "so this is significantly faster than our naive Monte Carlo integration (but lower quality).";

const std::string kPrecomputedTileToolTip = "By moving explicit sampling of our numerous lights out of the inner loop "
    "and into a preprocess, we can significantly reduce per-pixel memory divergence.  This can dramatically improve "
    "performance, especially as the number of lights in a scene grows.  It also helps on environment maps, and if your "
    "renderer supports many different light types.  See the HPG 2021 paper 'Rearchitecting Spatiotemporal Resampling "
    "for Production' for more details and discussion.  Using 128 tiles, each with 1024 lights seems to be a good setting "
    "across a wide variety of scenes.  In simpler scenes, this can be reduced significantly (but also doesn't seem to "
    "impact performance or quality, if left alone).";
