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
#include "RTXDITutorial3.h"

 /** 
 */

using namespace Falcor;

// Name of the pass and a text description, as exposed in the "RTXDITutorials.dll" for use by Falcor.
RenderPass::Info RTXDITutorial3::getClassDescription()
{
    return { "RTXDITutorial3", "Using RTXDI to spatially reuse light samples." };
}

// Our object factory, needed to expose instantiation of this RenderPass externally, outside the "RTXDITutorials.dll"
RTXDITutorial3::SharedPtr RTXDITutorial3::create(RenderContext* pRenderContext, const Dictionary& dict)
{
    return SharedPtr(new RTXDITutorial3(dict));
}

// Why do we have an explicit constructor for this tutorial?  The default number of spatial samples needs to be higher
// when doing *only* spatial reuse.  Other tutorials can use fewer spatial samples (see the HPG 2021 paper "Rearchitecting
// Spatiotemporal Resampling for Production" for details) so we need to override the base class default in this case.
RTXDITutorial3::RTXDITutorial3(const Dictionary& dict)
    : RTXDITutorialBase(dict, getClassDescription())
{
    mLightingParams.spatialSamples = 5u;
    mLightingParams.spatialIterations = 1u;
}

void RTXDITutorial3::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    mpPixelDebug->beginFrame(pRenderContext, mPassData.screenSize);

    // The rest of the rendering code in this pass may fail if there's no scene loaded
    if (!mPassData.scene) return;

    // If we haven't loaded our shaders yet, go ahead and load them.
    if (!mShader.shade) loadShaders();

    // If needed, allocate an RTXDI context & resources so we can run our lighting code
    if (!mpRtxdiContext) allocateRtxdiResrouces(pRenderContext, renderData);

    // A few UI changes give screwy results if reusing across the change; zero our reservoirs in these cases
    if (mPassData.clearReservoirs)
    {
        pRenderContext->clearUAV(mResources.reservoirBuffer->getUAV().get(), uint4(0u));
        mPassData.clearReservoirs = false;
    }

    // See what, if anything, changed since last frame.  Changes might indicate we need to update
    // data passed to RTXDI.  This queries our Falcor scene to update flags that track changes.
    checkForSceneUpdates();

    // Convert our input (standard) Falcor v-buffer into a packed G-buffer format to reduce reuse costs
    prepareSurfaceData(pRenderContext, renderData);

    // For each pixel in our image, randomly select some number of candidate light samples, (optionally) send
    // shadow rays to the selected one, reuse this data spatially between some neighbors, and then accumulate
    // lighting for one final selected light sample in each pixel.  This is ReSTIR with only spatial reuse.
    runSpatialReuseOnly(pRenderContext, renderData);

    // Increment our frame counter for next frame.  This is used to seed a RNG, which we want to change each frame 
    mRtxdiFrameParams.frameIndex++;

    mpPixelDebug->endFrame(pRenderContext);
}

void RTXDITutorial3::runSpatialReuseOnly(RenderContext* pRenderContext, const RenderData& renderData)
{
    // Set our RTXDI parameters for the current frame.
    //  -> This calls RTXDI's rtxdi::FillRuntimeParameters() routine.
    setCurrentFrameRTXDIParameters(renderData);

    // Create a PDF texture for our primitive lights (for now, just triangles)
    computePDFTextures(pRenderContext, renderData);

    // Create tiles of presampled lights once per frame to improve per-pixel memory coherence.
    // See:  Wyman & Panteleev, "Rearchitecting Spatiotemporal Resampling for Production," HPG 2021.
    presampleLights(pRenderContext, renderData);

    // Step 1: Pick candidate lights
    {
        FALCOR_PROFILE("Pick Candidates");
        auto candidatesVars = mShader.initialCandidates->getRootVar();
        candidatesVars["SampleCB"]["gLocalSamples"] = mLightingParams.primLightSamples;
        candidatesVars["SampleCB"]["gEnvironmentSamples"] = mLightingParams.envLightSamples;
        candidatesVars["SampleCB"]["gOutputReservoirIndex"] = uint(0);
        setupRTXDIBridgeVars(candidatesVars, renderData);
        mShader.initialCandidates->execute(pRenderContext, mPassData.screenSize.x, mPassData.screenSize.y);
    }

    // Step 2: (Optionally, but *highly* recommended) Test visibility for selected candidate.  
    if (mLightingParams.traceInitialShadowRay)
    {
        FALCOR_PROFILE("Candidate Visibility");
        auto visVars = mShader.initialCandidateVisibility->getRootVar();
        visVars["SampleCB"]["gReservoirIndex"] = uint(0);
        setupRTXDIBridgeVars(visVars, renderData);
        mShader.initialCandidateVisibility->execute(pRenderContext, mPassData.screenSize.x, mPassData.screenSize.y);
    }

    // We ping-pong between reservoir buffers, depending on # of spatial iterations
    uint32_t inBuffer = 0u;  // The gOutputReservoirIndex from above
    uint32_t outBuffer = 1u; // The reservoir our first spatial reuse pass outputs into

    // Step 3: Do spatial reuse
    {
        FALCOR_PROFILE("Spatial Reuse");

        for (uint32_t i = 0; i < mLightingParams.spatialIterations; ++i)
        {
            auto spatialVars = mShader.spatialReuse->getRootVar();
            spatialVars["ReuseCB"]["gInputReservoirIndex"] = uint(inBuffer);
            spatialVars["ReuseCB"]["gOutputReservoirIndex"] = uint(outBuffer);
            spatialVars["ReuseCB"]["gNumSamples"] = uint(mLightingParams.spatialSamples);
            spatialVars["ReuseCB"]["gSamplesInDisocclusions"] = uint(mLightingParams.spatialSamples);
            spatialVars["ReuseCB"]["gMaxHistory"] = uint(mLightingParams.maxHistoryLength);
            spatialVars["ReuseCB"]["gBiasCorrectionMode"] = uint(mLightingParams.biasCorrectionMode);
            spatialVars["ReuseCB"]["gReuseRadius"] = float(mLightingParams.spatialRadius);
            spatialVars["ReuseCB"]["gDepthThreshold"] = float(mLightingParams.depthThreshold);
            spatialVars["ReuseCB"]["gNormalThreshold"] = float(mLightingParams.normalThreshold);
            setupRTXDIBridgeVars(spatialVars, renderData);
            mpPixelDebug->prepareProgram(mShader.spatialReuse->getProgram(), spatialVars);
            mShader.spatialReuse->execute(pRenderContext, mPassData.screenSize.x, mPassData.screenSize.y);

            // Ping pong our input and output buffers.
            inBuffer = 1u - inBuffer;
            outBuffer = 1u - outBuffer;
        }
    }

    // Step 4: Do final shading
    {
        FALCOR_PROFILE("Final shading");
        auto shadeVars = mShader.shade->getRootVar();
        shadeVars["ShadeCB"]["gInputReservoirIndex"] = uint(inBuffer);
        shadeVars["gOutputColor"] = renderData["color"]->asTexture();
        shadeVars["gInputEmission"] = mResources.emissiveColors;
        setupRTXDIBridgeVars(shadeVars, renderData);
        mShader.shade->execute(pRenderContext, mPassData.screenSize.x, mPassData.screenSize.y);
    }
}


// Renders the GUI used to change options on the fly when running in Mogwai.
void RTXDITutorial3::renderUI(Gui::Widgets& widget)
{
    // For pixel debug UI
    RTXDITutorialBase::renderUI(widget);

    // Provide controls for the number of samples on each light type (largely consistent options between tutorials)
    Gui::Group candidateOptions(widget.gui(), "Per-pixel light sampling", true);
    candidateOptions.text("Number of per-pixel light candidates on:");
    candidateOptions.tooltip(kSampleCountToolTip);
    candidateOptions.var("Triangles", mLightingParams.primLightSamples, 0u, 256u);
    candidateOptions.var("Environment", mLightingParams.envLightSamples, 0u, 256u);
    if (candidateOptions.checkbox("Test selected candidate visibility?", mLightingParams.traceInitialShadowRay))
        mPassData.clearReservoirs = true;
    candidateOptions.tooltip(kCandidateVisibilityToolTip);
    candidateOptions.checkbox("Create per-tile light buffers?", mLightingParams.storePerTileLightGeom);
    candidateOptions.tooltip(kPerTileBuffersToolTip);
    candidateOptions.release();

    // Provide controllable options for each different rendering mode (e.g., each different tutorial)
    Gui::Group pipeOptions(widget.gui(), "Renderer Options", true);
    pipeOptions.dropdown("Bias correction", kBiasCorrection, mLightingParams.biasCorrectionMode);
    pipeOptions.var("Reuse radius", mLightingParams.spatialRadius, 0.0f, 50.0f, 0.1f);
    pipeOptions.var("Samples", mLightingParams.spatialSamples, 0u, 25u);
    pipeOptions.var("Iterations", mLightingParams.spatialIterations, 0u, 10u);
    pipeOptions.var("Depth Threshold", mLightingParams.depthThreshold, 0.0f, 1.0f, 0.001f);
    pipeOptions.var("Normal Threshold", mLightingParams.normalThreshold, 0.0f, 1.0f, 0.001f);
    pipeOptions.release();

    // RTXDI context parameters.  If any of these change, the RTXDI context needs to be rebuilt before the next use
    Gui::Group contextOptions(widget.gui(), "Precomputed light tile sizes", false);
    contextOptions.tooltip(kPrecomputedTileToolTip);
    if (contextOptions.var("Tile Count", mLightingParams.presampledTileCount, 1u, 1024u)) mpRtxdiContext = nullptr;
    if (contextOptions.var("Tile Size", mLightingParams.presampledTileSize, 256u, 8192u, 128u)) mpRtxdiContext = nullptr;
    contextOptions.release();
}
