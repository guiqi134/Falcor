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
#include "RTXDITutorial5.h"

 /** 
 */

using namespace Falcor;

// Name of the pass and a text description, as exposed in the "RTXDITutorials.dll" for use by Falcor.
RenderPass::Info RTXDITutorial5::getClassDescription()
{
    return { "RTXDITutorial5", "Using RTXDI to spatially and temporally reuse light samples." };
}

// Our object factory, needed to expose instantiation of this RenderPass externally, outside the "RTXDITutorials.dll"
RTXDITutorial5::SharedPtr RTXDITutorial5::create(RenderContext* pRenderContext, const Dictionary& dict)
{
    return SharedPtr(new RTXDITutorial5(dict));
}

void RTXDITutorial5::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    mpPixelDebug->beginFrame(pRenderContext, mPassData.screenSize);

    // The rest of the rendering code in this pass may fail if there's no scene loaded
    if (!mPassData.scene) return;

    // Update the scene, especially dynamic meshes because they will not be pre-transferred
    mPassData.scene->update(pRenderContext, gpFramework->getGlobalClock().getTime());

    // If we haven't loaded our shaders yet, go ahead and load them.
    if (!mShader.shade) loadShaders();

    // Add emissive sampler define to baseline shading
    //if (mpEmissiveSampler)
    //{
    //    mpEmissiveSampler->update(pRenderContext);
    //    mpBaselineShading->getProgram()->addDefines(mpEmissiveSampler->getDefines());
    //}

    // If needed, allocate an RTXDI context & resources so we can run our lighting code
    if (!mpRtxdiContext) allocateRtxdiResrouces(pRenderContext, renderData);

    // If needed, allocate the resources in our method
    if (!mpSortedLightsBuffer) allocateStochasticSmResrouces(pRenderContext, renderData);

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

    // Update the data in this render pass
    RTXDITutorialBase::execute(pRenderContext, renderData);

    // Wait until ReSTIR is stable. This should happen after updating the params
    mEnableShadowRay = mRtxdiFrameParams.frameIndex < mLightingParams.maxHistoryLength ? true : false;

    // Toggle between different visibility algorithms
    if ((mVisibility != Visibility::BaselineSM || mVisibility != Visibility::AllShadowRay)
        && !mEnableShadowRay)
    {
        // Do stochastic shadow map passes using last frame ReSTIR sample data
        prepareStochasticShadowMaps(pRenderContext, renderData);
        runSpatioTemporalReuse(pRenderContext, renderData);
    }
    else if (mVisibility == Visibility::BaselineSM && !mEnableShadowRay)
    {
        runBaselineShadowMap(pRenderContext, renderData);
    }
    else 
    {
        // For each pixel in our image, randomly select some number of candidate light samples, (optionally) send
        // shadow rays to the selected one, reuse this data spatially between some neighbors, and then accumulate
        // lighting for one final selected light sample in each pixel.  This is ReSTIR with only spatial reuse.
        runSpatioTemporalReuse(pRenderContext, renderData);
    }

    // Increment our frame counter for next frame.  This is used to seed a RNG, which we want to change each frame 
    mRtxdiFrameParams.frameIndex++;

    // When we do temporal reuse, we need a G-buffer for this frame *and* last frame to compute shading.  We have
    // two G-buffer indicies (0 and 1), ping-pong back and forth between them each frame.
    mLightingParams.currentGBufferIndex = 1u - mLightingParams.currentGBufferIndex;
    mLightingParams.priorGBufferIndex = 1u - mLightingParams.priorGBufferIndex;

    mPassData.updateLightPosition = false;
    mPassData.updateLightIntensity = false;

    mpPixelDebug->endFrame(pRenderContext);
}

void RTXDITutorial5::runSpatioTemporalReuse(RenderContext* pRenderContext, const RenderData& renderData)
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
        candidatesVars["SampleCB"]["gOutputReservoirIndex"] = uint(2);
        setupRTXDIBridgeVars(candidatesVars, renderData);
        mpPixelDebug->prepareProgram(mShader.initialCandidates->getProgram(), candidatesVars);
        mShader.initialCandidates->execute(pRenderContext, mPassData.screenSize.x, mPassData.screenSize.y);
    }

    // Step 2: (Optionally, but *highly* recommended) Test visibility for selected candidate.  
    if (mLightingParams.traceInitialShadowRay)
    {
        FALCOR_PROFILE("Candidate Visibility");
        uint checkCandiateVisMode = mEnableShadowRay ? uint(Visibility::AllShadowRay) : uint(mVisibility);
        checkCandiateVisMode = mOnlyUseIsmForTesting ? uint(Visibility::AllISM) : checkCandiateVisMode;

        auto visVars = mShader.initialCandidateVisibility->getRootVar();
        visVars["SampleCB"]["gReservoirIndex"] = uint(2);
        visVars["SampleCB"]["gVisMode"] = checkCandiateVisMode;
        setupRTXDIBridgeVars(visVars, renderData);
        mpPixelDebug->prepareProgram(mShader.initialCandidateVisibility->getProgram(), visVars);
        mShader.initialCandidateVisibility->execute(pRenderContext, mPassData.screenSize.x, mPassData.screenSize.y);
    }

    // How pixels' visibility is evaluated in previous pass
    if (mGpuDataToGet.getConverageData)
    {
        auto shadowOptionsResult = getDeviceResourceData<uint>(pRenderContext, mpShadowOptionsBuffer);
        calcDiffShadowCoverage(shadowOptionsResult, 0);
    }

    // Step 3: Do spatiotemporal reuse
    {
        FALCOR_PROFILE("Spatiotemporal Reuse");

        auto reuseVars = mShader.spatiotemporalReuse->getRootVar();
        reuseVars["ReuseCB"]["gCurFrameInputReservoirIndex"] = uint(2);
        reuseVars["ReuseCB"]["gPrevFrameInputReservoirIndex"] = uint(mLightingParams.lastFrameOutput);
        reuseVars["ReuseCB"]["gOutputReservoirIndex"] = uint(1u - mLightingParams.lastFrameOutput);
        reuseVars["ReuseCB"]["gMaxHistory"] = uint(mLightingParams.maxHistoryLength);
        reuseVars["ReuseCB"]["gBiasCorrectionMode"] = uint(mLightingParams.biasCorrectionMode);
        reuseVars["ReuseCB"]["gBoilingFilterStrength"] = (mLightingParams.useBoilFilter
            ? float(mLightingParams.boilFilterStrength) : 0.0f);
        reuseVars["ReuseCB"]["gDepthThreshold"] = float(mLightingParams.depthThreshold);
        reuseVars["ReuseCB"]["gNormalThreshold"] = float(mLightingParams.normalThreshold);
        reuseVars["ReuseCB"]["gReuseRadius"] = float(mLightingParams.spatialRadius);
        reuseVars["ReuseCB"]["gNumSamples"] = uint(mLightingParams.spatialSamples);
        reuseVars["ReuseCB"]["gSamplesInDisocclusions"] = uint(mLightingParams.spatialSamples);
        reuseVars["ReuseCB"]["gUseVisibilityShortcut"] = bool(mLightingParams.useVisibilityShortcut);
        reuseVars["ReuseCB"]["gEnablePermutationSampling"] = bool(mLightingParams.permuteTemporalSamples);
        setupRTXDIBridgeVars(reuseVars, renderData);
        mpPixelDebug->prepareProgram(mShader.spatiotemporalReuse->getProgram(), reuseVars);
        mShader.spatiotemporalReuse->execute(pRenderContext, mPassData.screenSize.x, mPassData.screenSize.y);
    }

    const auto& pOutColor = renderData["color"]->asTexture();
    pRenderContext->clearUAV(pOutColor->getUAV().get(), float4(float3(0.0f), 1.0f));

    // Step 4: Do final shading
    {
        FALCOR_PROFILE("Final shading");
        auto shadeVars = mShader.shade->getRootVar();
        shadeVars["ShadeCB"]["gInputReservoirIndex"] = uint(1u - mLightingParams.lastFrameOutput);
        shadeVars["ShadeCB"]["gShadingVisibility"] = mShadingVisibility;
        shadeVars["ShadeCB"]["gVisMode"] = mEnableShadowRay ? uint(Visibility::AllShadowRay) : uint(mVisibility);
        shadeVars["gOutputColor"] = renderData["color"]->asTexture();
        shadeVars["gInputEmission"] = mResources.emissiveColors;
        shadeVars["gVbuffer"] = renderData["vbuffer"]->asTexture();
        setupRTXDIBridgeVars(shadeVars, renderData);
        mpPixelDebug->prepareProgram(mShader.shade->getProgram(), shadeVars);
        mShader.shade->execute(pRenderContext, mPassData.screenSize.x, mPassData.screenSize.y);

        // Our "last frame" is now the one we just rendered; remember where we stored out output reservoir.
        // (Currently just toggling between 0 & 1 in alternate frames)
        mLightingParams.lastFrameOutput = 1u - mLightingParams.lastFrameOutput;
    }

    // How pixels' visibility is evaluated in previous pass
    if (mGpuDataToGet.getConverageData)
    {
        auto shadowOptionsResult = getDeviceResourceData<uint>(pRenderContext, mpShadowOptionsBuffer);
        calcDiffShadowCoverage(shadowOptionsResult, 1);
    }
}


// Renders the GUI used to change options on the fly when running in Mogwai.
void RTXDITutorial5::renderUI(Gui::Widgets& widget)
{
    // Provide controls for the number of samples on each light type (largely consistent options between tutorials)
    Gui::Group candidateOptions(widget.gui(), "Per-pixel light sampling", false);
    candidateOptions.text("Number of per-pixel light candidates on:");
    candidateOptions.tooltip(kSampleCountToolTip);
    candidateOptions.var("Triangles", mLightingParams.primLightSamples, 0u, 256u);
    if (candidateOptions.var("Environment", mLightingParams.envLightSamples, 0u, 256u))
        mPassData.clearReservoirs = true;
    if (candidateOptions.checkbox("Test selected candidate visibility?", mLightingParams.traceInitialShadowRay))
        mPassData.clearReservoirs = true;
    candidateOptions.tooltip(kCandidateVisibilityToolTip);
    candidateOptions.checkbox("Create per-tile light buffers?", mLightingParams.storePerTileLightGeom);
    candidateOptions.tooltip(kPerTileBuffersToolTip);
    candidateOptions.release();

    // Provide controllable options for each different rendering mode (e.g., each different tutorial)
    Gui::Group pipeOptions(widget.gui(), "Renderer Options", false);
    pipeOptions.dropdown("Bias correction", kBiasCorrection, mLightingParams.biasCorrectionMode);
    pipeOptions.var("Reuse radius", mLightingParams.spatialRadius, 0.0f, 50.0f, 0.1f);
    pipeOptions.var("History length", mLightingParams.maxHistoryLength, 1u, 30u, 1u);
    pipeOptions.var("Samples", mLightingParams.spatialSamples, 0u, 25u);
    pipeOptions.checkbox(mLightingParams.useBoilFilter ? "Boil filter strength:" : "Not using boil filter",
        mLightingParams.useBoilFilter);
    if (mLightingParams.useBoilFilter)
    {
        pipeOptions.var("##Boil Filter Stength", mLightingParams.boilFilterStrength, 0.0f, 1.0f, 0.001f, false);
    }
    pipeOptions.var("Depth Threshold", mLightingParams.depthThreshold, 0.0f, 1.0f, 0.001f);
    pipeOptions.var("Normal Threshold", mLightingParams.normalThreshold, 0.0f, 1.0f, 0.001f);
    pipeOptions.release();

    // RTXDI context parameters.  If any of these change, the RTXDI context needs to be rebuilt before the next use
    Gui::Group contextOptions(widget.gui(), "Precomputed light tile sizes", false);
    contextOptions.tooltip(kPrecomputedTileToolTip);
    if (contextOptions.var("Tile Count", mLightingParams.presampledTileCount, 1u, 1024u)) mpRtxdiContext = nullptr;
    if (contextOptions.var("Tile Size", mLightingParams.presampledTileSize, 8u, 8192u, 128u)) mpRtxdiContext = nullptr;
    contextOptions.release();

    RTXDITutorialBase::renderUI(widget);
}
