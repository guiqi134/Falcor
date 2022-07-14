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
#include "RTXDITutorial1.h"

 /** 
 */

using namespace Falcor;

// Name of the pass and a text description, as exposed in the "RTXDITutorials.dll" for use by Falcor.
RenderPass::Info RTXDITutorial1::getClassDescription()
{
    return { "RTXDITutorial1", "Basic Monte Carlo Integration using RTXDI data structures." };
}

// Our object factory, needed to expose instantiation of this RenderPass externally, outside the "RTXDITutorials.dll"
RTXDITutorial1::SharedPtr RTXDITutorial1::create(RenderContext* pRenderContext, const Dictionary& dict)
{
    return SharedPtr(new RTXDITutorial1(dict));
}


void RTXDITutorial1::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    mpPixelDebug->beginFrame(pRenderContext, mPassData.screenSize);

    // Run base execute method
    RTXDITutorialBase::execute(pRenderContext, renderData);

    // The rest of the rendering code in this pass may fail if there's no scene loaded
    if (!mPassData.scene) return;

    // If we haven't loaded our shaders yet, go ahead and load them.
    if (!mShader.shade) loadShaders();

    // If needed, allocate an RTXDI context & resources so we can run our lighting code
    if (!mpRtxdiContext) allocateRtxdiResrouces(pRenderContext, renderData);

    // See what, if anything, changed since last frame.  Changes might indicate we need to update
    // data passed to RTXDI.  This queries our Falcor scene to update flags that track changes.
    checkForSceneUpdates();

    // Convert our input (standard) Falcor v-buffer into a packed G-buffer format.  This isn't really
    // important in this simple Monte Carlo sample, but it's vital for performance for more general
    // RTXDI use.  It creates a cache-coherent buffer of hits & material properties to reduce memory
    // divergence when resampling for spatiotemporal reuse. 
    prepareSurfaceData(pRenderContext, renderData);

    // Do z-buffer raster pass
    runZBufferRaster(pRenderContext, renderData);

    // For each pixel in our image, randomly select some number of points on our lights, send shadow
    // rays to each one, and accumulate lighting using standard Monte Carlo integration (i.e., no fancy
    // RIS or RTXDI integration).
    runMonteCarloLighting(pRenderContext, renderData);

    // Increment our frame counter for next frame.  This is used to seed a RNG, which we want to change each frame 
    if (!mFrozenFrame) mRtxdiFrameParams.frameIndex++;

    mpPixelDebug->endFrame(pRenderContext);
}

void RTXDITutorial1::runMonteCarloLighting(RenderContext* pRenderContext, const RenderData& renderData)
{
    // Set our RTXDI parameters for the current frame.
    //  -> This calls RTXDI's rtxdi::FillRuntimeParameters() routine.
    setCurrentFrameRTXDIParameters(renderData);

    // (Optional) Create a PDF texture for our primitive lights and environment map
    if (mLightingParams.initialLightPdfMode != kUniformSamplingMode)
        computePDFTextures(pRenderContext, renderData);

    // (Optional) Create tiles of presampled lights once per frame to improve per-pixel memory coherence.
    // See:  Wyman & Panteleev, "Rearchitecting Spatiotemporal Resampling for Production," HPG 2021.
    if (mLightingParams.initialLightPdfMode == kPrecomputedTileSamplingMode)
        presampleLights(pRenderContext, renderData);

    // Do our Monte Carlo integration
    {
        FALCOR_PROFILE("Monte Carlo Integration");
        auto mcVars = mShader.monteCarloBaseline->getRootVar();
        mcVars["CB"]["gLightSamplingMode"] = mLightingParams.initialLightPdfMode;  // Sampling mode to use to pick our MC samples
        mcVars["CB"]["gNumTriSamples"] = mLightingParams.primLightSamples;         // Number of MC samples on emissive triangles
        mcVars["CB"]["gNumEnvSamples"] = mLightingParams.envLightSamples;          // Number of MC samples on environment map
        mcVars["gOutputColor"] = renderData["color"]->asTexture();                 // Per-pixel output color goes here
        mcVars["gInputEmission"] = mResources.emissiveColors;                      // Input from prepareSurfaceData() -- contains directly seen emissives
        setupRTXDIBridgeVars(mcVars, renderData);                                  // Setup data structures needed by RTXDI shader routines
        setupSSRTVars(mcVars, renderData, false);
        mpPixelDebug->prepareProgram(mShader.monteCarloBaseline->getProgram(), mcVars);
        mShader.monteCarloBaseline->execute(pRenderContext, mPassData.screenSize.x, mPassData.screenSize.y);
    }
}


// Renders the GUI used to change options on the fly when running in Mogwai.
void RTXDITutorial1::renderUI(Gui::Widgets& widget)
{
    // Provide controls for the number of samples on each light type (largely consistent options between tutorials)
    Gui::Group candidateOptions(widget.gui(), "Per-pixel light sampling", true);
    candidateOptions.text("Number of per-pixel light candidates on:");
    candidateOptions.tooltip(kNaiveSampleToolTip);
    candidateOptions.var("Triangles", mLightingParams.primLightSamples, 0u, 256u);
    candidateOptions.var("Environment", mLightingParams.envLightSamples, 0u, 256u);
    candidateOptions.release();

    // Provide controllable options for each different rendering mode (e.g., each different tutorial)
    Gui::Group pipeOptions(widget.gui(), "Renderer Options", true);
    pipeOptions.text("Light sampling technique to use:");
    pipeOptions.tooltip(kSamplingTypeToolTip);
    pipeOptions.radioButtons(kSampleList, mLightingParams.initialLightPdfMode);
    pipeOptions.release();

    // RTXDI context parameters.  If any of these change, the RTXDI context needs to be rebuilt before the next use
    if (mLightingParams.initialLightPdfMode == kPrecomputedTileSamplingMode)
    {
        Gui::Group contextOptions(widget.gui(), "Precomputed light tile sizes", true);
        contextOptions.tooltip(kPrecomputedTileToolTip);
        if (contextOptions.var("Tile Count", mLightingParams.presampledTileCount, 1u, 1024u)) mpRtxdiContext = nullptr;
        if (contextOptions.var("Tile Size", mLightingParams.presampledTileSize, 256u, 8192u, 128u)) mpRtxdiContext = nullptr;
        contextOptions.release();
    }

    RTXDITutorialBase::renderUI(widget);
}
