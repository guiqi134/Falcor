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
#include "RTXDICombinedTutorial.h"

 /** 
 */

using namespace Falcor;

// Name of the pass and a text description, as exposed in the "RTXDITutorials.dll" for use by Falcor.
const RenderPass::Info RTXDICombinedTutorial::getClassDescription()
{
    return { "RTXDICombinedTutorial", "Combined tutorial, allowing easy switching between various tutorial modes for comparison." };
}

// Our object factory, needed to expose instantiation of this RenderPass externally, outside the "RTXDITutorials.dll"
RTXDICombinedTutorial::SharedPtr RTXDICombinedTutorial::create(RenderContext* pRenderContext, const Dictionary& dict)
{
    return SharedPtr(new RTXDICombinedTutorial(dict));
}

RTXDICombinedTutorial::RTXDICombinedTutorial(const Dictionary& dict)
    : RTXDITutorial5(dict), RTXDITutorial4(dict), RTXDITutorial3(dict), RTXDITutorial2(dict), RTXDITutorial1(dict),
    RTXDITutorialBase(dict, getClassDescription())
{
    // Why do this?  Using 1 spatial sample is typically best.  However, Tutorial 3 overrode this setting, 
    // as for spatial-only reuse, using more spatial samples typically gives better quality than using just one.
    // Here we just set the value to the default that works best in most cases.
    mLightingParams.spatialSamples = 1u;
}

void RTXDICombinedTutorial::execute(RenderContext* pRenderContext, const RenderData& renderData)
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

    // Do z-buffer raster pass
    runZBufferRaster(pRenderContext, renderData);

    // Do resampling with one of our RTXDI pipelines
    switch (mLightingParams.selectedDisplayPipeline)
    {
    case kMonteCarloBaselineMode:
        runMonteCarloLighting(pRenderContext, renderData);
        break;
    case kTalbotRISMode: 
        runBasicRISLighting(pRenderContext, renderData);
        break;
    case kSpatialOnlyReuseMode: 
        runSpatialReuseOnly(pRenderContext, renderData);
        break;
    case kTemporalOnlyReuseMode:
        runTemporalReuseOnly(pRenderContext, renderData);
        break;
    default:
    case kSpatiotemporalReuseMode: 
        runSpatioTemporalReuse(pRenderContext, renderData);
        break;
    }

    // Increment our frame counter for next frame.  This is used to seed a RNG, which we want to change each frame 
    if (!mFrozenFrame) mRtxdiFrameParams.frameIndex++;

    // When we do temporal reuse, we need a G-buffer for this frame *and* last frame to compute shading.  We have
    // two G-buffer indicies (0 and 1), ping-pong back and forth between them each frame.
    mLightingParams.currentGBufferIndex = 1u - mLightingParams.currentGBufferIndex;
    mLightingParams.priorGBufferIndex = 1u - mLightingParams.priorGBufferIndex;

    // Store this frame z-buffer as previous z-buffer
    mSSRTResources.prevZBufferTexture = mSSRTResources.currZBufferTexture;

    mpPixelDebug->endFrame(pRenderContext);
}

// Renders the GUI used to change options on the fly when running in Mogwai.
void RTXDICombinedTutorial::renderUI(Gui::Widgets& widget)
{
    // Allow users to choose which of the premade RTXDI pipelines to run.
    widget.text("Renderer:");
    if (widget.dropdown("##Sample pipeline", kDropList, mLightingParams.selectedDisplayPipeline, true))
        mPassData.clearReservoirs = true;

    // Rather than reconsturcting the GUI for all these different tutorials, let's just display their GUI.
    switch (mLightingParams.selectedDisplayPipeline)
    {
    case kMonteCarloBaselineMode: 
        RTXDITutorial1::renderUI(widget);
        break;
    case kTalbotRISMode: 
        RTXDITutorial2::renderUI(widget);
        break;
    case kSpatialOnlyReuseMode: 
        RTXDITutorial3::renderUI(widget);
        break;
    case kTemporalOnlyReuseMode: 
        RTXDITutorial4::renderUI(widget);
        break;
    default:
    case kSpatiotemporalReuseMode:
        RTXDITutorial5::renderUI(widget);
        break;
    }
}
