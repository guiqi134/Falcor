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
#include "RTXDITutorialBase.h"
#include "RTXDITutorialConstants.h"

 /** This RenderPass has common resources and methods needed by all our tutorials.

     Please do not start reading this code!

     Read the tutorial passes, and refer to specific methods below as needed by the
     tutorial steps.  This will help clarify that you don't need to start an
     integration with a 500+ line complete base class (like this), but can start
     with a (relatively) smaller set of methods.
 */

using namespace Falcor;

namespace
{
    // Some shortcuts for common resource access patterns, used when initializing Falcor buffers and textures.

    // In our tutorials, we use Falcor buffers as read-write objects
    const Resource::BindFlags kBufferBindFlags =
        Resource::BindFlags::ShaderResource |        // Allows reading a buffer via a Buffer<> object in HLSL
        Resource::BindFlags::UnorderedAccess;        // Allows reading/writing a buffer via a RWBuffer<> object in HLSL

    // In our tutorials, we use Falcor's automatic mipmap generation, which requires the following access types
    const Resource::BindFlags kAutoMipBindFlags =
        Resource::BindFlags::ShaderResource |        // Allows using the mipmapped texture as a Texture2D in HLSL
        Resource::BindFlags::UnorderedAccess |       // Allows writing the base mip layer as a RWTexture2D in HLSL
        Resource::BindFlags::RenderTarget;           // Needed for the automatic mipmap generation

    const Resource::BindFlags kShadowMapFlags = Resource::BindFlags::ShaderResource | Resource::BindFlags::DepthStencil;
    const Resource::BindFlags kTextureCubeFlags = Resource::BindFlags::ShaderResource | Resource::BindFlags::RenderTarget;

    const std::string kOutputFileDirectory = "D:/ReSTIR/Data";

    const uint kShadowMapsPerLight = 6u;

    // These should be evenly divisble by mLightTopN.
    //const uint kMaxLightsPerTexArray = 340u;
    //const uint kMaxLightsPerTexArray1024 = 168u;
    //const uint kMaxSupportedLights1024 = 336;

    const uint kMaxIsmCount = 1024;

};


/** Our render pass constructor checks if the Python script which specifies a render graph has provided
    and specific settings we should respect.  The keys below are optional parameters the tutorial passes
    respond to from these Python scripts.
*/
RTXDITutorialBase::RTXDITutorialBase(const Dictionary& dict, const RenderPass::Info& desc)
    : RenderPass(desc)
{
    for (const auto& [key, value] : dict)
    {
        if (key == "useLowerShininess")        mLightingParams.useHigherShininess = !bool(value);
        else if (key == "triEmissiveScale")    mLightingParams.relativeTriangleWeight = float(value);
        else if (key == "envEmissiveScale")    mLightingParams.relativeEnvMapWeight = float(value);
        else if (key == "epsilon")             mLightingParams.epsilon = float(value);
        else if (key == "compressLightTiles")  mLightingParams.storePerTileLightGeom = bool(value);
        else if (key == "ismMipLevels") mIsmMipLevels = uint(value);
        else if (key == "smDepthBias") mSmDepthBias = float(value);
        else if (key == "ismDepthBias") mIsmDepthBias = float(value);
        else if (key == "ismPushMode") mIsmPushMode = uint(value);
        else if (key == "baseTriangleSize") mBaseTriSize = float(value);
        else if (key == "sceneName") mSceneName = uint(value);
        else if (key == "adaptiveLightNear") mAdaptiveLightNearPlane = bool(value);
        else logWarning("Unknown field '" + key + "' in RTXDITutorialBase dictionary");
    }

    mpPixelDebug = PixelDebug::create();
    mpFence = GpuFence::create();

    // Create samplers
    Sampler::Desc samplerDesc;
    samplerDesc.setAddressingMode(Sampler::AddressMode::Clamp, Sampler::AddressMode::Clamp, Sampler::AddressMode::Clamp);
    samplerDesc.setFilterMode(Sampler::Filter::Point, Sampler::Filter::Point, Sampler::Filter::Point);
    mpPointSampler = Sampler::create(samplerDesc);

    samplerDesc.setFilterMode(Sampler::Filter::Linear, Sampler::Filter::Linear, Sampler::Filter::Linear);
    mpLinearSampler = Sampler::create(samplerDesc);

    // Containers
    mValuesToPrint.shadowOptionsCoverage1.resize(6);
    mValuesToPrint.shadowOptionsCoverage2.resize(6);

}


/** Falcor render passes are just one part of a renderer.  Our RTXDI tutorials assume they get connected up
    to a V-buffer (see https://jcgt.org/published/0002/02/04/).  Falcor provides a standard V-buffer pass
    in both ray traced and raster variance (these are located in the GBuffer render pass, which gets compiled
    into GBuffer.dll).  Our tutorials use this standard pass.

    When hooking passes together, we need to tell Mogwai (which executes sequences of render graphs) what
    resources we expect from prior passes and what output resources we provide for later passes to consume.
    This specification happens in the reflect() callback, which is used to validate that all expected resources
    have been provided for correct execution.
*/
RenderPassReflection RTXDITutorialBase::reflect(const CompileData& compileData)
{
    RenderPassReflection reflector;

    // What buffers does this render pass expect as inputs from prior render passes?
    reflector.addInput("vbuffer", "");    // We expect an input V-Buffer, allowing look ups of surface data at primary hits
    reflector.addInput("mvec", "");       // The V-buffer pass should also give us a texture with motion vectors

    // This pass outputs a shaded color for display or for other passes to run postprocessing
    reflector.addOutput("color", "").format(ResourceFormat::RGBA16Float);
    reflector.addOutput("debugSM", "").format(ResourceFormat::RGBA32Float);
    reflector.addOutput("wireframe", "").format(ResourceFormat::RGBA32Float);

    // debug ISMs
    reflector.addOutput("wireframe", "").format(ResourceFormat::RGBA32Float);
    reflector.addOutput("ism after push", "").texture2D(mIsmSize, mIsmSize).format(ResourceFormat::RGBA32Float);
    reflector.addOutput("ism after pull", "").texture2D(mIsmSize, mIsmSize).format(ResourceFormat::RGBA32Float);
    reflector.addOutput("all ism 2k", "").texture2D(mIsmVisTextureSize, mIsmVisTextureSize).format(ResourceFormat::RGBA32Float);
    reflector.addOutput("all ism 4k", "").texture2D(mIsmVisTextureSize * 2u, mIsmVisTextureSize * 2u).format(ResourceFormat::RGBA32Float);
    reflector.addOutput("all ism 8k", "").texture2D(mIsmVisTextureSize * 4u, mIsmVisTextureSize * 4u).format(ResourceFormat::RGBA32Float);
    reflector.addOutput("all ism 14k", "").texture2D(mIsmVisTextureSize * 7u, mIsmVisTextureSize * 7u).format(ResourceFormat::RGBA32Float);


    return reflector;
}


/** Falcor "recompiles" the render graph whenever the window size changes.  Recompilation happens other times,
    but we're using this callback solely to capture window resize events, after which we need to reinitialize
    the RTXDI context.
*/
void RTXDITutorialBase::compile(RenderContext* pContext, const CompileData& compileData)
{
    // If our screen resolution has changed, we need to remake our RTXDI context
    if (mPassData.screenSize != compileData.defaultTexDims)
    {
        mpRtxdiContext = nullptr;
        mPassData.screenSize = compileData.defaultTexDims;
    }
}

// Updating functions. TODO: create a new function for this
void RTXDITutorialBase::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    auto& dict = renderData.getDictionary();

    if (bool(mUpdates & UpdateFlags::ResourcesChanged) || bool(mUpdates & UpdateFlags::VisibilityChanged))
    {
        logInfo("Update resources");

        // Auto reset accumulation
        auto flags = dict.getValue(kRenderPassRefreshFlags, Falcor::RenderPassRefreshFlags::None);
        flags |= Falcor::RenderPassRefreshFlags::RenderOptionsChanged;
        dict[Falcor::kRenderPassRefreshFlags] = flags;

        // Update parameters
        //uint totalIsmLights = mVisibility == Visibility::ShadowMap_ISM ? mTotalLightsCount - (mLightTopN * mTemporalReusingLength) : mTotalLightsCount;
        mTotalIsmCount = std::min(kMaxIsmCount, mTotalLightsCount * mIsmPerLight);
        mCurrFrameLightStartIdx = 0;
        mRtxdiFrameParams.frameIndex = 0;

        // Update all resources
        mpReusingLightIndexBuffer = Buffer::createTyped<uint>(mLightTopN * mTemporalReusingLength, kBufferBindFlags, Buffer::CpuAccess::Read);
        pRenderContext->clearUAV(mpReusingLightIndexBuffer->getUAV().get(), uint4(-1));
        mpIsmTextureArray = Texture::create2D(mIsmSize, mIsmSize, ResourceFormat::R32Float, mTotalIsmCount, mIsmMipLevels, nullptr, kAutoMipBindFlags);
        allocateShadowTextureArrays();

        // Clear light mesh data buffer. TODO: use computer shader to clear
        prepareLightShadowMapData();

        if (mVisibility == Visibility::BaselineSM)
        {
            mpHeroLightShadowMapsTexture = Texture::create2D(mShadowMapSize, mShadowMapSize, ResourceFormat::R32Float, kShadowMapsPerLight * mLightTopN,
                1, nullptr, kAutoMipBindFlags);

            // Clear light mesh data buffer. TODO: use computer shader to clear
            prepareLightShadowMapData();
        }
    }

    if (bool(mUpdates & UpdateFlags::ShaderDefinesChanged))
    {
        mIsmRenderPass.pProgram->addDefine("GS_OUT_STREAM", std::to_string(mIsmSceneType));
    }

    mUpdates = UpdateFlags::None;
}

void RTXDITutorialBase::renderUI(Gui::Widgets& widget)
{
    bool resourcesChanged = false;
    bool definesChanged = false;

    Gui::DropdownList ismPushModes = {
        {0, "Point"}, {1, "Linear"}
    };
    Gui::DropdownList ismSceneTypes = {
        {0, "Triangle"}, {1, "Three Points"}, {2, "Center Point"}
    };
    Gui::DropdownList ismSizes = {
        {32, "32"}, {64, "64"}, {128, "128"}, {256, "256"}, {512, "512"}
    };
    Gui::DropdownList sortingRules = {
        {0, "All Pixels"}, {1, "Occluded Pixels"}, {2, "Shadow Edge Pixels"}
    };
    Gui::DropdownList depthBiasMode = {
        {0, "Constant"}, {1, "Slope Scale"}, {2, "Dou2014"}
    };

    // Global parameters
    widget.dropdown("Adaptive Shadow Map Size Heuristic", sortingRules, mSortingRules);
    widget.dropdown("Shadow Depth Bias Mode", depthBiasMode, mShadowDepthBias);
    resourcesChanged |= widget.var("Number of Top N light meshes", mLightTopN);
    resourcesChanged |= widget.var("Temporal Reusing Length", mTemporalReusingLength);
    widget.var("Updating Frequency (in frames)", mUpdatingFrequency);
    widget.var("Cubic Shadow Map Depth Bias", mSmDepthBias);
    widget.var("Debug Light Mesh ID", mDebugLightMeshID);
    widget.checkbox("Adaptive ISM", mAdaptiveISM);
    widget.checkbox("Only Use ISM for Testing Candidates?", mOnlyUseIsmForTesting);
    widget.checkbox("Enable Debug Light?", mDebugLight);

    widget.checkbox("Full Size Shadow Maps?", mFullSizeShadowMaps);
    resourcesChanged |= widget.var("Light Near and Far Plane", mLightNearFarPlane);
    widget.checkbox("Enable Shadow Ray?", mShadowRayForRestPixels);
    widget.checkbox("Final Shading Visibility?", mShadingVisibility);
    widget.checkbox("Draw Wireframe?", mDrawWireframe);
    resourcesChanged |= widget.checkbox("Use Adaptive Light Near Plane?", mAdaptiveLightNearPlane);

    // ISM parameters
    Gui::Group ismParams(widget.gui(), "ISM Parameters", true);
    ismParams.var("ISM Light Sampling Mode", mIsmLightSamplingMode);
    ismParams.var("ISM Depth Bias", mIsmDepthBias);
    resourcesChanged |= ismParams.var("ISM Pull&Push Mip Levels", mIsmMipLevels);
    ismParams.dropdown("ISM Push Mode", ismPushModes, mIsmPushMode);
    definesChanged |= ismParams.dropdown("ISM Scene Type", ismSceneTypes, mIsmSceneType);
    resourcesChanged |= ismParams.dropdown("ISM Size", ismSizes, mIsmSize);
    ismParams.var("Scene depth threshold scale", mSceneDepthThresholdScale);
    ismParams.var("Base Triangle Size", mBaseTriSize);

    ismParams.var("Mip level to visualize", mVisualizeMipLevel);
    ismParams.var("Light's ISM to visualize", mVisLightFaceID);
    ismParams.release();

    // GPU data
    Gui::Group gpuData(widget.gui(), "Print GPU Data", true);
    gpuData.checkbox("Get Shadow Map Size Data?", mGpuDataToGet.getShadowMapSizeData);
    gpuData.checkbox("Get Light Sorting Data?", mGpuDataToGet.getSortingData);
    gpuData.checkbox("Get ISM Data?", mGpuDataToGet.getIsmData);
    gpuData.checkbox("Get Converage Data?", mGpuDataToGet.getConverageData);
    gpuData.release();


    // TODO: use string stream to output
    std::ostringstream oss;

    Gui::Group printValues(widget.gui(), "Printing", true);
    printValues.text(std::format("Current frame: {}", mRtxdiFrameParams.frameIndex));
    printValues.text(std::format("Total mesh lights: {}, Total point lights: {}", mTotalLightMeshCount, mTotalPointLightsCount));
    printValues.text(std::format("Total ISM Count: {}", mTotalIsmCount));
    printValues.text(std::format("Reusing shadow maps count: {}", mLightTopN * mTemporalReusingLength * kShadowMapsPerLight));
    printValues.text(std::format("Total shadow map textures size: {}", formatByteSize(mValuesToPrint.totalShadowTexSize)));
    printValues.text(std::format("ISM texture array memory: {}", formatByteSize(mpIsmTextureArray->getTextureSizeInBytes())));

    if (mGpuDataToGet.getShadowMapSizeData)
    {
        printValues.text("");
        printValues.text(mValuesToPrint.shadowMapSizeCount);
    }

    if (mGpuDataToGet.getIsmData)
    {
        printValues.text("");
        printValues.text(std::format("Total Extra points generated for ISM: {}", mValuesToPrint.totalExtraPoints));
    }

    if (mGpuDataToGet.getSortingData)
    {
        printValues.text("");
        printValues.text(std::format("Sorted light index buffer each frame: \n[{}]", mValuesToPrint.sortedLightIndexArray));
        printValues.text(std::format("Reusing light index buffer: \n[{}]", mValuesToPrint.reusingLightIndexArray));
        printValues.text(std::format("ISM light index buffer: \n[{}]", mValuesToPrint.ismLightIndexArray));
    }

    if (mGpuDataToGet.getConverageData)
    {
        printValues.text("");
        printValues.text("In test candidate visiblity pass");
        printValues.text(std::format("Invalid pixel coverage: {:.3f}", mValuesToPrint.shadowOptionsCoverage1[0]));
        printValues.text(std::format("Fully lit pixel coverage: {:.3f}", mValuesToPrint.shadowOptionsCoverage1[1]));
        printValues.text(std::format("Shadow map pixel coverage: {:.3f}", mValuesToPrint.shadowOptionsCoverage1[2]));
        printValues.text(std::format("ISM pixel coverage: {:.3f}", mValuesToPrint.shadowOptionsCoverage1[3]));
        printValues.text(std::format("Shadow ray pixel coverage: {:.3f}", mValuesToPrint.shadowOptionsCoverage1[4]));
        printValues.text("");

        printValues.text("In final shading pass");
        printValues.text(std::format("Invalid pixel coverage: {:.3f}", mValuesToPrint.shadowOptionsCoverage2[0]));
        printValues.text(std::format("Fully lit pixel coverage: {:.3f}", mValuesToPrint.shadowOptionsCoverage2[1]));
        printValues.text(std::format("Shadow map pixel coverage: {:.3f}", mValuesToPrint.shadowOptionsCoverage2[2]));
        printValues.text(std::format("ISM pixel coverage: {:.3f}", mValuesToPrint.shadowOptionsCoverage2[3]));
        printValues.text(std::format("Shadow ray pixel coverage: {:.3f}", mValuesToPrint.shadowOptionsCoverage2[4]));
    }

    printValues.release();

    mpPixelDebug->renderUI(widget);

    if (resourcesChanged) mUpdates |= UpdateFlags::ResourcesChanged;
    if (definesChanged) mUpdates |= UpdateFlags::ShaderDefinesChanged;
}


/** Updates flags inside this base class to (selectively/lazily) rebuild structures due to dynamic scene changes
*/
void RTXDITutorialBase::checkForSceneUpdates()
{
    // TODO: Make more granular / flexible; need to update Falcor UpdateFlags to allow more granular queries.
    //   For instance Scene::UpdateFlags::MaterialsChanged checks if any scene material has changed in any way,
    //   but for RTXDI, we really only care if a material's *emissivity* has changed.

    // Determine what, if anything happened since last frame.
    Scene::UpdateFlags updates = mPassData.scene->getUpdates();
    if (bool(updates & Scene::UpdateFlags::LightCollectionChanged))  mPassData.updateEmissiveTriangleGeom = true;
    if (bool(updates & Scene::UpdateFlags::MaterialsChanged))        mPassData.updateEmissiveTriangleFlux = true;
    if (bool(updates & Scene::UpdateFlags::EnvMapChanged))           mPassData.updateEnvironmentMapPdf = true;
    if (bool(updates & Scene::UpdateFlags::LightsMoved)) mPassData.updateLightPosition = true;
    if (bool(updates & Scene::UpdateFlags::LightIntensityChanged)) mPassData.updateLightIntensity = true;
    if (bool(updates & Scene::UpdateFlags::LightPropertiesChanged)) mPassData.updateLightOtherProperties = true;
}


/** When Falcor/Mogwai load a scene, this callback gets called.  We stash the new scene and set flags to
    rebuild the RTXDI context and all the internal light structures.
*/
void RTXDITutorialBase::setScene(RenderContext* pContext, const Scene::SharedPtr& pScene)
{
    // Callback for when Mogwai detects we have a new scene.  Stash it.
    mPassData.scene = pScene;

    // Force RTXDI to recreate its context, just to make sure we size resources correctly.
    mpRtxdiContext = nullptr;

    // If we successfully loaded a scene, tell Falcor to build a collection of emissives for the scene.
    if (mPassData.scene)
    {
        mPassData.lights = mPassData.scene->getLightCollection(pContext);
        mPassData.updateEnvironmentMapPdf = true;
        mPassData.updateEmissiveTriangleFlux = true;
        mPassData.updateEmissiveTriangleGeom = true;

        // Get and count all the point lights
        auto lights = mPassData.scene->getLights();
        for (uint i = 0; i < mPassData.scene->getLightCount(); i++)
        {
            auto pointLight = dynamic_cast<PointLight*>(lights[i].get());
            if (pointLight != nullptr)
            {
                mPointLights.push_back(pointLight);
                mTotalPointLightsCount++;
            }
        }
        logInfo(std::format("Total point light count = {}", mTotalPointLightsCount));

        // Get the light mesh count
        mTotalLightMeshCount = (uint)mPassData.lights->getMeshLights().size();
        logInfo(std::format("Total Light Mesh Count = {}", mTotalLightMeshCount));

        // Adjust the number of top N lights
        mTotalLightsCount = mTotalLightMeshCount + mTotalPointLightsCount;
        mLightTopN = std::min(mLightTopN, mTotalLightsCount);

        // Compute the total ISMs
        mTotalIsmCount = std::min(kMaxIsmCount, mIsmPerLight * mTotalLightsCount);

        // Create emissive light sampler
        //mpEmissiveSampler = EmissivePowerSampler::create(pContext, pScene);
        mpEmissiveSampler = EmissiveUniformSampler::create(pContext, pScene);

        // Get the max extend of the scene
        float3 sceneExtend = mPassData.scene->getSceneBounds().extent();
        float maxExtend = std::max(std::max(sceneExtend.x, sceneExtend.y), sceneExtend.z);
        mLightNearFarPlane.y = std::min(mLightNearFarPlane.y, maxExtend);
        mDepthThreshold = mSceneDepthThresholdScale * (maxExtend - mLightNearFarPlane.x) / (mLightNearFarPlane.y - mLightNearFarPlane.x);
        mConstEpsilonForAdaptiveDepthBias = glm::length(sceneExtend) * 0.0001f;
        //if (mRtxdiFrameParams.frameIndex == 50)
        //    logInfo(std::format("Scene extend = ({}, {}, {}), depth threshold = {}", sceneExtend.x, sceneExtend.y, sceneExtend.z, depthThreshold));

        mSortedLightsShadowMaps.resize(mLightTopN * mTemporalReusingLength);


        if (mPassData.scene->getMeshCount() <= 1 << 16)
        {
            // Get scene's packed static vertex data
            auto vertexBuffer = mPassData.scene->getMeshVao16()->getVertexBuffer(0);
            auto indexBuffer = mPassData.scene->getMeshVao16()->getIndexBuffer();
        }

    }
}

bool RTXDITutorialBase::onKeyEvent(const KeyboardEvent& keyEvent)
{
    if (keyEvent.type == KeyboardEvent::Type::KeyPressed && keyEvent.key == Input::Key::Key1)
    {
        mVisibility = Visibility::AllShadowRay;
        mUpdates |= UpdateFlags::VisibilityChanged;
        return true;
    }

    if (keyEvent.type == KeyboardEvent::Type::KeyPressed && keyEvent.key == Input::Key::Key2)
    {
        mVisibility = Visibility::ShadowMap_FullyLit;
        mUpdates |= UpdateFlags::VisibilityChanged;
        return true;
    }

    if (keyEvent.type == KeyboardEvent::Type::KeyPressed && keyEvent.key == Input::Key::Key3)
    {
        mVisibility = Visibility::ShadowMap_ISM;
        mUpdates |= UpdateFlags::VisibilityChanged;
        return true;
    }

    if (keyEvent.type == KeyboardEvent::Type::KeyPressed && keyEvent.key == Input::Key::Key4)
    {
        mVisibility = Visibility::AllISM;
        mUpdates |= UpdateFlags::VisibilityChanged;
        return true;
    }

    return false;
}

/** When RTXDI reuses spatial neighbors, it is _not_ a dense reuse (e.g., of all pixels in a 5x5 regions).

    Reuse is instead stochastic, it reuses some random subset of pixels within a specified radius of the
    current pixel.

    But for various reasons, you don't want _pure_ random reuse with white noise samples.  RTXDI provides
    a function that fills a buffer with reasonably distributed samples (in roughly a poisson / blue-noise
    density).  You could, instead, fill this buffer with your own distribution.  This buffer is passed
    to the RTXDI bridge, which draws samples from this during spatial reuse.

    When starting an integration, start with this distribution.  It's likely you'll never change it.
*/
bool RTXDITutorialBase::updateNeighborBuffer()
{
    // Don't reallocate this; it's really a one-size-fits-all setting and buffer.
    if (!mResources.neighborOffsetBuffer)
    {
        // Create a CPU array of offsets; ask RTXDI to fill it with the neighbor offsets it needs
        std::vector<uint8_t> offsets(2 * size_t(mRtxdiContextParams.NeighborOffsetCount));
        mpRtxdiContext->FillNeighborOffsetBuffer(offsets.data());

        // Create a GPU buffer, initializing it with the CPU array we just created
        mResources.neighborOffsetBuffer = Buffer::createTyped(ResourceFormat::RG8Snorm, mRtxdiContextParams.NeighborOffsetCount,
            kBufferBindFlags, Buffer::CpuAccess::None, static_cast<void*>(offsets.data()));
    }
    return (mResources.neighborOffsetBuffer != nullptr);
}

/** This executes a shader that converts the input V-buffer (see the comment on reflect() method)
    into a packed G-buffer format that is efficient to use during RTXDI execution.  This is important,
    since access to your engine's material properties at surface hit points may not be particularly
    efficient (or, more importantly, memory coherent).  During reuse in RTXDI, we re-evaluate shading
    at hits _many_ times per pixel, so we need loading material properties to be memory coherent.

    This is common code called by all our tutorials.  The contents of the executed shader are not
    particularly important, as long as you can replicate a (similar) output from your engine.
*/
void RTXDITutorialBase::prepareSurfaceData(RenderContext* pContext, const RenderData& data)
{
    // This kernel repacks the v-buffer data and stores it in mResources.gBufferData.  This occurs in the
    // RTXDI_PrepareSurfaceData.cs.slang kernel.  The packed format is 128 bits (one float4), with
    // packed surface normal (32-bits), packed diffuse reflectance (24-bits), relative weight of the diffuse
    // lobe (8-bits), packed specular reflectance (24-bits), surface roughness (8-bits), and distance from
    // camera to the hitpoint (32-bits).  The exact terms needed may depend on the material model you use
    // during resampling in the RTXDI bridge.  Our format is a decent place to start.

    FALCOR_PROFILE("Format Surface Hit Data");
    auto prepSurfaceVars = mShader.prepareSurfaceData->getRootVar();
    prepSurfaceVars["CB"]["gScreenRes"] = mPassData.screenSize;
    prepSurfaceVars["CB"]["gEnvironmentLightPresent"] = bool(mPassData.scene->getEnvMap() != nullptr);
    prepSurfaceVars["CB"]["gRelativeTrianglePower"] = float(mLightingParams.relativeTriangleWeight);
    prepSurfaceVars["CB"]["gRelativeEnvironmentPower"] = float(mLightingParams.relativeEnvMapWeight);
    prepSurfaceVars["CB"]["gUsePrimaryHitTextureLoD"] = mLightingParams.usePrimaryLoD;
    prepSurfaceVars["CB"]["gGBufferOffset"] = uint32_t(mLightingParams.currentGBufferIndex * mPassData.screenSize.x * mPassData.screenSize.y);
    prepSurfaceVars["gVbuffer"] = data["vbuffer"]->asTexture();        // Our input V-buffer
    prepSurfaceVars["gOutputGeom"] = mResources.gBufferData;           // Our output packed surface data (G-buffer)
    prepSurfaceVars["gOutputEmissives"] = mResources.emissiveColors;   // Full-screen image with emission from directly-visible geometry (and env map)
    mShader.prepareSurfaceData->execute(pContext, mPassData.screenSize.x, mPassData.screenSize.y);
}

void RTXDITutorialBase::computePDFTextures(RenderContext* pContext, const RenderData& data)
{
    FALCOR_PROFILE("Create PDF Texure");

    /* Create the PDF texture for primitive lights.  Right now, this is only triangles,
       but it could include other primitives (spheres, points, rects, cylinders, etc.)
    */
    {
        // This shader executes one thread per primitive light.  This means the kernel launch
        // doesn't have a natural width & height.  We split it into a 2D kernel launch, as 1D launches
        // have a maximum size (i.e., max # of lights).  Our 2D "width" is a arbitrary size of 8192,
        // with the height dependent on the number of lights in the scene.  Optimal sizing has not
        // been explored.
        //  -> it will have lots of dummy thread, but essentially it compute each light's weight to a n*n PDF texture
        uint launchWidth = 8192u;
        uint launchHeight = (mRtxdiFrameParams.numLocalLights / launchWidth) + 1;

        // This shader also (optionally) repacks Falcor lights into a more cache coherent format,
        // in addition to creating the pdf texture for use in initial candidate sampling.
        auto pdfVars = mShader.createLightResources->getRootVar();
        pdfVars["CB"]["gLaunchWidth"] = launchWidth;
        pdfVars["CB"]["gTotalPrimitives"] = mRtxdiFrameParams.numLocalLights;
        pdfVars["CB"]["gTotalTriPrimitives"] = mPassData.lights->getActiveLightCount();
        pdfVars["CB"]["gUpdatePrimitiveGeometry"] = mPassData.updateEmissiveTriangleGeom;  // Did our lights move?
        pdfVars["CB"]["gUpdatePrimitiveFlux"] = mPassData.updateEmissiveTriangleFlux;      // Did our lights change color?
        pdfVars["gPackedLightGeom"] = mResources.lightGeometry;             // Output re-packed light geometry
        pdfVars["gPrimitivePDFTexture"] = mResources.localLightPdfTexture;  // Output PDF texture for sampling
        mShader.createLightResources->execute(pContext, launchWidth, launchHeight);
        mPassData.updateEmissiveTriangleGeom = false;

        // The above kernel creates the base level of the light pdf texture; now, create a mipmap chain
        if (mPassData.updateEmissiveTriangleFlux || mPassData.updateLightIntensity)
        {
            mResources.localLightPdfTexture->generateMips(pContext);
            mPassData.updateEmissiveTriangleFlux = false;
            mPassData.updateLightIntensity = false;
        }
    }

    // If we have an environment map, create a pdf texture to sample that, too.
    if (mPassData.scene->getEnvMap() && mPassData.updateEnvironmentMapPdf)
    {
        // Yeah, I know.  Stash the environment map texture size for easier use.
        uint2 envMapSize = uint2(mPassData.scene->getEnvMap()->getEnvMap()->getWidth(),
            mPassData.scene->getEnvMap()->getEnvMap()->getHeight());

        auto pdfVars = mShader.createEnvironmentPDFTexture->getRootVar();
        pdfVars["gEnvMapPDFTexture"] = mResources.environmentPdfTexture;   // Output PDF texture for sampling
        mShader.createEnvironmentPDFTexture->execute(pContext, envMapSize.x, envMapSize.y);

        // Create a mipmap chain from this light pdf texure
        mResources.environmentPdfTexture->generateMips(pContext);
        mPassData.updateEnvironmentMapPdf = false;
    }
}

void RTXDITutorialBase::presampleLights(RenderContext* pContext, const RenderData& data)
{
    /* Do presampling of lights into tiles.  The two shaders executed here are *exceptionally* simple,
       as they just call the approrpriate RTXDI API (i.e., RTXDI_PresampleLocalLights() and
       RTXDI_PresampleEnvironmentMap())
    */

    FALCOR_PROFILE("Presample Lights");

    // For our primitive / triangle lights
    {
        auto presampleVars = mShader.presamplePrimitiveLights->getRootVar();
        setupRTXDIBridgeVars(presampleVars, data);
        mShader.presamplePrimitiveLights->execute(pContext,
            mRtxdiContextParams.TileSize, mRtxdiContextParams.TileCount); // total light samples = tile size * tile count = 1024 * 128
    }

    // For our environment lights
    if (mRtxdiFrameParams.environmentLightPresent)
    {
        auto presampleVars = mShader.presampleEnvironmentLights->getRootVar();
        setupRTXDIBridgeVars(presampleVars, data);
        mShader.presampleEnvironmentLights->execute(pContext,
            mRtxdiContextParams.EnvironmentTileSize, mRtxdiContextParams.EnvironmentTileCount);
    }
}

void RTXDITutorialBase::setCurrentFrameRTXDIParameters(const RenderData& renderData)
{
    /* At the start of each frame, we need to update RTXDI parameters, so it knows the situation
       in the current frame.  This checks for changes, and calls the RTXDI C++ API to fill our
       shader parameter buffer that we give to each of the shaders using RTXDI.
    */

    // Do we use light tile presampling?  (Generally, you should.)
    mRtxdiFrameParams.enableLocalLightImportanceSampling = true;

    // Specify how many emissive triangles we have (for RTXDI, these can be other types of primitives
    // but our Falcor scenes currently only have triangle lights)
    mRtxdiFrameParams.firstLocalLight = 0;
    mRtxdiFrameParams.numLocalLights = mPassData.lights->getActiveLightCount() + mTotalPointLightsCount;
    mRtxdiFrameParams.firstPointLight = mPassData.lights->getActiveLightCount();

    // Are we using an environment map?  If so, store it in the RTXDI light list right
    // after the last triangle light (i.e., envMapIndex == mPassData.lights->getTotalLightCount())
    mRtxdiFrameParams.environmentLightPresent = (mPassData.scene->getEnvMap() != nullptr);
    mRtxdiFrameParams.environmentLightIndex = mPassData.lights->getTotalLightCount() + mTotalPointLightsCount;

    // We currently do not handle "infinite" lights (aka traditional GL/DX directional lights)
    mRtxdiFrameParams.numInfiniteLights = 0;

    // In case we're using ReGIR (currently not done here), update the grid center to be at the camera.
    auto cameraPos = mPassData.scene->getCamera()->getPosition();
    mRtxdiFrameParams.regirCenter = rtxdi::float3{ cameraPos.x, cameraPos.y, cameraPos.z };

    // Update the parameters RTXDI needs when we call its functions in our shaders.
    mpRtxdiContext->FillRuntimeParameters(mRtxdiShaderParams, mRtxdiFrameParams);
}

void RTXDITutorialBase::setupRTXDIBridgeVars(ShaderVar& vars, const RenderData& renderData)
{
    /* Our Falcor - specific RTXDI application bridge has a number of shader parameters & resources
       that we need to set for the bridge to function.  This routine consistently sets them.  Any
       shaders using the bridge needs to call setupRTXDIBridgeVars() prior to launching the shader.
    */

    // Update our shared RTXDI bridge parameter block for the current frame
    vars["BridgeCB"]["gEpsilon"] = float(mLightingParams.epsilon);
    vars["BridgeCB"]["gFrameIndex"] = uint32_t(mRtxdiFrameParams.frameIndex);
    vars["BridgeCB"]["gFrameSize"] = uint2(mpRtxdiContext->GetParameters().RenderWidth, mpRtxdiContext->GetParameters().RenderHeight);
    vars["BridgeCB"]["gEnvMapRes"] = !mResources.environmentPdfTexture
        ? uint2(0u) : uint2(mResources.environmentPdfTexture->getWidth(), mResources.environmentPdfTexture->getHeight());
    vars["BridgeCB"]["gUseLowerShininess"] = !mLightingParams.useHigherShininess;  // Allows matching early prototypes with different matl models
    vars["BridgeCB"]["gRtxdiParams"].setBlob(&mRtxdiShaderParams, sizeof(mRtxdiShaderParams));
    vars["BridgeCB"]["gRelativeTrianglePower"] = float(mLightingParams.relativeTriangleWeight);
    vars["BridgeCB"]["gRelativeEnvironmentPower"] = float(mLightingParams.relativeEnvMapWeight);
    vars["BridgeCB"]["gUseEmissiveTextures"] = mLightingParams.useTextureForShade;
    vars["BridgeCB"]["gUseRTXDIInitialSampling"] = (mLightingParams.selectedDisplayPipeline >= 2u);  // True if not using a "baseline" pipeline
    vars["BridgeCB"]["gStoreCompactedLightInfo"] = mLightingParams.storePerTileLightGeom;
    vars["BridgeCB"]["gCurrentGBufferOffset"] = uint32_t(mLightingParams.currentGBufferIndex * mPassData.screenSize.x * mPassData.screenSize.y);
    vars["BridgeCB"]["gPriorGBufferOffset"] = uint32_t(mLightingParams.priorGBufferIndex * mPassData.screenSize.x * mPassData.screenSize.y);

    // Setup textures and other buffers needed by the RTXDI bridge
    vars["gLightGeometry"] = mResources.lightGeometry;
    vars["gGBuf"] = mResources.gBufferData;
    vars["gRisBuffer"] = mResources.lightTileBuffer;
    vars["gPackedPresampledLights"] = mResources.compressedLightBuffer;
    vars["gLightReservoirs"] = mResources.reservoirBuffer;
    vars["gLocalLightPdfTexture"] = mResources.localLightPdfTexture;
    vars["gEnvMapPdfTexture"] = mResources.environmentPdfTexture;
    vars["gNeighborBuffer"] = mResources.neighborOffsetBuffer;
    vars["gMotionVectorTexture"] = renderData["mvec"]->asTexture();

    // Some debug textures

    // Shared shader parameters. TODO: separate this into another function
    vars["SharedCB"]["gLightMeshTopN"] = mLightTopN;
    vars["SharedCB"]["gCollectingData"] = mEnableStatistics;
    vars["SharedCB"]["gLinearSampler"] = mpLinearSampler;
    vars["SharedCB"]["gSmDepthBias"] = mSmDepthBias;
    vars["SharedCB"]["gIsmDepthBias"] = mIsmDepthBias;
    vars["SharedCB"]["gShadowRayForRestPixels"] = mShadowRayForRestPixels;
    vars["SharedCB"]["gScreenSize"] = mPassData.screenSize;
    vars["SharedCB"]["gTotalLightMeshCount"] = mTotalLightMeshCount;
    vars["SharedCB"]["gSortingRules"] = mSortingRules;
    vars["SharedCB"]["gDepthBiasMode"] = mShadowDepthBias;
    vars["SharedCB"]["gConstEpsilon"] = mConstEpsilonForAdaptiveDepthBias;
    vars["SharedCB"]["gDebugLightMeshID"] = mDebugLightMeshID;
    vars["SharedCB"]["gDebugLight"] = mDebugLight;

    vars["gPrevLightSelectionBuffer"] = mpPrevLightSelectionBuffer;
    vars["gSortedLightsBuffer"] = mpSortedLightsBuffer;
    vars["gLightShadowDataBuffer"] = mpLightShadowDataBuffer;
    vars["gISMs"] = mpIsmTextureArray;
    vars["gShadowOptionsBuffer"] = mpShadowOptionsBuffer;

    for (uint i = 0; i < mLightTopN * mTemporalReusingLength; i++)
    {
        vars["gSortedLightsShadowMaps"][i] = mSortedLightsShadowMaps[i];
    }
}

void RTXDITutorialBase::allocateShadowTextureArrays()
{
    // Create shadow map texture array for each light.
    uint totalShadowMapLights = mLightTopN * mTemporalReusingLength;
    uint maxSupportedLights1024 = std::min(totalShadowMapLights, kMaxSupportedLights1024);
    mSortedLightsShadowMaps.resize(totalShadowMapLights);

    for (uint i = 0; i < totalShadowMapLights; i++)
    {
        uint shadowMapSize = i < maxSupportedLights1024 ? 1024 : 512;
        uint mipLevels = (uint)log2(shadowMapSize / 32) + 1u;
        auto pTextureArray = Texture::create2D(shadowMapSize, shadowMapSize, ResourceFormat::R32Float, kShadowMapsPerLight, mipLevels, nullptr, kAutoMipBindFlags);
        mSortedLightsShadowMaps[i] = pTextureArray;
    }
}

bool RTXDITutorialBase::allocateRtxdiResrouces(RenderContext* pContext, const RenderData& data)
{
    // We can't allocate resources without a scene
    if (!mPassData.scene) return false;

    // Ask for some other refreshes elsewhere to make sure we're all consistent
    mPassData.clearReservoirs = true;
    mPassData.updateEmissiveTriangleFlux = true;
    mPassData.updateEmissiveTriangleGeom = true;
    mPassData.updateEnvironmentMapPdf = true;

    // Make sure the RTXDI context has the current screen resolution
    mRtxdiContextParams.RenderWidth = mPassData.screenSize.x;
    mRtxdiContextParams.RenderHeight = mPassData.screenSize.y;

    // Set the number and size of our presampled tiles
    mRtxdiContextParams.TileSize = mLightingParams.presampledTileSize;
    mRtxdiContextParams.TileCount = mLightingParams.presampledTileCount;
    mRtxdiContextParams.EnvironmentTileSize = mLightingParams.presampledTileSize;
    mRtxdiContextParams.EnvironmentTileCount = mLightingParams.presampledTileCount;

    // Create a new RTXDI context.
    mpRtxdiContext = std::make_unique<rtxdi::Context>(mRtxdiContextParams);
    if (!mpRtxdiContext) return false;

    // Allocate a neighbor offset buffer
    //   -> This really should be a one-time cost; no need to rebuild it as the 8k size is plenty
    //      big, yet is cheap and cache-coherent to use.  Should set and forget this.
    updateNeighborBuffer();

    // Allocate our "RIS buffers" which is what RTXDI calls the buffers containing presampled light tiles
    // (see the HPG 2021 paper, "Rearchitecting Spatiotemporal Resampling for Production")
    uint32_t tileBufferCount = std::max(mpRtxdiContext->GetRisBufferElementCount(), 1u);
    mResources.lightTileBuffer = Buffer::createTyped(ResourceFormat::RG32Uint, tileBufferCount);

    // Each of our presampled lights in the tiles above can have their geometric info stashed in a packed
    // format, which improves coherency (over calling RAB_LoadLightInfo() from the global list of scene lights)
    // Each light currently takes 2x RGBA32F values in this packed format.
    mResources.compressedLightBuffer = Buffer::createTyped(ResourceFormat::RGBA32Float, 2 * tileBufferCount);

    // Allocate light index mapping buffer.  (TODO) Currently unused.  The right thing to do with this
    // buffer is create a correspondance between this frame's light and last frame's lights.  However,
    // since Falcor currently doesn't (really) allow much dynamic lighting, beyond animating lights, we
    // skip this.  Our RTXDI bridge uses the identity mapping when asked for frame-to-frame corresponances.
    uint32_t lightBufferElements = 2 * (mPassData.lights->getTotalLightCount() + mTotalPointLightsCount);
    mResources.lightIndexMapBuffer = Buffer::createTyped(ResourceFormat::R32Uint, lightBufferElements);

    // Allocate light reservoir buffer.  There are multiple reservoirs (specified by kMaxReservoirs) concatenated together,
    // which allows RTXDI to swap between them with just an additional offset.  The number needed depends on the complexity
    // of the lighting pipeline.  2 is *probably* sufficient, but 3 gives a bit more flexibility.
    uint32_t reservoirBufferElementCount = mpRtxdiContext->GetReservoirBufferElementCount() * kMaxReservoirs;
    mResources.reservoirBuffer = Buffer::createStructured(uint32_t(sizeof(RTXDI_PackedReservoir)), reservoirBufferElementCount);

    // Allocate texture for environment map importance sampling.
    if (mPassData.scene->getEnvMap())
    {
        uint envWidth = mPassData.scene->getEnvMap()->getEnvMap()->getWidth();
        uint envHeight = mPassData.scene->getEnvMap()->getEnvMap()->getHeight();
        mResources.environmentPdfTexture = Texture::create2D(envWidth, envHeight, ResourceFormat::R16Float, 1, uint(-1), nullptr, kAutoMipBindFlags);
    }

    // Allocate local light PDF map, which RTXDI uses for importance sampling emissives
    uint32_t texWidth, texHeight, texMipLevels;
    rtxdi::ComputePdfTextureSize(mPassData.lights->getTotalLightCount() + mTotalPointLightsCount, texWidth, texHeight, texMipLevels);
    mResources.localLightPdfTexture = Texture::create2D(texWidth, texHeight, ResourceFormat::R16Float, 1, uint(-1), nullptr, kAutoMipBindFlags);

    // Allocate buffers to store primary hit data.  (Think: G-Buffers)  See `RTXDI_PrepareSurfaceData.cs.slang` for
    // discussion of *why* you might do this rather than using your existing G-Buffer format, whatever it is.  These
    // are, strictly speaking, not necessary if you use your existing G-buffer directly as input to the RTXDI bridge
    mResources.gBufferData = Buffer::createTyped<float4>(2 * mPassData.screenSize.x * mPassData.screenSize.y, kBufferBindFlags);
    mResources.emissiveColors = Texture::create2D(mPassData.screenSize.x, mPassData.screenSize.y, ResourceFormat::RGBA16Float, 1, uint(-1), nullptr, kBufferBindFlags);

    // Falcor stores light info in a fairly complex structure that involves some memory indirection.  In order to
    // improve memory coherency during resampling, where touching light data in a poor way can be catastrophic,
    // we pack data into a coherent structure.
    //  -> Song: for point lights, I don't think we need to build a buffer like this
    if (mPassData.lights->getTotalLightCount() > 0)
        mResources.lightGeometry = Buffer::createTyped<float4>(2 * mPassData.lights->getTotalLightCount());


    // Stochastic shadow map resources
    assert(mTotalLightsCount > 0);
    mpPrevLightSelectionBuffer = Buffer::createTyped<int>(mPassData.screenSize.x * mPassData.screenSize.y, kBufferBindFlags);
    mpLightHistogramBuffer = Buffer::createTyped<uint>(mTotalLightsCount, kBufferBindFlags, Buffer::CpuAccess::Read);
    mpSortedLightsBuffer = Buffer::createTyped<uint2>(mTotalLightsCount, kBufferBindFlags, Buffer::CpuAccess::Read);
    mpTotalValidPixels = Buffer::createTyped<uint>(4, kBufferBindFlags, Buffer::CpuAccess::Read);
    mpReusingLightIndexBuffer = Buffer::createTyped<uint>(mLightTopN * mTemporalReusingLength, kBufferBindFlags, Buffer::CpuAccess::Read);
    pContext->clearUAV(mpReusingLightIndexBuffer->getUAV().get(), uint4(-1));

    mpLightPdfBuffer = Buffer::createTyped<float>(mTotalLightsCount, kBufferBindFlags, Buffer::CpuAccess::Read);
    mpLightCdfBuffer = Buffer::createTyped<float>(mTotalLightsCount + 1, kBufferBindFlags, Buffer::CpuAccess::Read);

    // ISM resources
    auto totalTriangleCount = mPassData.scene->getSceneStats().instancedTriangleCount;
    mpIsmTextureArray = Texture::create2D(mIsmSize, mIsmSize, ResourceFormat::R32Float, mTotalIsmCount, mIsmMipLevels, nullptr, kAutoMipBindFlags);
    mpIsmTexture = Texture::create2D(mIsmVisTextureSize, mIsmVisTextureSize, ResourceFormat::R32Float, 1, 1, nullptr, kAutoMipBindFlags);
    mpPointsBuffer = Buffer::createTyped<float4>((uint32_t)std::min((uint64_t)Buffer::kMaxPossible, totalTriangleCount), kBufferBindFlags, Buffer::CpuAccess::Read);
    mpCounterBuffer = Buffer::createTyped<uint>(mTotalLightsCount, kBufferBindFlags, Buffer::CpuAccess::Read);
    mpIsmLightIndexBuffer = Buffer::createTyped<uint>(mTotalIsmCount / mIsmPerLight, kBufferBindFlags, Buffer::CpuAccess::Read);

    logInfo(std::format("totalTriangleCount = {}, totalVertexCount = {}", totalTriangleCount, mPassData.scene->getSceneStats().instancedVertexCount));

    // Different shadow map size from 32 - 512
    allocateShadowTextureArrays();

    // Recording data resources
    mpShadowOptionsBuffer = Buffer::createTyped<uint>(mPassData.screenSize.x * mPassData.screenSize.y, kBufferBindFlags, Buffer::CpuAccess::Read);
    mpExtraPointsCountBuffer = Buffer::createTyped<uint>(100, kBufferBindFlags, Buffer::CpuAccess::Read);
    mpShadowMapSizeBuffer = Buffer::createTyped<uint>(6, kBufferBindFlags, Buffer::CpuAccess::Read);

    // Baseline resource
    if (mVisibility == Visibility::BaselineSM)
    {
        mpHeroLightShadowMapsTexture = Texture::create2D(mShadowMapSize, mShadowMapSize, ResourceFormat::R32Float, kShadowMapsPerLight * mLightTopN,
            1, nullptr, kAutoMipBindFlags);
    }

    return true;
}

ComputePass::SharedPtr RTXDITutorialBase::createComputeShader(const std::string& file, const std::string& entryPoint)
{
    // Where is the shader?  What shader model to use?
    Program::Desc risDesc(kShaderDirectory + file);
    risDesc.setShaderModel("6_5");

    // Create a Falcor shader wrapper, with specified entry point and scene-specific #defines (for common Falcor HLSL routines)
    ComputePass::SharedPtr pShader = ComputePass::create(risDesc.csEntry(entryPoint), mPassData.scene->getSceneDefines());

    // Type conformances are needed for specific Slang language constructs used in common Falcor shaders, no need to worry
    // about this unless you're using advanced Slang functionality in a very specific way.
    pShader->getProgram()->setTypeConformances(mPassData.scene->getTypeConformances());

    // Zero out structures to make sure they're regenerated with correct settings (conformances, scene defines, etc)
    pShader->setVars(nullptr);

    // Falcor doesn't, by default, pass down scene geometry, materials, etc. for compute shaders.  But we'll need that
    // in all (or most) of our shaders in these tutorials, so just automatically send this data down.
    pShader->getRootVar()["gScene"] = mPassData.scene->getParameterBlock();
    return pShader;
}

ComputePass::SharedPtr RTXDITutorialBase::createComputeShader(const std::string& file, const Program::DefineList& defines, const std::string& entryPoint, bool hasScene)
{
    // Where is the shader?  What shader model to use?
    Program::Desc risDesc(kShaderDirectory + file);
    risDesc.setShaderModel("6_5");

    // Create a Falcor shader wrapper, with specified entry point and scene-specific #defines (for common Falcor HLSL routines)
    auto csDefines = defines;
    if (hasScene)
    {
        csDefines.add(mPassData.scene->getSceneDefines());
    }
    ComputePass::SharedPtr pShader = ComputePass::create(risDesc.csEntry(entryPoint), csDefines);

    // Type conformances are needed for specific Slang language constructs used in common Falcor shaders, no need to worry
    // about this unless you're using advanced Slang functionality in a very specific way.
    pShader->getProgram()->setTypeConformances(mPassData.scene->getTypeConformances());

    // Zero out structures to make sure they're regenerated with correct settings (conformances, scene defines, etc)
    pShader->setVars(nullptr);

    if (hasScene) pShader->getRootVar()["gScene"] = mPassData.scene->getParameterBlock();

    return pShader;
}

void RTXDITutorialBase::loadCommonShaders(void)
{
    // The current setup configuration requires having a scene *before* loading shaders.
    if (!mPassData.scene) return;

    // Load utility shaders needed to setup (some) pipeline variants and data structures for RTXDI.
    mShader.createLightResources = createComputeShader(kRTXDI_CreateLightResources);
    mShader.createEnvironmentPDFTexture = createComputeShader(kRTXDI_CreateEnvMapPDFTexture);
    mShader.presamplePrimitiveLights = createComputeShader(kRTXDI_PresamplePrimitiveLights);
    mShader.presampleEnvironmentLights = createComputeShader(kRTXDI_PresampleEnvironmentLights);
    mShader.prepareSurfaceData = createComputeShader(kRTXDI_PrepareSurfaceData);
}

void RTXDITutorialBase::loadIsmShaders()
{
    Program::DefineList defines;

    // Load CDF building passes
    mpBuildPDF = createComputeShader("BuildReSTIRcdf.cs.slang", defines, "buildPDF");
    mpBuildCDF = createComputeShader("BuildReSTIRcdf.cs.slang", defines, "buildCDF");

    // Load ISM render pass
    {
        Program::Desc desc;
        desc.addShaderLibrary(kShaderDirectory + "ISM_Render.3d.slang").vsEntry("vsMain").gsEntry("gsMain").psEntry("psMain");
        desc.setShaderModel("6_5");
        mIsmRenderPass.pProgram = GraphicsProgram::create(desc);
        mIsmRenderPass.pProgram->addDefines(mPassData.scene->getSceneDefines());
        mIsmRenderPass.pProgram->addDefine("GS_OUT_STREAM", std::to_string(mIsmSceneType));
        mIsmRenderPass.pProgram->setTypeConformances(mPassData.scene->getTypeConformances());

        // Debug tessellation stage
        RasterizerState::Desc wireframeDesc;
        wireframeDesc.setFillMode(RasterizerState::FillMode::Wireframe);
        wireframeDesc.setCullMode(RasterizerState::CullMode::None);
        mpWireframeRS = RasterizerState::create(wireframeDesc);

        // Depth test
        DepthStencilState::Desc dsDesc;
        dsDesc.setDepthEnabled(false);
        mpNoDepthDS = DepthStencilState::create(dsDesc);
        dsDesc.setDepthFunc(ComparisonFunc::Less).setDepthEnabled(true);
        mpDepthTestDS = DepthStencilState::create(dsDesc);

        mIsmRenderPass.pState = GraphicsState::create();
        mIsmRenderPass.pState->setProgram(mIsmRenderPass.pProgram);

        mIsmRenderPass.pVars = GraphicsVars::create(mIsmRenderPass.pProgram->getReflector());

        mIsmRenderPass.pFbo = Fbo::create();
    }

    // Load ISM pull and push pass
    mpIsmPullPass = createComputeShader("ISM_Pull.cs.slang", defines);
    mpIsmPushPass = createComputeShader("ISM_Push.cs.slang", defines);

    //mCsIsmRenderPass.generatePoints = createComputeShader("ISM_Render.cs.slang", defines, "generatePointsCS", true);
    //mCsIsmRenderPass.renderISMs = createComputeShader("ISM_Render.cs.slang", defines, "renderIsmCS", true);
}

void RTXDITutorialBase::loadShaders()
{
    // The current setup configuration requires having a scene *before* loading shaders.
    if (!mPassData.scene) return;

    // Load shaders common to all tutorials
    loadCommonShaders();

    // Load a shader that does a simple Monte Carlo baseline
    mShader.monteCarloBaseline = createComputeShader(kRTXDI_MonteCarloBaseline);

    // Load our shaders for our simple RIS-only RTXDI example
    mShader.risOnlyInitialCandidates = createComputeShader(kRTXDI_RISOnly_InitialSamples);
    mShader.risOnlyShade = createComputeShader(kRTXDI_RISOnly_Shade);

    // Load a shader that only does a single spatial reuse pass (can be run iteratively)
    mShader.spatialReuse = createComputeShader(kRTXDI_SpatialReuseOnly);

    // Load a shader that does temporal reuse
    mShader.temporalReuse = createComputeShader(kRTXDI_TemporalReuseOnly);

    // Load a shader that combines temporal reuse with a (single) spatial/spatiotemporal pass
    mShader.spatiotemporalReuse = createComputeShader(kRTXDI_SpatiotemporalReuse);

    // Load common shaders (across multiple example RTXDI pipelines) for generating initial candidates and doing final shading
    mShader.initialCandidates = createComputeShader(kRTXDI_InitialSamples);
    mShader.initialCandidateVisibility = createComputeShader(kRTXDI_InitialVisibility);
    mShader.shade = createComputeShader(kRTXDI_Shade);


    // ReSTIR shadow map shaders
    
    // Load shaders to compute top N lights
    Program::DefineList defines;
    mComputeTopLightsPass.computeLightHistogram = createComputeShader("ComputeHistogram.cs.slang", defines, "computeHistogramTwoAdds");
    mComputeTopLightsPass.createKeyValuePairs = createComputeShader("ComputeHistogram.cs.slang", defines, "createKeyValuePairs");

    // Load arbitray size Bitonic Sort
    mComputeTopLightsPass.bitonicSort.preSort = createComputeShader("SortHistogram.cs.slang", defines, "preSortCS");
    mComputeTopLightsPass.bitonicSort.innerSort = createComputeShader("SortHistogram.cs.slang", defines, "innerSortCS");
    mComputeTopLightsPass.bitonicSort.outerSort = createComputeShader("SortHistogram.cs.slang", defines, "outerSortCS");

    // Load shadow map pass
    {
        Program::Desc desc;
        desc.addShaderLibrary(kShaderDirectory + "GenerateShadowMap.3d.slang").vsEntry("vsMain").gsEntry("gsMain").psEntry("psMain");
        desc.setShaderModel("6_5");
        mShadowMapPass.pProgram = GraphicsProgram::create(desc);
        mShadowMapPass.pState = GraphicsState::create();
        mShadowMapPass.pState->setProgram(mShadowMapPass.pProgram);

        DepthStencilState::Desc depthStencilDesc;
        //depthStencilDesc.setDepthFunc(DepthStencilState::Func::LessEqual);
        auto depthStencilState = DepthStencilState::create(depthStencilDesc);
        mShadowMapPass.pState->setDepthStencilState(depthStencilState);

        mShadowMapPass.pProgram->addDefines(mPassData.scene->getSceneDefines());
        mShadowMapPass.pVars = GraphicsVars::create(mShadowMapPass.pProgram->getReflector());
        mShadowMapPass.pProgram->setTypeConformances(mPassData.scene->getTypeConformances());

        mShadowMapPass.pFbo = Fbo::create();
    }

    // Update pass
    mpUpdateLightMeshData = createComputeShader("UpdateLightMeshData.cs.slang", defines, "main");
    mpUpdateLightShadowDataCenter = createComputeShader("UpdateLightMeshData.cs.slang", defines, "updateCenterCS");

    // Baseline shade pass
    if (mpEmissiveSampler)
    {
        defines = mpEmissiveSampler->getDefines();
        //defines.add(mpSampleGenerator->getDefines());
    }
    //mpBaselineShading = createComputeShader("BaselineSM_Shade.cs.slang", defines, "main", true);

    // Load ISM shaders
    loadIsmShaders();

    // Load visualize passes
    mVisualizePass.pVisualizeSingle = createComputeShader("VisualizeSM.cs.slang", defines, "singleIsmCS");
    mVisualizePass.pVisualizeAll = createComputeShader("VisualizeSM.cs.slang", defines, "allIsmCS");
    mVisualizePass.pVisualizeAll2 = createComputeShader("VisualizeSM.cs.slang", defines, "allIsmCS2");

    // Load wireframe pass shader
    {
        Program::Desc desc;
        desc.addShaderLibrary(kShaderDirectory + "Wireframe.3d.slang").vsEntry("vsMain").gsEntry("gsMain").psEntry("psMain");
        desc.setShaderModel("6_5");
        mWireframePass.pProgram = GraphicsProgram::create(desc);
        mWireframePass.pProgram->addDefines(mPassData.scene->getSceneDefines());
        mWireframePass.pProgram->setTypeConformances(mPassData.scene->getTypeConformances());
        mWireframePass.pState = GraphicsState::create();
        mWireframePass.pState->setProgram(mWireframePass.pProgram);
        mWireframePass.pVars = GraphicsVars::create(mWireframePass.pProgram->getReflector());
        mWireframePass.pFbo = Fbo::create();
    }
}

void RTXDITutorialBase::prepareLightShadowMapData()
{
    // Get light mesh data from scene
    const auto& meshLights = mPassData.lights->getMeshLights();
    std::vector<LightShadowMapData> lightMeshData(mTotalLightsCount);

    for (uint i = 0; i < mTotalLightsCount; i++)
    {
        LightShadowMapData data;
        data.ismArrayStartIndex = -1;
        data.shadowMapCount = 0;
        data.shadowMapType = 0;
        data.nearFarPlane = mLightNearFarPlane;
        data.shadowMapSize = 1;
        data.age = 0;
        data.reusingArrayIndex = -1;

        // Get the light center for mesh light or point light
        float3 center;
        if (i < mTotalLightMeshCount)
        {
            const auto& instance = mPassData.scene->getGeometryInstance(meshLights[i].instanceID);
            //logInfo(std::format("meshLights[{}], instanceID = {}, geometry instanceIndex = {}, geometryID = {}, geometryIndex = {}",
            //    i, meshLights[i].instanceID, instance.instanceIndex, instance.geometryID, instance.geometryIndex));

            // If it is the Emerald scene, we need to delete some bad emissive meshes

            // We have to apply a transformation on those bounds, because dynamic meshes will not be pre-transferred
            const auto& meshBound = mPassData.scene->getMeshBounds(instance.geometryID);
            const auto& globalMatrices = mPassData.scene->getAnimationController()->getGlobalMatrices();
            const auto& transform = globalMatrices[instance.globalMatrixID];
            auto newMeshBound = meshBound.transform(transform);

            //logInfo(std::format("Mesh Light #{} Name: {}", i, mPassData.scene->getMeshName(instance.geometryID)));

            if (!newMeshBound.valid())
            {
                data.shadowMapCount = -1;
                continue;
            }

            center = newMeshBound.center();

            // Emissive light needs to adjust their near plane
            if (mAdaptiveLightNearPlane)
            {
                float3 extend = newMeshBound.extent();
                float maxExtendComponent = std::max(std::max(extend.x, extend.y), extend.z);
                float minExtendComponent = std::min(std::min(extend.x, extend.y), extend.z);
                data.nearFarPlane.x += minExtendComponent; // Max or min???
            }

            // Need to fix one emissive mesh in Zero Day scene
            if (mSceneName == 1 && i == 203)
            {
                data.nearFarPlane.x = 0.00001f;
            }
        }
        else
        {
            uint pointLightIdx = i - mTotalLightMeshCount;
            center = mPointLights[pointLightIdx]->getWorldPosition();
        }

        // In static scene: output light camera distance
        auto cameraPosW = mPassData.scene->getCamera()->getPosition();
        float distance = glm::length(center - cameraPosW);
        //logInfo(std::format("#{} = {}", i, distance));

        // Set the center position
        data.centerPosW = center;

        // Compute the light space matrix for six faces. Order: +X, -X, +Y, -Y, +Z, -Z
        data.persProjMat = glm::perspective(glm::radians(90.0f), 1.0f, data.nearFarPlane.x, data.nearFarPlane.y);
        for (uint j = 0; j < kShadowMapsPerLight; j++)
        {
            data.viewMats[0] = glm::lookAt(center, center + glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
            data.viewMats[1] = glm::lookAt(center, center + glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
            data.viewMats[2] = glm::lookAt(center, center + glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f));
            data.viewMats[3] = glm::lookAt(center, center + glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f, 0.0f, -1.0f));
            data.viewMats[4] = glm::lookAt(center, center + glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f, -1.0f, 0.0f));
            data.viewMats[5] = glm::lookAt(center, center + glm::vec3(0.0f, 0.0f, -1.0f), glm::vec3(0.0f, -1.0f, 0.0f));
            data.viewProjMats[0] = data.persProjMat * data.viewMats[0];
            data.viewProjMats[1] = data.persProjMat * data.viewMats[1];
            data.viewProjMats[2] = data.persProjMat * data.viewMats[2];
            data.viewProjMats[3] = data.persProjMat * data.viewMats[3];
            data.viewProjMats[4] = data.persProjMat * data.viewMats[4];
            data.viewProjMats[5] = data.persProjMat * data.viewMats[5];
        }

        // Compute light frustum size
        data.lightFrustumSize = 2.0f * data.nearFarPlane.x * glm::tan(glm::radians(0.5f * 90.0f));

        lightMeshData[i] = data;
    }

    // Create the light data buffer
    mpLightShadowDataBuffer = Buffer::createStructured(mShadowMapPass.pVars->getRootVar()["gLightShadowDataBuffer"], mTotalLightsCount, kBufferBindFlags,
        Buffer::CpuAccess::None, lightMeshData.data());
}

void RTXDITutorialBase::computeTopLightsPass(RenderContext* pRenderContext)
{
    FALCOR_PROFILE("Compute Top N Lights");

    pRenderContext->clearUAV(mpTotalValidPixels->getUAV().get(), uint4(0));

    {
        FALCOR_PROFILE("Counting Lights");
        // 1. Count the frequency of each light mesh
        auto histogramVars = mComputeTopLightsPass.computeLightHistogram->getRootVar();
        histogramVars["histogramCB"]["gTotalLightsCount"] = mTotalLightsCount;
        histogramVars["gPrevLightSelectionBuffer"] = mpPrevLightSelectionBuffer; // input
        histogramVars["gFinalHistogramBuffer"] = mpLightHistogramBuffer; // output
        mpPixelDebug->prepareProgram(mComputeTopLightsPass.computeLightHistogram->getProgram(), histogramVars);
        mComputeTopLightsPass.computeLightHistogram->execute(pRenderContext, mPassData.screenSize.x * mPassData.screenSize.y, 1);

        auto keyValueVars = mComputeTopLightsPass.createKeyValuePairs->getRootVar();
        keyValueVars["gFinalHistogramBuffer"] = mpLightHistogramBuffer;
        keyValueVars["gSortBuffer"] = mpSortedLightsBuffer;
        keyValueVars["gTotalValidPixels"] = mpTotalValidPixels;
        mComputeTopLightsPass.createKeyValuePairs->execute(pRenderContext, mTotalLightsCount, 1);
    }

    {
        FALCOR_PROFILE("Sorting Lights");

        // Get the max iteration for inner and outer sort pass. This excludes the pre-sort pass.
        const uint alignedTotalLightCount = (uint)pow(2, (uint)log2(mTotalLightsCount) + 1);
        const uint maxIterations = (uint)log2(std::max(2048u, alignedTotalLightCount)) - 10u;
        const uint nThreads = alignedTotalLightCount / 2;

        // Pre-Sort the buffer up to k = 2048.  This also pads the list with invalid indices
        // that will drift to the end of the sorted list.
        auto preSortVars = mComputeTopLightsPass.bitonicSort.preSort->getRootVar();
        preSortVars["sortCB"]["gTotalLightsCount"] = mTotalLightsCount;
        preSortVars["sortCB"]["gNullItem"] = uint(0);
        preSortVars["gSortBuffer"] = mpSortedLightsBuffer;
        mpPixelDebug->prepareProgram(mComputeTopLightsPass.bitonicSort.preSort->getProgram(), preSortVars);
        mComputeTopLightsPass.bitonicSort.preSort->execute(pRenderContext, nThreads, 1);

        // We have already pre-sorted up through k = 2048 when first writing our list, so
        // we continue sorting with k = 4096.  For unnecessarily large values of k, these
        // indirect dispatches will be skipped over with thread counts of 0.
        for (uint k = 4096; k <= alignedTotalLightCount; k <<= 1)
        {
            // Outer sort on j >= 2048
            auto outerSortVars = mComputeTopLightsPass.bitonicSort.outerSort->getRootVar();
            outerSortVars["sortCB"]["gTotalLightsCount"] = mTotalLightsCount;
            outerSortVars["outerCB"]["k"] = k;
            for (uint j = k / 2; j >= 2048; j >>= 1)
            {
                outerSortVars["outerCB"]["j"] = j;
                outerSortVars["gSortBuffer"] = mpSortedLightsBuffer;
                mComputeTopLightsPass.bitonicSort.outerSort->execute(pRenderContext, nThreads, 1);
            }

            // Inner sort on j <= 1024
            auto innerSortVars = mComputeTopLightsPass.bitonicSort.innerSort->getRootVar();
            innerSortVars["sortCB"]["gTotalLightsCount"] = mTotalLightsCount;
            innerSortVars["sortCB"]["gNullItem"] = uint(0);
            innerSortVars["gSortBuffer"] = mpSortedLightsBuffer;
            mComputeTopLightsPass.bitonicSort.innerSort->execute(pRenderContext, nThreads, 1);
        }
    }
}

void RTXDITutorialBase::prepareIsms(RenderContext* pRenderContext, const RenderData& renderData)
{
    FALCOR_PROFILE("Prepare ISMs");

    // Build CDF buffer on ReSTIR distribution  
    if (mAdaptiveISM)
    {
        FALCOR_PROFILE("Build CDF");

        auto pdfVars = mpBuildPDF->getRootVar();
        pdfVars["gLightHistogramBuffer"] = mpLightHistogramBuffer;
        pdfVars["gTotalValidPixels"] = mpTotalValidPixels;
        pdfVars["gLightPdfBuffer"] = mpLightPdfBuffer;
        pdfVars["gLightShadowDataBuffer"] = mpLightShadowDataBuffer;
        //mpPixelDebug->prepareProgram(mpBuildPDF->getProgram(), pdfVars);
        mpBuildPDF->execute(pRenderContext, mTotalLightsCount, 1);

        auto cdfVars = mpBuildCDF->getRootVar();
        cdfVars["gLightPdfBuffer"] = mpLightPdfBuffer;
        cdfVars["gLightCdfBuffer"] = mpLightCdfBuffer;
        //mpPixelDebug->prepareProgram(mpBuildCDF->getProgram(), cdfVars);
        mpBuildCDF->execute(pRenderContext, mTotalLightsCount, 1);
    }

    // This rasterization stage simplifies the scene into point representation (vs + (hs + ds) + gs)
    {
        FALCOR_PROFILE("ISM Render");

        GraphicsState::Viewport viewport(0.0f, 0.0f, (float)mIsmSize, (float)mIsmSize, 0.0f, 1.0f);
        mIsmRenderPass.pState->setViewport(0, viewport);

        // Clear all counting buffer for each frame
        pRenderContext->clearUAV(mpCounterBuffer->getUAV().get(), uint4(0));
        pRenderContext->clearUAV(mpExtraPointsCountBuffer->getUAV().get(), uint4(0));

        mIsmRenderPass.pFbo->attachColorTarget(mpIsmTextureArray, 0, 0, 0, mTotalIsmCount);
        pRenderContext->clearTexture(mpIsmTextureArray.get(), float4(1.0f));

        mIsmRenderPass.pState->setFbo(mIsmRenderPass.pFbo);
        auto pStagingDepthTexture = Texture::create2D(mIsmSize, mIsmSize, ResourceFormat::D32Float, mTotalIsmCount, 1, nullptr, kShadowMapFlags);
        pRenderContext->clearTexture(pStagingDepthTexture.get(), float4(1.0f));
        mIsmRenderPass.pFbo->attachDepthStencilTarget(pStagingDepthTexture, 0, 0, mTotalIsmCount); // Depth test needs this
        mIsmRenderPass.pState->setDepthStencilState(mpDepthTestDS);


        auto ismVars = mIsmRenderPass.pVars->getRootVar();
        ismVars["ismCB"]["gTotalLightsCount"] = mTotalLightsCount;
        ismVars["ismCB"]["gTotalIsmCount"] = mTotalIsmCount;
        ismVars["ismCB"]["gIsmPerLight"] = mIsmPerLight;
        ismVars["ismCB"]["gScreenSize"] = mPassData.screenSize;
        ismVars["ismCB"]["gBaseTriSize"] = mBaseTriSize;
        ismVars["ismCB"]["gAdaptiveISM"] = mAdaptiveISM;
        ismVars["ismCB"]["gIsmLightSamplingMode"] = mIsmLightSamplingMode;
        ismVars["gLightShadowDataBuffer"] = mpLightShadowDataBuffer;
        ismVars["gExtraPointsCountBuffer"] = mpExtraPointsCountBuffer;
        ismVars["gLightPdfBuffer"] = mpLightPdfBuffer;
        ismVars["gLightCdfBuffer"] = mpLightCdfBuffer;
        //ismVars["gPointsBuffer"] = mpPointsBuffer;
        ismVars["gCounterBuffer"] = mpCounterBuffer;

        mpPixelDebug->prepareProgram(mIsmRenderPass.pProgram, ismVars);

        // Since we are rendering points, it doesn't matter if we cull back or not
        mPassData.scene->rasterize(pRenderContext, mIsmRenderPass.pState.get(), mIsmRenderPass.pVars.get(), RasterizerState::CullMode::None); 
    }

    // TODO: if the light (array index) already has shadow map, return in the shader side.
    // Do pull pass on ISMs
    {
        FALCOR_PROFILE("ISM Pull");
        for (uint m = 0; m < mpIsmTextureArray->getMipCount() - 1; m++)
        {
            // TODO: parallelize this loop 
            for (uint a = 0; a < mpIsmTextureArray->getArraySize(); a++)
            {
                const auto& pDefaultBlockReflection = mpIsmPullPass->getProgram()->getReflector()->getDefaultParameterBlock();
                auto srcTexBindLoc = pDefaultBlockReflection->getResourceBinding("gIsmInput");
                auto dstTexBindLoc = pDefaultBlockReflection->getResourceBinding("gIsmOutput");

                // Get src and dst's texture shared pointer
                auto pSrc = mpIsmTextureArray->getSRV(m, 1, a, 1);
                auto pDst = mpIsmTextureArray->getUAV(m + 1, a, 1);
                const uint32_t srcMipLevel = pSrc->getViewInfo().mostDetailedMip;
                const uint32_t dstMipLevel = pDst->getViewInfo().mostDetailedMip;
                Texture::SharedPtr pSrcTexture = std::static_pointer_cast<Texture>(pSrc->getResource());
                Texture::SharedPtr pDstTexture = std::static_pointer_cast<Texture>(pDst->getResource());

                auto pullVars = mpIsmPullPass->getRootVar();
                pullVars["PullCB"]["gDepthDiffThreshold"] = mDepthThreshold /* * (float)pow(2, m)*/;
                mpIsmPullPass->getVars()->setSrv(srcTexBindLoc, pSrc);
                mpIsmPullPass->getVars()->setUav(dstTexBindLoc, pDst);
                mpPixelDebug->prepareProgram(mpIsmPullPass->getProgram(), pullVars);
                mpIsmPullPass->execute(pRenderContext, mpIsmTextureArray->getWidth(m + 1), mpIsmTextureArray->getHeight(m + 1));
            }
        }
    }

    {
        visualizeShadowMapTex(mVisualizePass.pVisualizeSingle, mpIsmTextureArray, renderData["ism after pull"]->asTexture(), pRenderContext, mVisualizeMipLevel,
            mVisLightFaceID, "single");
    }

    // Do push pass on ISMs
    {
        FALCOR_PROFILE("ISM Push");
        for (uint m = mpIsmTextureArray->getMipCount() - 1; m > 0; m--)
        {
            for (uint a = 0; a < mpIsmTextureArray->getArraySize(); a++)
            {
                const auto& pDefaultBlockReflection = mpIsmPushPass->getProgram()->getReflector()->getDefaultParameterBlock();
                auto srcTexBindLoc = pDefaultBlockReflection->getResourceBinding("gInput");
                auto dstTexBindLoc = pDefaultBlockReflection->getResourceBinding("gOutput");
                auto pSrc = mpIsmTextureArray->getSRV(m, 1, a, 1);
                auto pDst = mpIsmTextureArray->getUAV(m - 1, a, 1);

                auto pushVars = mpIsmPushPass->getRootVar();
                pushVars["PushCB"]["gPushMode"] = mIsmPushMode;
                pushVars["PushCB"]["gDepthDiffThreshold"] = mDepthThreshold;
                mpIsmPushPass->getVars()->setSrv(srcTexBindLoc, pSrc);
                mpIsmPushPass->getVars()->setUav(dstTexBindLoc, pDst);
                //mpPixelDebug->prepareProgram(mpIsmPushPass->getProgram(), pushVars);
                mpIsmPushPass->execute(pRenderContext, mpIsmTextureArray->getWidth(m - 1), mpIsmTextureArray->getHeight(m - 1));
            }
        }
    }

    // Visualize ISM pass
    {
        visualizeShadowMapTex(mVisualizePass.pVisualizeSingle, mpIsmTextureArray, renderData["ism after push"]->asTexture(), pRenderContext, mVisualizeMipLevel,
            mVisLightFaceID, "single");

        // Get the proper texture to display
        uint resolution = mIsmSize * mIsmSize * mTotalLightsCount * mIsmPerLight; 
        std::string texName;
        if (resolution <= uint(1 << 22))
        {
            texName = "all ism 2k";
        }
        else if (resolution <= uint(1 << 24))
        {
            texName = "all ism 4k";
        }
        else if (resolution <= uint(1 << 26))
        {
            texName = "all ism 8k";
        }
        else
        {
            texName = "all ism 14k";
        }

        const auto& pOutputTex = renderData[texName]->asTexture();
        //visualizeShadowMapTex(mVisualizePass.pVisualizeAll, mpIsmTextureArray, pOutputTex, pRenderContext, mVisualizeMipLevel,
        //    mVisLightID, "all");
        visualizeShadowMapTex(mVisualizePass.pVisualizeAll2, mpIsmTextureArray, pOutputTex, pRenderContext, mVisualizeMipLevel,
            mVisLightFaceID, "all in order");
    }

    // Print resource data
    if (mGpuDataToGet.getIsmData)
    {
        mValuesToPrint.extraPointsCount = getDeviceResourceData<uint>(pRenderContext, mpExtraPointsCountBuffer);

        std::string output;
        mValuesToPrint.totalExtraPoints = 0;
        for (uint i = 0; i < mValuesToPrint.extraPointsCount.size(); i++)
        {
            output += std::to_string(mValuesToPrint.extraPointsCount[i]) + " ";
            mValuesToPrint.totalExtraPoints += i * mValuesToPrint.extraPointsCount[i];
        }
        //logInfo(std::format("extra Points Count buffer = [{}]", output));
    }
}

void RTXDITutorialBase::prepareStochasticShadowMaps(RenderContext* pRenderContext, const RenderData& renderData)
{
    FALCOR_PROFILE("Prepare shadow maps");

    if (mDrawWireframe)
    {
        mWireframePass.pFbo->attachColorTarget(renderData["wireframe"]->asTexture(), 0);
        mWireframePass.pState->setFbo(mWireframePass.pFbo);
        mWireframePass.pState->setDepthStencilState(mpNoDepthDS);
        pRenderContext->clearFbo(mWireframePass.pFbo.get(), float4(float3(0.0f), 1.0f), 1.0f, 0);

        auto wireframeVars = mWireframePass.pVars->getRootVar();
        wireframeVars["PerFrameCB"]["gColor"] = float4(0, 1, 0, 1);
        mPassData.scene->rasterize(pRenderContext, mWireframePass.pState.get(), mWireframePass.pVars.get(), mpWireframeRS, mpWireframeRS);
    }

    // Animation updates if light center changes
    if (mPassData.updateEmissiveTriangleGeom || mPassData.updateLightPosition)
    {
        // Get light mesh data from scene
        const auto& meshLights = mPassData.lights->getMeshLights();
        std::vector<float3> newCenters(mTotalLightsCount);

        for (uint i = 0; i < mTotalLightsCount; i++)
        {
            // Get the light center for mesh light or point light
            float3 center;
            if (i < mTotalLightMeshCount)
            {
                const auto& instance = mPassData.scene->getGeometryInstance(meshLights[i].instanceID);

                // We have to apply a transformation on those bounds, because dynamic meshes will not be pre-transferred
                const auto& meshBound = mPassData.scene->getMeshBounds(instance.geometryID);
                const auto& globalMatrices = mPassData.scene->getAnimationController()->getGlobalMatrices();
                const auto& transform = globalMatrices[instance.globalMatrixID];
                auto newMeshBound = meshBound.transform(transform);

                if (!newMeshBound.valid())
                {
                    continue;
                }
                center = newMeshBound.center();
            }
            else
            {
                uint pointLightIdx = i - mTotalLightMeshCount;
                center = mPointLights[pointLightIdx]->getWorldPosition();
            }

            newCenters[i] = center;
        }

        // Pass new centers to device
        auto pNewCentersBuffer = Buffer::createTyped<float3>(mTotalLightsCount, kBufferBindFlags, Buffer::CpuAccess::None, newCenters.data());

        auto vars = mpUpdateLightShadowDataCenter->getRootVar();
        vars["gNewCenterBuffer"] = pNewCentersBuffer;
        vars["gLightShadowDataBuffer"] = mpLightShadowDataBuffer;
        mpUpdateLightShadowDataCenter->execute(pRenderContext, mTotalLightsCount, 1);
    }

    // Only update ranking list and shadow maps in some frames
    if (mRtxdiFrameParams.frameIndex % mUpdatingFrequency != 0) return;

    // TODO: We also need to clear the light shadow map data buffer using a compute pass. Only clear the light whose shadow map type != 1


    // Because we are directly adding to the histogram buffer, so in each frame we need to clear this buffer instead of accumulating the result
    pRenderContext->clearUAV(mpLightHistogramBuffer->getUAV().get(), uint4(0));


    // Get the sorted light index list
    computeTopLightsPass(pRenderContext);

    // Update corresponding light mesh data for those top N lights. It is only reusing the shadow maps for now
    {
        FALCOR_PROFILE("Update Light Data");

        pRenderContext->clearUAV(mpIsmLightIndexBuffer->getUAV().get(), uint4(-1));
        pRenderContext->clearUAV(mpShadowMapSizeBuffer->getUAV().get(), uint4(0));

        auto vars = mpUpdateLightMeshData.getRootVar();
        vars["CB"]["gTotalLightsCount"] = mTotalLightsCount;
        vars["CB"]["gMaxReusingCount"] = mLightTopN * mTemporalReusingLength;
        vars["CB"]["gLightTopN"] = mLightTopN;
        vars["CB"]["gTemporalReusingLength"] = mTemporalReusingLength;
        vars["CB"]["gShadowMapsPerLight"] = kShadowMapsPerLight;
        vars["CB"]["gCurrFrameLightStartIdx"] = mCurrFrameLightStartIdx;
        vars["CB"]["gIsmPerLight"] = mIsmPerLight;
        vars["CB"]["gVisibility"] = (uint)mVisibility;
        vars["CB"]["gFullSizeShadowMaps"] = mFullSizeShadowMaps;
        vars["CB"]["gSceneName"] = mSceneName;
        vars["gSortedLightsBuffer"] = mpSortedLightsBuffer;
        vars["gReusingLightIndexBuffer"] = mpReusingLightIndexBuffer;
        vars["gIsmLightIndexBuffer"] = mpIsmLightIndexBuffer;
        vars["gLightShadowDataBuffer"] = mpLightShadowDataBuffer;
        vars["gShadowMapSizeBuffer"] = mpShadowMapSizeBuffer;
        vars["gTotalValidPixels"] = mpTotalValidPixels;
        mpPixelDebug->prepareProgram(mpUpdateLightMeshData->getProgram(), vars);
        mpUpdateLightMeshData->execute(pRenderContext, 2, 1); // change back to 1
    }


    // Cubic shadow maps. TODO: we don't need to render the same light if it is still in the shadow map list
    //  -> create an array only to store the unique light added into reusing list
    if (mVisibility == Visibility::ShadowMap_FullyLit || mVisibility == Visibility::ShadowMap_ISM)
    {
        FALCOR_PROFILE("Cubic Shadow Maps");

        auto shadowVars = mShadowMapPass.pVars->getRootVar();
        shadowVars["shadowMapCB"]["gVisMode"] = (uint)mVisibility;
        shadowVars["shadowMapCB"]["gCurrFrameLightStartIdx"] = mCurrFrameLightStartIdx;
        shadowVars["shadowMapCB"]["gDepthBias"] = mSmDepthBias;
        shadowVars["gReusingLightIndexBuffer"] = mpReusingLightIndexBuffer;
        shadowVars["gLightShadowDataBuffer"] = mpLightShadowDataBuffer;

        // Loop over top N light mesh index from sorted list and generate a shadow map for each one
        for (uint i = 0; i < mLightTopN; i++)
        {
            // Get current light's texture array to be rendered
            uint shadowMapArrayIndex = mCurrFrameLightStartIdx + i;
            const auto& pShadowMapTextureArray = mSortedLightsShadowMaps[shadowMapArrayIndex];
            uint shadowMapSize = pShadowMapTextureArray->getWidth();

            // Depth test needs this
            auto pStagingDepthTexture = Texture::create2D(shadowMapSize, shadowMapSize, ResourceFormat::D32Float, kShadowMapsPerLight, 1, nullptr, kShadowMapFlags);
            mShadowMapPass.pFbo->attachDepthStencilTarget(pStagingDepthTexture, 0, 0, kShadowMapsPerLight); 

            // Render the six faces
            mShadowMapPass.pFbo->attachColorTarget(mSortedLightsShadowMaps[shadowMapArrayIndex], 0, 0, 0, kShadowMapsPerLight);
            mShadowMapPass.pState->setFbo(mShadowMapPass.pFbo);

            // Clear the texture for each light mesh shadow map
            pRenderContext->clearFbo(mShadowMapPass.pFbo.get(), float4(1.0f), 1.0f, 0);

            shadowVars["shadowMapCB"]["gRanking"] = i;
            mpPixelDebug->prepareProgram(mShadowMapPass.pProgram, shadowVars);
            mPassData.scene->rasterize(pRenderContext, mShadowMapPass.pState.get(), mShadowMapPass.pVars.get(), RasterizerState::CullMode::Back);
        }
    }

    {
        FALCOR_PROFILE("Generate Shadow Mipmaps");
        for (uint i = 0; i < mLightTopN; i++)
        {
            uint shadowMapArrayIndex = mCurrFrameLightStartIdx + i;
            auto& pShadowMapTextureArray = mSortedLightsShadowMaps[shadowMapArrayIndex];
            pShadowMapTextureArray->generateMips(pRenderContext);
        }
    }

    {
        //visualizeShadowMapTex(mVisualizePass.pVisualizeSingle, mpReusingShadowMapsTexture, renderData["debugSM"]->asTexture(), pRenderContext, 0, 3, "single");
    }

    // Imperfect shadow maps
    if (mVisibility == Visibility::AllISM || mVisibility == Visibility::ShadowMap_ISM)
    {
        prepareIsms(pRenderContext, renderData);
    }

    // Move the light start index to next location
    uint totalReusingLightCount = mLightTopN * mTemporalReusingLength;
    mCurrFrameLightStartIdx = (mCurrFrameLightStartIdx + mLightTopN) % totalReusingLightCount;

    // TODO: use the member function
    if (mRtxdiFrameParams.frameIndex >= 100 && mRtxdiFrameParams.frameIndex <= 100)
    {
        printDeviceResources(pRenderContext);

        auto totalValidPixels = getDeviceResourceData<uint>(pRenderContext, mpTotalValidPixels);
        logInfo(std::format("Total Valid Pixels = {}", totalValidPixels[0]));

        // Compute all shadow maps memory size
        mValuesToPrint.totalShadowTexSize = 0;
        for (uint i = 0; i < mLightTopN * mTemporalReusingLength; i++)
        {
            mValuesToPrint.totalShadowTexSize += mSortedLightsShadowMaps[i]->getTextureSizeInBytes();
        }

        // PDF and CDF
        auto pdfArray = getDeviceResourceData<float>(pRenderContext, mpLightPdfBuffer);
        auto cdfArray = getDeviceResourceData<float>(pRenderContext, mpLightCdfBuffer);
        std::string pdfOutput = "";
        std::string cdfOutput = "";
        for (uint i = 0; i < cdfArray.size(); i++)
        {
            pdfOutput += std::to_string(pdfArray[i]) + ", ";
            pdfOutput += (i + 1) % 10 == 0 ? "\n" : "";

            cdfOutput += std::to_string(cdfArray[i]) + ", ";
            cdfOutput += (i + 1) % 10 == 0 ? "\n" : "";
        }
        logInfo(std::format("PDF Array = [\n{}\n]", pdfOutput));
        logInfo(std::format("CDF Array = [\n{}\n]", cdfOutput));

        // Light primitive distribution
        auto primCounterArray = getDeviceResourceData<uint>(pRenderContext, mpCounterBuffer);
        std::string output = "";
        for (uint i = 0; i < primCounterArray.size(); i++)
        {
            output += std::to_string(primCounterArray[i]) + " ";
            output += (i + 1) % 10 == 0 ? "\n" : "";
        }
        logInfo(std::format("Light point distribution = [\n{}\n]", output));

    }

    if (mGpuDataToGet.getSortingData)
    {
        std::string output;

        // Sorted lights array in each frame
        auto sortedLightIndexArray = getDeviceResourceData<uint2>(pRenderContext, mpSortedLightsBuffer);
        output = "";
        for (uint i = 0; i < sortedLightIndexArray.size(); i++)
        {
            output += std::to_string(sortedLightIndexArray[i].x) + " ";
            output += (i + 1) % 10 == 0 ? "\n" : "";
        }
        mValuesToPrint.sortedLightIndexArray = output;

        // Shadow map and ISM corresponding light indexes
        auto reusingLightIndexArray = getDeviceResourceData<uint>(pRenderContext, mpReusingLightIndexBuffer);
        output = "";
        for (uint i = 0; i < reusingLightIndexArray.size(); i++)
        {
            output += std::to_string(reusingLightIndexArray[i]) + " ";
            output += (i + 1) % 10 == 0 ? "\n" : "";
        }
        mValuesToPrint.reusingLightIndexArray = output;

        auto ismLightIndexArray = getDeviceResourceData<uint>(pRenderContext, mpIsmLightIndexBuffer);
        output = "";
        for (uint i = 0; i < ismLightIndexArray.size(); i++)
        {
            output += std::to_string(ismLightIndexArray[i]) + " ";
            output += (i + 1) % 10 == 0 ? "\n" : "";
        }
        mValuesToPrint.ismLightIndexArray = output;
    }

    if (mGpuDataToGet.getShadowMapSizeData)
    {
        // Shadow map size statistics
        auto shadowMapSizeCount = getDeviceResourceData<uint>(pRenderContext, mpShadowMapSizeBuffer);
        std::string output = "";
        uint totalMemorySizeInByte = 0;
        for (uint i = 0; i < shadowMapSizeCount.size(); i++)
        {
            output += std::format("Shadow Map Size {}: {}", pow(2, 5 + i), shadowMapSizeCount[i]);
            output += i != shadowMapSizeCount.size() - 1 ? "\n" : "";
        }
        mValuesToPrint.shadowMapSizeCount = output;
    }
}

void RTXDITutorialBase::runBaselineShadowMap(RenderContext* pRenderContext, const RenderData& renderData)
{
    // TODO: implement stochastic light tiling later, so turn off for now
    return;

    // Get the top N lights. Only need to do once at the beginning
    if (mRtxdiFrameParams.frameIndex == mLightingParams.maxHistoryLength)
    {
        computeTopLightsPass(pRenderContext);
    }

    // We are going to overload some variables: mLightTopN = number of hero lights, mpReusingLightIndexBuffer = light index of hero lights,
    // mpReusingShadowMapsTexture = shadow maps of hero lights
    {
        FALCOR_PROFILE("Generate Shadow Maps");

        auto shadowVars = mShadowMapPass.pVars->getRootVar();
        shadowVars["shadowMapCB"]["gLightNear"] = mLightNearFarPlane.x;
        shadowVars["shadowMapCB"]["gLightFar"] = mLightNearFarPlane.y;
        shadowVars["shadowMapCB"]["gVisMode"] = (uint)mVisibility;
        shadowVars["gLightShadowDataBuffer"] = mpLightShadowDataBuffer;
        shadowVars["gSortedLightsBuffer"] = mpSortedLightsBuffer;

        // Loop over top N light mesh index from sorted list and generate a shadow map for each one
        auto pStagingDepthTexture = Texture::create2D(mShadowMapSize, mShadowMapSize, ResourceFormat::D32Float, kShadowMapsPerLight, 1, nullptr, kShadowMapFlags);
        mShadowMapPass.pFbo->attachDepthStencilTarget(pStagingDepthTexture); // Depth test needs this
        for (uint i = 0; i < mLightTopN; i++)
        {
            // Render the six faces
            uint sliceOffset = kShadowMapsPerLight * i;
            mShadowMapPass.pFbo->attachColorTarget(mpHeroLightShadowMapsTexture, 0, 0, sliceOffset, kShadowMapsPerLight);
            mShadowMapPass.pState->setFbo(mShadowMapPass.pFbo);

            // Clear the texture for each light mesh shadow map
            pRenderContext->clearFbo(mShadowMapPass.pFbo.get(), float4(1.0f), 1.0f, 0);

            shadowVars["shadowMapCB"]["gRanking"] = i;
            mpPixelDebug->prepareProgram(mShadowMapPass.pProgram, shadowVars);
            mPassData.scene->rasterize(pRenderContext, mShadowMapPass.pState.get(), mShadowMapPass.pVars.get(), RasterizerState::CullMode::Back);
        }
    }

    {
        FALCOR_PROFILE("Baseline Shading");

        auto baselineVars = mpBaselineShading->getRootVar();
        baselineVars["BaselineCB"]["gHeroLightsCount"] = mLightTopN;
        baselineVars["BaselineCB"]["gTotalLightMeshCount"] = mTotalPointLightsCount;
        baselineVars["gHeroLightShadowMaps"] = mpHeroLightShadowMapsTexture;
        baselineVars["gInputEmission"] = mResources.emissiveColors;
        baselineVars["gOutputColor"] = renderData["color"]->asTexture();
        baselineVars["gVbuffer"] = renderData["vbuffer"]->asTexture();
        mpEmissiveSampler->setShaderData(baselineVars["BaselineCB"]["gEmissiveSampler"]);
        setupRTXDIBridgeVars(baselineVars, renderData);
        mpPixelDebug->prepareProgram(mpBaselineShading->getProgram(), baselineVars);
        mpBaselineShading->execute(pRenderContext, mPassData.screenSize.x, mPassData.screenSize.y);
    }
}

void RTXDITutorialBase::visualizeShadowMapTex(ComputePass::SharedPtr pPass, Texture::SharedPtr pSrcTexture, Texture::SharedPtr pDstTexture, RenderContext* pRenderContext,
    uint mipLevel, uint2 lightFaceIndex, const std::string& mode)
{
    uint2 nThreads = uint2(pSrcTexture->getWidth(), pSrcTexture->getHeight());

    // Bind common shader variables
    auto visualizeVars = pPass->getRootVar();
    visualizeVars["VisCB"]["gPointSampler"] = mpPointSampler;
    visualizeVars["VisCB"]["gLinearSampler"] = mpLinearSampler;
    visualizeVars["VisCB"]["gInputSize"] = nThreads;
    visualizeVars["gIsmInput"] = pSrcTexture;
    visualizeVars["gLightShadowDataBuffer"] = mpLightShadowDataBuffer;
    visualizeVars["gIsmOutput"] = pDstTexture;
    visualizeVars["gIsmLightIndexBuffer"] = mpIsmLightIndexBuffer;

    if (mode == "single")
    {
        visualizeVars["VisCB"]["gMipLevel"] = (float)mipLevel;
        visualizeVars["VisCB"]["gLightFaceIndex"] = lightFaceIndex;

        mpPixelDebug->prepareProgram(mVisualizePass.pVisualizeSingle->getProgram(), visualizeVars);
        pPass->execute(pRenderContext, nThreads.x, nThreads.y);
    }
    else
    {
        visualizeVars["VisCB"]["gOutputSize"] = uint2(pDstTexture->getWidth(), pDstTexture->getHeight());
        mpPixelDebug->prepareProgram(mVisualizePass.pVisualizeAll->getProgram(), visualizeVars);

        if (mode == "all")
            pPass->execute(pRenderContext, nThreads.x, nThreads.y, pSrcTexture->getArraySize());
        else
            pPass->execute(pRenderContext, nThreads.x, nThreads.y, mIsmPerLight * mTotalLightsCount);
    }
}

// Debugging GPU result. TODO: remove and use getDeviceResourceData function
void RTXDITutorialBase::printDeviceResources(RenderContext* pRenderContext)
{
    uint size = mLightTopN * mTemporalReusingLength;

    // Must use a staging buffer which has no bind flags to map data
    auto pStagingBuffer1 = Buffer::createTyped<uint>(mTotalLightsCount, ResourceBindFlags::None, Buffer::CpuAccess::Read);
    auto pStagingBuffer2 = Buffer::createTyped<uint2>(mTotalLightsCount, ResourceBindFlags::None, Buffer::CpuAccess::Read);
    pRenderContext->copyBufferRegion(pStagingBuffer1.get(), 0, mpLightHistogramBuffer.get(), 0, sizeof(uint) * mTotalLightsCount);
    pRenderContext->copyBufferRegion(pStagingBuffer2.get(), 0, mpSortedLightsBuffer.get(), 0, sizeof(uint2) * mTotalLightsCount);

    auto pStagingBuffer3 = Buffer::createTyped<uint>(size, ResourceBindFlags::None, Buffer::CpuAccess::Read);
    pRenderContext->copyBufferRegion(pStagingBuffer3.get(), 0, mpReusingLightIndexBuffer.get(), 0, sizeof(uint) * size);


    // Flush GPU and wait for results to be available.
    pRenderContext->flush(false);
    mpFence->gpuSignal(pRenderContext->getLowLevelData()->getCommandQueue());
    mpFence->syncCpu();

    // Read back GPU results
    uint* pData1 = reinterpret_cast<uint*>(pStagingBuffer1->map(Buffer::MapType::Read));
    std::vector<uint> deviceResult1(pData1, pData1 + mTotalLightsCount); // invoke range constructor (linear time)
    pStagingBuffer1->unmap();

    uint2* pData2 = reinterpret_cast<uint2*>(pStagingBuffer2->map(Buffer::MapType::Read));
    std::vector<uint2> deviceResult2(pData2, pData2 + mTotalLightsCount); // invoke range constructor (linear time)
    pStagingBuffer2->unmap();

    uint* pData3 = reinterpret_cast<uint*>(pStagingBuffer3->map(Buffer::MapType::Read));
    std::vector<uint> deviceResult3(pData3, pData3 + size);
    pStagingBuffer3->unmap();

    // Printing to log
    uint sum = 0;
    for (uint i = 0; i < deviceResult1.size(); i++)
    {
        //logInfo(std::format("mpLightHistogramBuffer[{}] = {}", i, deviceResult1[i]));
    }

    for (uint i = 0; i < deviceResult2.size(); i++)
    //for (uint i = 0; i < 100; i++)
    {
        sum += deviceResult2[i].y;
        logInfo(std::format("Light Mesh ID {} = {}", deviceResult2[i].x, deviceResult2[i].y));
    }
    logInfo(std::format("sum = {}, # of pixels = {}", sum, mPassData.screenSize.x * mPassData.screenSize.y));

    // Print reusing light index buffer
    std::string output;
    for (uint i = 0; i < size; i++)
    {
        output += std::to_string(deviceResult3[i]) + " ";
    }
    logInfo(std::format("Reusing light index buffer = [{}]", output));

}

void RTXDITutorialBase::calcDiffShadowCoverage(const std::vector<uint>& shadowOptionsList, uint pass)
{
    uint totalShadowOptions = 5;

    // Wait unitl the whole temporal reusing is stable
    if (mRtxdiFrameParams.frameIndex >= mLightingParams.maxHistoryLength + mTemporalReusingLength)
    {
        std::unordered_map<uint, uint> shadowOptionsCount;
        auto totalPixels = shadowOptionsList.size();
        for (uint i = 0; i < shadowOptionsList.size(); i++)
        {
            shadowOptionsCount[shadowOptionsList[i]] += 1;
        }

        //logInfo(std::format("totalPixels = {}", totalPixels));
        //logInfo(std::format("shadowOptionsCount[0] = {}", shadowOptionsCount[0]));

        for (uint i = 0; i < totalShadowOptions; i++)
        {
            if (pass == 0)
            {
                mValuesToPrint.shadowOptionsCoverage1[i] = 1.0f * shadowOptionsCount[i] / totalPixels;
            }
            else
            {
                mValuesToPrint.shadowOptionsCoverage2[i] = 1.0f * shadowOptionsCount[i] / totalPixels;
            }
        }
    }
}
