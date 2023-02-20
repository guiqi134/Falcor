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

    // TODO: better can be divided by number of PSMs in each frame
    const uint kMaxIsmCountPerArray = 2046;
    const uint kMaxPsmCountPerArray = 2046;
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
        else if (key == "ismParaDepthBias") mIsmDepthBias.x = float(value);
        else if (key == "ismPersDepthBias") mIsmDepthBias.y = float(value);
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

    // Scene independent parameters
    mIsmPerLight = mIsmLinearProjection ? 6u : 2u;
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
    reflector.addOutput("debugSM", "").texture2D(mShadowMapSize, mShadowMapSize).format(ResourceFormat::RGBA32Float);
    reflector.addOutput("staticSM", "").texture2D(mShadowMapSize, mShadowMapSize).format(ResourceFormat::RGBA32Float);
    reflector.addOutput("debugMotionTex", "").texture2D(mShadowMapSize, mShadowMapSize).format(ResourceFormat::RGBA32Float);
    reflector.addOutput("debugScreenMotion", "").texture2D(mShadowMapSize, mShadowMapSize).format(ResourceFormat::RGBA32Float);
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
        mTopN = mSortingRules == (uint)SortingRules::LightFaces ? mLightFaceTopN : mLightTopN;
        mIsmPerLight = mIsmLinearProjection ? 6u : 2u;
        mTotalIsmCount = mTotalLightsCount * mIsmPerLight;
        mCurrFrameReusingStartIdx = 0;
        mRtxdiFrameParams.frameIndex = 0;
        mPsmCountPerFrame = mSortingRules == (uint)SortingRules::LightFaces ? mLightFaceTopN : mLightTopN * kShadowMapsPerLight;
        mPassData.clearReservoirs = true;

        // Update all resources
        //allocateRtxdiResrouces(pRenderContext, renderData);
        allocateStochasticSmResources(pRenderContext, renderData);
    }

    if (bool(mUpdates & UpdateFlags::ShaderDefinesChanged))
    {
        logInfo("Update defines");

        mIsmRenderPass.pProgram->addDefine("GS_OUT_STREAM", std::to_string(mIsmSceneType));
        mpUpdateLightMeshData->addDefine("VALID_ARRAY_SIZE", std::to_string(mTopN));
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
        {0, "All Pixels"}, {1, "Occluded Pixels"}, {2, "All pixels per light face"}
    };
    Gui::DropdownList depthBiasMode = {
        {0, "Constant"}, {1, "Slope Scale"}, {2, "Dou2014"}
    };
    Gui::DropdownList temporalReusingFix = {
        {0, "None"}, {1, "Split Dynamic Objects"}, {2, "Update with Motion Vector"}
    };

    // Global parameters
    resourcesChanged |= widget.dropdown("ReSTIR Statistics Sorting Rules", sortingRules, mSortingRules);
    resourcesChanged |= widget.dropdown("Temporal Resuing Fix Method", temporalReusingFix, mTemporalReusingFix);
    widget.dropdown("Shadow Depth Bias Mode", depthBiasMode, mShadowDepthBias);
    if (widget.var("Number of Top N light meshes", mLightTopN))
    {
        resourcesChanged = true;
        definesChanged = true;
    }
    if (widget.var("Number of Top N light faces", mLightFaceTopN, 6u, mTotalLightsCount * kShadowMapsPerLight, kShadowMapsPerLight))
    {
        resourcesChanged = true;
        definesChanged = true;
    }
    resourcesChanged |= widget.var("Temporal Reusing Length", mTemporalReusingLength, 1u);
    widget.var("Light Age Threshold", mLightAgeThreshold);
    widget.var("Updating Frequency (in frames)", mUpdatingFrequency);
    widget.var("Cubic Shadow Map Depth Bias", mSmDepthBias);
    widget.var("Debug Light Mesh ID", mDebugLightMeshID);
    widget.checkbox("Adaptive ISM", mAdaptiveISM);
    widget.checkbox("Restrict light changing between SM and ISM?", mLimitSwitchRegionInUpdating);
    widget.checkbox("Only Use ISM for Testing Candidates?", mOnlyIsmForRanking);
    widget.checkbox("Use Compute Shader to render ISM?", mRenderIsmCS);
    widget.checkbox("Reconstruct PSMs?", mUseReconstructPSMs);

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
    ismParams.var("ISM Paraboloid & Perspective Depth Bias", mIsmDepthBias);
    resourcesChanged |= ismParams.var("ISM Pull&Push Mip Levels", mIsmMipLevels);
    ismParams.dropdown("ISM Push Mode", ismPushModes, mIsmPushMode);
    definesChanged |= ismParams.dropdown("ISM Scene Type", ismSceneTypes, mIsmSceneType);
    resourcesChanged |= ismParams.dropdown("ISM Size", ismSizes, mIsmSize);
    resourcesChanged |= ismParams.dropdown("ISM Min Size", ismSizes, mIsmMinSize);
    ismParams.var("Scene depth threshold scale", mSceneDepthThresholdScale);
    ismParams.var("Base Triangle Size", mBaseTriSize);
    resourcesChanged |= ismParams.checkbox("Use Linear Projection?", mIsmLinearProjection);

    ismParams.var("Mip level to visualize", mVisualizeMipLevel);
    ismParams.var("Light's ISM to visualize", mVisLightFaceID, 0);
    ismParams.release();

    // GPU data
    Gui::Group gpuData(widget.gui(), "Print GPU Data", true);
    gpuData.checkbox("Get Shadow Map Size Data?", mGpuDataToGet.getShadowMapSizeData);
    gpuData.checkbox("Get ISM Data?", mGpuDataToGet.getIsmData);
    gpuData.checkbox("Get Light Sorting Data?", mGpuDataToGet.getSortingData);
    gpuData.checkbox("Get Reusing Light Data?", mGpuDataToGet.getReusingLightData);
    gpuData.checkbox("Get Converage Data?", mGpuDataToGet.getConverageData);
    gpuData.release();


    // TODO: use string stream to output
    std::ostringstream oss;

    Gui::Group printValues(widget.gui(), "Printing", true);
    printValues.text(std::format("Current frame: {}", mRtxdiFrameParams.frameIndex));
    printValues.text(std::format("Total mesh lights: {}, Total point lights: {}", mTotalLightMeshCount, mTotalPointLightsCount));
    printValues.text(std::format("Total ISM Count: {}", mTotalIsmCount));
    printValues.text(std::format("Reusing shadow maps count: {}", mPsmCountPerFrame * mTemporalReusingLength));
    printValues.text(std::format("Total shadow map textures size: {}", formatByteSize(mValuesToPrint.totalSmTexSize)));
    printValues.text(std::format("ISM texture array memory: {}", formatByteSize(mValuesToPrint.totalIsmTexSize)));

    if (mGpuDataToGet.getShadowMapSizeData)
    {
        printValues.text("");
        printValues.text(mValuesToPrint.shadowMapSizeCount);
    }

    if (mGpuDataToGet.getIsmData)
    {
        printValues.text("");
        printValues.text(std::format("Total points generated for ISM: {}", mValuesToPrint.totalPoints));
        printValues.text(std::format("Total Extra points generated for ISM: {}", mValuesToPrint.totalExtraPoints));
    }

    if (mGpuDataToGet.getSortingData)
    {
        printValues.text("");
        printValues.text(std::format("Total non-zero light/face: {}", mValuesToPrint.totalNonZeroLightOrFace));
        printValues.text(std::format("Sorted light index buffer each frame: \n[{}]", mValuesToPrint.sortedLightIndexArray));
        printValues.text(std::format("Reusing light index buffer: \n[{}]", mValuesToPrint.reusingLightIndexArray));
        printValues.text(std::format("ISM light index buffer: \n[{}]", mValuesToPrint.ismLightIndexArray));

        printValues.text(mValuesToPrint.lightRankingAcrossFrames);
    }

    if (mGpuDataToGet.getReusingLightData)
    {
        printValues.text("");
        printValues.text(std::format("Reusing Light Age: \n[{}]", mValuesToPrint.reusingLightAge));
        printValues.text(std::format("Reusing Light Ranking: \n[{}]", mValuesToPrint.reusingLightRanking));
        printValues.text(std::format("Reusing Light Frequency: \n[{}]", mValuesToPrint.reusingLightFrequency));
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
        mLightFaceTopN = std::min(mLightFaceTopN, mTotalLightsCount * kShadowMapsPerLight);
        mTopN = mSortingRules == (uint)SortingRules::LightFaces ? mLightFaceTopN : mLightTopN;
        mPsmCountPerFrame = mSortingRules == (uint)SortingRules::LightFaces ? mLightFaceTopN : mLightTopN * kShadowMapsPerLight;

        // Compute the total ISMs
        mTotalIsmCount = mIsmPerLight * mTotalLightsCount;

        // Create emissive light sampler
        //mpEmissiveSampler = EmissivePowerSampler::create(pContext, pScene);
        mpEmissiveSampler = EmissiveUniformSampler::create(pContext, pScene);

        // Get the max extend of the scene
        float3 sceneExtend = mPassData.scene->getSceneBounds().extent();
        float maxExtend = std::max(std::max(sceneExtend.x, sceneExtend.y), sceneExtend.z);
        //mLightNearFarPlane.y = std::min(mLightNearFarPlane.y, maxExtend);
        mDepthThreshold = mSceneDepthThresholdScale * (maxExtend - mLightNearFarPlane.x) / (mLightNearFarPlane.y - mLightNearFarPlane.x);
        mConstEpsilonForAdaptiveDepthBias = glm::length(sceneExtend) * 0.0001f;
        //if (mRtxdiFrameParams.frameIndex == 50)
        //    logInfo(std::format("Scene extend = ({}, {}, {}), depth threshold = {}", sceneExtend.x, sceneExtend.y, sceneExtend.z, depthThreshold));

        //mSortedLightsISMs.resize(size_t(mTotalIsmCount / kMaxIsmCountPerArray + 1u));
        //mSortedLightsISMsUint.resize(size_t(mTotalIsmCount / kMaxIsmCountPerArray + 1u));

        // Create a buffer to store each instance's triangle count
        uint totalTriangleCount = (uint)mPassData.scene->getSceneStats().instancedTriangleCount;
        std::vector<uint> triOffsetArray;
        uint accumIndexCount = 0;
        uint accumTriangleCount = 0;
        for (uint instanceID = 0; instanceID < pScene->getGeometryInstanceCount(); instanceID++)
        {
            triOffsetArray.push_back(accumTriangleCount);

            const auto& instance = pScene->getGeometryInstance(instanceID);
            const auto& meshDesc = pScene->getMesh(instance.geometryID);

            uint triangleCount = meshDesc.getTriangleCount();
            mIsmMaxTriCountAcrossInstance = std::max(mIsmMaxTriCountAcrossInstance, triangleCount);
            accumTriangleCount += triangleCount;

            //logInfo(std::format("instance ID = {}, triangleCount = {}, index count = {}", instanceID, accumTriangleCount, meshDesc.indexCount));
        }
        triOffsetArray.push_back(accumTriangleCount);
        mpTriOffsetBuffer = Buffer::createTyped<uint>((uint)triOffsetArray.size(), kBufferBindFlags, Buffer::CpuAccess::Read, triOffsetArray.data());
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
    mRtxdiFrameParams.environmentLightIndex = mPassData.lights->getTotalLightCount() + mTotalPointLightsCount; // Song: why not use activeLightCount()?

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
    vars["SharedCB"]["gCollectingData"] = mEnableStatistics;
    vars["SharedCB"]["gLinearSampler"] = mpLinearSampler;
    vars["SharedCB"]["gSmDepthBias"] = mSmDepthBias;
    vars["SharedCB"]["gIsmDepthBias"] = mIsmDepthBias;
    vars["SharedCB"]["gShadowRayForRestPixels"] = mShadowRayForRestPixels;
    vars["SharedCB"]["gScreenSize"] = mPassData.screenSize;
    vars["SharedCB"]["gTotalLightMeshCount"] = mTotalLightMeshCount;
    vars["SharedCB"]["gDepthBiasMode"] = mShadowDepthBias;
    vars["SharedCB"]["gConstEpsilon"] = mConstEpsilonForAdaptiveDepthBias;
    vars["SharedCB"]["gIsmLinearProjection"] = mIsmLinearProjection;
    vars["SharedCB"]["gShadowMapsPerLight"] = kShadowMapsPerLight;
    vars["SharedCB"]["gSortingRules"] = mSortingRules;
    vars["SharedCB"]["gOnlyIsmForRanking"] = mEnableShadowRay ? false : mOnlyIsmForRanking;
    
    vars["SharedCB"]["gDebugLightMeshID"] = mDebugLightMeshID;
    vars["SharedCB"]["gDebugLight"] = mDebugLight;

    vars["gPrevLightSelectionBuffer"] = mpPrevLightSelectionBuffer;
    vars["gSortedLightsBuffer"] = mpSortedLightsBuffer;
    vars["gLightShadowDataBuffer"] = mpLightShadowDataBuffer;
    vars["gShadowOptionsBuffer"] = mpShadowOptionsBuffer;

    //vars["gDebugScreenMotion"] = renderData["debugScreenMotion"]->asTexture();

    for (uint i = 0; i < mSortedLightsShadowMaps.size(); i++)
    {
        vars["gSortedLightsShadowMaps"][i] = mSortedLightsShadowMaps[i];
    }

    for (uint i = 0; i < mSortedLightsISMs.size(); i++)
    {
        vars["gSortedLightsISMs"][i] = mSortedLightsISMs[i];
    }
}

void RTXDITutorialBase::allocateShadowTextureArrays(RenderContext* pContext)
{
    // Create shadow map texture array (for each light).
    uint totalShadowMaps =  mPsmCountPerFrame * mTemporalReusingLength;
    uint maxSupportedShadowMaps1024 = kMaxSupportedLights1024 * kShadowMapsPerLight;
    uint numPsmTexArrays = totalShadowMaps / maxSupportedShadowMaps1024 + 1u;
    if (numPsmTexArrays > 1)
        numPsmTexArrays += (totalShadowMaps - maxSupportedShadowMaps1024) / kMaxPsmCountPerArray + 1u;
    mSortedLightsShadowMaps.resize(numPsmTexArrays);
    mSortedLightsShadowMapsStatic.resize(numPsmTexArrays);
    //mSortedLightsDepthTexturesStatic.resize(numPsmTexArrays);
    mSortedLightsMotionTextures.resize(numPsmTexArrays);

    logInfo(std::format("numPsmTexArrays = {}", numPsmTexArrays));

    for (uint i = 0; i < numPsmTexArrays; i++)
    {
        // Only the PSM in first texture array will have 1024*1024 size
        uint shadowMapSize = i < 1 ? 1024 : 512;
        //uint mipLevels = (uint)log2(shadowMapSize / 32) + 1u;
        uint mipLevels = 1u;
        uint arraySize = 1;
        if (i == numPsmTexArrays - 1)
        {
            if (i <= 1) arraySize = totalShadowMaps % maxSupportedShadowMaps1024;
            else arraySize = (totalShadowMaps - maxSupportedShadowMaps1024) % kMaxPsmCountPerArray;
        }
        else
        {
            if (i == 0) arraySize = maxSupportedShadowMaps1024;
            else arraySize = kMaxPsmCountPerArray;
        }

        logInfo(std::format("PSM arraySize = {}", arraySize));

        auto pTextureArray = Texture::create2D(shadowMapSize, shadowMapSize, ResourceFormat::R32Float, arraySize, mipLevels, nullptr, kAutoMipBindFlags);
        pContext->clearTexture(pTextureArray.get(), float4(1.0f));

        mSortedLightsShadowMaps[i] = pTextureArray;

        if (mTemporalReusingFix != (uint)TemporalReusingFix::None)
        {
            auto pStaticTextureArray = Texture::create2D(shadowMapSize, shadowMapSize, ResourceFormat::R32Float, arraySize, mipLevels, nullptr, kAutoMipBindFlags);
            pContext->clearTexture(pStaticTextureArray.get(), float4(1.0f));
            mSortedLightsShadowMapsStatic[i] = pStaticTextureArray;

            //auto pDepthTexture = Texture::create2D(shadowMapSize, shadowMapSize, ResourceFormat::D32Float, arraySize, 1, nullptr, kShadowMapFlags);
            //pContext->clearTexture(pDepthTexture.get(), float4(1.0f));
            //mSortedLightsDepthTexturesStatic[i] = pDepthTexture;

            if (mTemporalReusingFix == (uint)TemporalReusingFix::MotionVector)
            {
                auto pMotionTexture = Texture::create2D(shadowMapSize, shadowMapSize, ResourceFormat::RGBA32Float, arraySize, mipLevels, nullptr, kAutoMipBindFlags);
                pContext->clearTexture(pMotionTexture.get());
                mSortedLightsMotionTextures[i] = pMotionTexture;
            }
        }
    }

    // Create ISM texture array to hold all lights + a staging uint ISM texture array
    uint numIsmTexArrays = mTotalIsmCount / kMaxIsmCountPerArray + 1u;
    mSortedLightsISMs.resize(numIsmTexArrays);
    mSortedLightsISMsUint.resize(numIsmTexArrays);

    for (uint i = 0; i < numIsmTexArrays; i++)
    {
        uint arraySize = i == numIsmTexArrays - 1 ? mTotalIsmCount % kMaxIsmCountPerArray : kMaxIsmCountPerArray;
        auto pIsmTexArray = Texture::create2D(mIsmSize, mIsmSize, ResourceFormat::R32Float, arraySize, mIsmMipLevels, nullptr, kAutoMipBindFlags);
        mSortedLightsISMs[i] = pIsmTexArray;

        auto pIsmTexArrayUint = Texture::create2D(mIsmSize, mIsmSize, ResourceFormat::R32Uint, arraySize, mIsmMipLevels, nullptr, kAutoMipBindFlags);
        mSortedLightsISMsUint[i] = pIsmTexArrayUint;
    }
}

void RTXDITutorialBase::allocateStochasticSmResources(RenderContext* pRenderContext, const RenderData& renderData)
{
    assert(mTotalLightsCount > 0);
    uint elementCount = mSortingRules == (uint)SortingRules::LightFaces ? mTotalLightsCount * kShadowMapsPerLight : mTotalLightsCount;

    // PSM resources
    mpPrevLightSelectionBuffer = Buffer::createTyped<int>(mPassData.screenSize.x * mPassData.screenSize.y, kBufferBindFlags);
    mpLightHistogramBuffer = Buffer::createTyped<uint>(elementCount, kBufferBindFlags, Buffer::CpuAccess::Read);
    mpSortedLightsBuffer = Buffer::createTyped<uint2>(elementCount, kBufferBindFlags, Buffer::CpuAccess::Read);
    mpTotalValidPixels = Buffer::createTyped<uint>(4, kBufferBindFlags, Buffer::CpuAccess::Read);
    mpReusingLightIndexBuffer = Buffer::createTyped<uint>(mTopN * mTemporalReusingLength, kBufferBindFlags, Buffer::CpuAccess::Read);
    pRenderContext->clearUAV(mpReusingLightIndexBuffer->getUAV().get(), uint4(-1));

    mpLightPdfBuffer = Buffer::createTyped<float>(elementCount, kBufferBindFlags, Buffer::CpuAccess::Read);
    mpLightCdfBuffer = Buffer::createTyped<float>(elementCount + 1, kBufferBindFlags, Buffer::CpuAccess::Read);

    // Pre-compute the center of all light meshes and their transformation matrix
    // TODO: use computer shader to clear this structured buffer
    auto lightMeshData = prepareLightShadowMapData();
    mpLightShadowDataBuffer = Buffer::createStructured(mShadowMapPass.pVars->getRootVar()["gLightShadowDataBuffer"], mTotalLightsCount, kBufferBindFlags,
        Buffer::CpuAccess::None, lightMeshData.data());

    // ISM resources
    elementCount = mSortingRules == (uint)SortingRules::LightFaces ? mTotalIsmCount : mTotalLightsCount;
    auto totalTriangleCount = mPassData.scene->getSceneStats().instancedTriangleCount;
    mpPointsBuffer = Buffer::createTyped<float4>((uint32_t)std::min((uint64_t)Buffer::kMaxPossible, totalTriangleCount), kBufferBindFlags, Buffer::CpuAccess::Read);
    mpCounterBuffer = Buffer::createTyped<uint>(elementCount, kBufferBindFlags, Buffer::CpuAccess::Read);
    mpIsmLightIndexBuffer = Buffer::createTyped<uint>((uint)elementCount, kBufferBindFlags, Buffer::CpuAccess::Read);

    logInfo(std::format("totalTriangleCount = {}, totalVertexCount = {}", totalTriangleCount, mPassData.scene->getSceneStats().instancedVertexCount));

    // Different shadow map size from 32 - 512
    allocateShadowTextureArrays(pRenderContext);

    // Recording data resources
    mpShadowOptionsBuffer = Buffer::createTyped<uint>(mPassData.screenSize.x * mPassData.screenSize.y, kBufferBindFlags, Buffer::CpuAccess::Read);
    mpExtraPointsCountBuffer = Buffer::createTyped<uint>(100, kBufferBindFlags, Buffer::CpuAccess::Read);
    mpShadowMapSizeBuffer = Buffer::createTyped<uint>(6, kBufferBindFlags, Buffer::CpuAccess::Read);
    mpRecordReusingLightData = Buffer::createTyped<uint4>(mTopN * mTemporalReusingLength, kBufferBindFlags, Buffer::CpuAccess::Read);

    // Baseline resource
    if (mVisibility == Visibility::BaselineSM)
    {
        mpHeroLightShadowMapsTexture = Texture::create2D(mShadowMapSize, mShadowMapSize, ResourceFormat::R32Float, kShadowMapsPerLight * mTopN,
            1, nullptr, kAutoMipBindFlags);
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

    // ISM CS render
    mpIsmRenderPassCS = createComputeShader("ISM_Render.cs.slang", defines, "renderIsmCS", true);
    mpIsmUint2Float = createComputeShader("ISM_Render.cs.slang", defines, "uint2FloatCS", true);
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

    // Defines needed for light SM and ISM array
    Program::DefineList shadowMapDefines;

    // Load common shaders (across multiple example RTXDI pipelines) for generating initial candidates and doing final shading
    mShader.initialCandidates = createComputeShader(kRTXDI_InitialSamples);
    mShader.initialCandidateVisibility = createComputeShader(kRTXDI_InitialVisibility, shadowMapDefines, "main", true);
    mShader.shade = createComputeShader(kRTXDI_Shade, shadowMapDefines, "main", true);


    // ReSTIR shadow map shaders
    
    // Load shaders to compute top N lights
    Program::DefineList dummyDefines;
    mComputeTopLightsPass.computeLightHistogram = createComputeShader("ComputeHistogram.cs.slang", dummyDefines, "computeHistogramTwoAdds");
    mComputeTopLightsPass.createKeyValuePairs = createComputeShader("ComputeHistogram.cs.slang", dummyDefines, "createKeyValuePairs");

    // Load arbitray size Bitonic Sort
    mComputeTopLightsPass.bitonicSort.preSort = createComputeShader("SortHistogram.cs.slang", dummyDefines, "preSortCS");
    mComputeTopLightsPass.bitonicSort.innerSort = createComputeShader("SortHistogram.cs.slang", dummyDefines, "innerSortCS");
    mComputeTopLightsPass.bitonicSort.outerSort = createComputeShader("SortHistogram.cs.slang", dummyDefines, "outerSortCS");

    // Update pass
    Program::DefineList updatePassDefines;
    updatePassDefines.add("VALID_ARRAY_SIZE", std::to_string(mTopN));
    mpUpdateLightMeshData = createComputeShader("UpdateLightMeshData.cs.slang", updatePassDefines, "main");
    mpUpdateLightShadowDataCenter = createComputeShader("UpdateLightMeshData.cs.slang", dummyDefines, "updateCenterCS");
    mpPerFrameGeneralUpdates = createComputeShader("UpdateLightMeshData.cs.slang", dummyDefines, "generalUpdates");
    mpUpdatePrevViewMatrices = createComputeShader("UpdateLightMeshData.cs.slang", dummyDefines, "updatePrevViewMatrices");

    // Load (static) shadow map pass
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
        mShadowMapPass.pProgram->addDefine("MAX_VERTEX_COUNT", std::to_string(3u * kShadowMapsPerLight));
        mShadowMapPass.pVars = GraphicsVars::create(mShadowMapPass.pProgram->getReflector());
        mShadowMapPass.pProgram->setTypeConformances(mPassData.scene->getTypeConformances());

        mShadowMapPass.pFbo = Fbo::create();
    }

    // Load PSM reconstruction pass
    mpReconstructPSMs = createComputeShader("ReconstructPSMs.cs.slang", dummyDefines, "main");

    // Load ISM shaders
    loadIsmShaders();

    // Load visualize passes
    mVisualizePass.pVisualizeSinglePsm = createComputeShader("VisualizeSM.cs.slang", dummyDefines, "singlePsmCS");
    mVisualizePass.pVisualizeSingleIsm = createComputeShader("VisualizeSM.cs.slang", dummyDefines, "singleIsmCS");
    mVisualizePass.pVisualizeSingleMotionTex = createComputeShader("VisualizeSM.cs.slang", dummyDefines, "singleMotionTexCS");
    mVisualizePass.pVisualizeAll = createComputeShader("VisualizeSM.cs.slang", dummyDefines, "allIsmCS");
    mVisualizePass.pVisualizeAll2 = createComputeShader("VisualizeSM.cs.slang", dummyDefines, "allIsmCS2");

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

std::vector<LightShadowMapData> RTXDITutorialBase::prepareLightShadowMapData()
{
    //logInfo("prepareLightShadowMapData() called");

    // Get light mesh data from scene
    const auto& meshLights = mPassData.lights->getMeshLights();
    std::vector<LightShadowMapData> lightDataArray(mTotalLightsCount);

    for (uint i = 0; i < mTotalLightsCount; i++)
    {
        LightShadowMapData lightData;
        lightData.nearFarPlane = mLightNearFarPlane;

        // Initialize light face data
        for (uint face = 0; face < kShadowMapsPerLight; face++)
        {
            lightData.lightFaceData[face].psmTexArrayIdx = -1;
            lightData.lightFaceData[face].ismTexArrayIdx = -1;
            lightData.lightFaceData[face].currRanking = -1;
            lightData.lightFaceData[face].newPsmLightFace = true;
            lightData.lightFaceData[face].shadowMapType = 0;
            lightData.lightFaceData[face].psmAge = 0;
        }

        // Get the light center for mesh light or point light
        float3 center;
        if (i < mTotalLightMeshCount)
        {
            const auto& instance = mPassData.scene->getGeometryInstance(meshLights[i].instanceID);

            mPassData.scene->getAnimationController()->isMatrixChanged(instance.globalMatrixID);

            // We have to apply a transformation on those bounds, because dynamic meshes will not be pre-transferred
            const auto& meshBound = mPassData.scene->getMeshBounds(instance.geometryID);
            const auto& globalMatrices = mPassData.scene->getAnimationController()->getGlobalMatrices();
            const auto& transform = globalMatrices[instance.globalMatrixID];
            auto newMeshBound = meshBound.transform(transform);

            if (!newMeshBound.valid())
            {
                lightData.isLightValid = false;
                continue;
            }
            center = newMeshBound.center();

            //logInfo(std::format("meshLights[{}], instanceID = {}, geometry instanceIndex = {}, geometryID = {}, geometryIndex = {}, name = {}, material name = {}, Center: ({}, {}, {})",
            //    i, meshLights[i].instanceID, instance.instanceIndex, instance.geometryID, instance.geometryIndex, mPassData.scene->getMeshName(instance.geometryID), mPassData.scene->getMaterial(meshLights[i].materialID)->getName(), center.x, center.y, center.z));

            // Emissive light needs to adjust their near plane. TODO: adaptively for each face
            if (mAdaptiveLightNearPlane)
            {
                float3 extend = newMeshBound.extent();
                float maxExtendComponent = std::max(std::max(extend.x, extend.y), extend.z);
                float minExtendComponent = std::min(std::min(extend.x, extend.y), extend.z);
                lightData.nearFarPlane.x += minExtendComponent; 
            }

            // Need to fix one emissive mesh in Zero Day scene
            if (mSceneName == 1 && i == 203)
            {
                lightData.nearFarPlane.x = 0.00001f;
            }
        }
        else
        {
            uint pointLightIdx = i - mTotalLightMeshCount;
            center = mPointLights[pointLightIdx]->getWorldPosition();
        }

        // Set the center position
        lightData.centerPosW = center;

        // Compute the light space projection matrix 
        lightData.persProjMat = glm::perspective(glm::radians(90.0f), 1.0f, lightData.nearFarPlane.x, lightData.nearFarPlane.y);

        // Compute the light space view matrix for six faces. Order: +X, -X, +Y, -Y, +Z, -Z
        lightData.lightFaceData[0].viewMat = glm::lookAt(center, center + glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        lightData.lightFaceData[1].viewMat = glm::lookAt(center, center + glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        lightData.lightFaceData[2].viewMat = glm::lookAt(center, center + glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f));
        lightData.lightFaceData[3].viewMat = glm::lookAt(center, center + glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f, 0.0f, -1.0f));
        lightData.lightFaceData[4].viewMat = glm::lookAt(center, center + glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        lightData.lightFaceData[5].viewMat = glm::lookAt(center, center + glm::vec3(0.0f, 0.0f, -1.0f), glm::vec3(0.0f, 1.0f, 0.0f));

        // Initialize previous view matrix
        lightData.lightFaceData[0].prevViewMat = glm::lookAt(center, center + glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        lightData.lightFaceData[1].prevViewMat = glm::lookAt(center, center + glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        lightData.lightFaceData[2].prevViewMat = glm::lookAt(center, center + glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f));
        lightData.lightFaceData[3].prevViewMat = glm::lookAt(center, center + glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f, 0.0f, -1.0f));
        lightData.lightFaceData[4].prevViewMat = glm::lookAt(center, center + glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        lightData.lightFaceData[5].prevViewMat = glm::lookAt(center, center + glm::vec3(0.0f, 0.0f, -1.0f), glm::vec3(0.0f, 1.0f, 0.0f));

        // Compute light frustum size
        lightData.lightFrustumSize = 2.0f * lightData.nearFarPlane.x * glm::tan(glm::radians(0.5f * 90.0f));

        lightDataArray[i] = lightData;  
    }

    return lightDataArray;
}

void RTXDITutorialBase::computeTopLightsPass(RenderContext* pRenderContext)
{
    FALCOR_PROFILE("Compute Top N Lights");

    pRenderContext->clearUAV(mpTotalValidPixels->getUAV().get(), uint4(0));
    uint totalBinCount = mSortingRules == (uint)SortingRules::LightFaces ? mTotalLightsCount * kShadowMapsPerLight : mTotalLightsCount;

    {
        FALCOR_PROFILE("Counting Phase");
        // 1. Count the frequency of each light mesh/face
        auto histogramVars = mComputeTopLightsPass.computeLightHistogram->getRootVar();
        histogramVars["histogramCB"]["gTotalBinCount"] = totalBinCount;
        histogramVars["gPrevLightSelectionBuffer"] = mpPrevLightSelectionBuffer; // input
        histogramVars["gFinalHistogramBuffer"] = mpLightHistogramBuffer; // output
        mpPixelDebug->prepareProgram(mComputeTopLightsPass.computeLightHistogram->getProgram(), histogramVars);
        mComputeTopLightsPass.computeLightHistogram->execute(pRenderContext, mPassData.screenSize.x * mPassData.screenSize.y, 1);

        auto keyValueVars = mComputeTopLightsPass.createKeyValuePairs->getRootVar();
        keyValueVars["gFinalHistogramBuffer"] = mpLightHistogramBuffer;
        keyValueVars["gSortBuffer"] = mpSortedLightsBuffer;
        keyValueVars["gTotalValidPixels"] = mpTotalValidPixels;
        mComputeTopLightsPass.createKeyValuePairs->execute(pRenderContext, totalBinCount, 1);
    }

    {
        FALCOR_PROFILE("Sorting Phase");

        // Get the max iteration for inner and outer sort pass. This excludes the pre-sort pass.
        const uint alignedTotalElementCount = (uint)pow(2, (uint)log2(totalBinCount) + 1);
        const uint maxIterations = (uint)log2(std::max(2048u, alignedTotalElementCount)) - 10u;
        const uint nThreads = alignedTotalElementCount / 2;

        // Pre-Sort the buffer up to k = 2048.  This also pads the list with invalid indices
        // that will drift to the end of the sorted list.
        auto preSortVars = mComputeTopLightsPass.bitonicSort.preSort->getRootVar();
        preSortVars["sortCB"]["gTotalElementCount"] = totalBinCount;
        preSortVars["sortCB"]["gNullItem"] = uint(0);
        preSortVars["gSortBuffer"] = mpSortedLightsBuffer;
        mpPixelDebug->prepareProgram(mComputeTopLightsPass.bitonicSort.preSort->getProgram(), preSortVars);
        mComputeTopLightsPass.bitonicSort.preSort->execute(pRenderContext, nThreads, 1);

        // We have already pre-sorted up through k = 2048 when first writing our list, so
        // we continue sorting with k = 4096.  For unnecessarily large values of k, these
        // indirect dispatches will be skipped over with thread counts of 0.
        for (uint k = 4096; k <= alignedTotalElementCount; k <<= 1)
        {
            // Outer sort on j >= 2048
            auto outerSortVars = mComputeTopLightsPass.bitonicSort.outerSort->getRootVar();
            outerSortVars["sortCB"]["gTotalElementCount"] = totalBinCount;
            outerSortVars["outerCB"]["k"] = k;
            for (uint j = k / 2; j >= 2048; j >>= 1)
            {
                outerSortVars["outerCB"]["j"] = j;
                outerSortVars["gSortBuffer"] = mpSortedLightsBuffer;
                mComputeTopLightsPass.bitonicSort.outerSort->execute(pRenderContext, nThreads, 1);
            }

            // Inner sort on j <= 1024
            auto innerSortVars = mComputeTopLightsPass.bitonicSort.innerSort->getRootVar();
            innerSortVars["sortCB"]["gTotalElementCount"] = totalBinCount;
            innerSortVars["sortCB"]["gNullItem"] = uint(0);
            innerSortVars["gSortBuffer"] = mpSortedLightsBuffer;
            mComputeTopLightsPass.bitonicSort.innerSort->execute(pRenderContext, nThreads, 1);
        }
    }
}

void RTXDITutorialBase::prepareIsms(RenderContext* pRenderContext, const RenderData& renderData)
{
    FALCOR_PROFILE("Prepare ISMs");

    uint totalLightOrIsmCount = mSortingRules == (uint)SortingRules::LightFaces ? mTotalIsmCount : mTotalLightsCount;

    // Build CDF buffer on ReSTIR distribution  
    if (mAdaptiveISM)
    {
        FALCOR_PROFILE("Build CDF");

        auto pdfVars = mpBuildPDF->getRootVar();
        pdfVars["pdfCB"]["gSortingRules"] = mSortingRules;
        pdfVars["pdfCB"]["gIsmPerLight"] = mIsmPerLight;
        pdfVars["gLightHistogramBuffer"] = mpLightHistogramBuffer;
        pdfVars["gTotalValidPixels"] = mpTotalValidPixels;
        pdfVars["gLightPdfBuffer"] = mpLightPdfBuffer;
        pdfVars["gLightShadowDataBuffer"] = mpLightShadowDataBuffer;
        //mpPixelDebug->prepareProgram(mpBuildPDF->getProgram(), pdfVars);
        mpBuildPDF->execute(pRenderContext, totalLightOrIsmCount, 1);

        auto cdfVars = mpBuildCDF->getRootVar();
        cdfVars["gLightPdfBuffer"] = mpLightPdfBuffer;
        cdfVars["gLightCdfBuffer"] = mpLightCdfBuffer;
        //mpPixelDebug->prepareProgram(mpBuildCDF->getProgram(), cdfVars);
        mpBuildCDF->execute(pRenderContext, totalLightOrIsmCount, 1);
    }

    if (mRenderIsmCS)
    {
        FALCOR_PROFILE("ISM Render CS");

        uint totalTriangleCount = (uint)mPassData.scene->getSceneStats().instancedTriangleCount;
        uint launchWidth = 4096u;
        uint launchHeight = totalTriangleCount / launchWidth + 1u;
        uint totalPasses = mIsmSize / mIsmMinSize;

        // Clear the texture first
        for (uint i = 0; i < mSortedLightsISMsUint.size(); i++)
            pRenderContext->clearUAV(mSortedLightsISMsUint[i]->getUAV().get(), uint4(1065353216)); // = asuint(1.0f)

        for (uint pass = 0; pass < totalPasses; pass++)
        {
            // Clear all counting buffer for each frame
            pRenderContext->clearUAV(mpCounterBuffer->getUAV().get(), uint4(0));
            pRenderContext->clearUAV(mpExtraPointsCountBuffer->getUAV().get(), uint4(0));

            auto ismVars = mpIsmRenderPassCS->getRootVar();
            ismVars["ismCB"]["gBaseTriSize"] = mBaseTriSize;
            ismVars["ismCB"]["gTotalLightOrIsmCount"] = totalLightOrIsmCount;
            ismVars["ismCB"]["gTotalLightsCount"] = mTotalLightsCount;
            ismVars["ismCB"]["gIsmLinearProjection"] = mIsmLinearProjection;
            ismVars["ismCB"]["gIsmSize"] = mIsmSize;
            ismVars["ismCB"]["gLaunchWidth"] = launchWidth;
            ismVars["ismCB"]["gTotalTriangleCount"] = totalTriangleCount;
            ismVars["ismCB"]["gPassIndex"] = pass;
            ismVars["ismCB"]["gTotalPasses"] = totalPasses;
            ismVars["ismCB"]["gSortingRules"] = mSortingRules;
            ismVars["ismCB"]["gIsmPerLight"] = mIsmPerLight;
            ismVars["gLightShadowDataBuffer"] = mpLightShadowDataBuffer;
            ismVars["gTriOffsetBuffer"] = mpTriOffsetBuffer;
            ismVars["gExtraPointsCountBuffer"] = mpExtraPointsCountBuffer;
            ismVars["gCounterBuffer"] = mpCounterBuffer;
            for (uint i = 0; i < mSortedLightsISMsUint.size(); i++)
            {
                ismVars["gSortedLightsISMsUint"][i] = mSortedLightsISMsUint[i];
            }
        
            // Loop over all triangles
            mpPixelDebug->prepareProgram(mpIsmRenderPassCS->getProgram(), ismVars);
            mpIsmRenderPassCS->execute(pRenderContext, launchWidth, launchHeight);

            // Convert uint ISM texture to float
            auto convertVars = mpIsmUint2Float->getRootVar();
            convertVars["convertCB"]["gMaxIsmPerArray"] = kMaxIsmCountPerArray;
            for (uint i = 0; i < mSortedLightsISMsUint.size(); i++)
            {
                convertVars["gSortedLightsISMsUint"][i] = mSortedLightsISMsUint[i];
                convertVars["gSortedLightsISMs"][i] = mSortedLightsISMs[i];

                mpIsmUint2Float->execute(pRenderContext, mIsmSize, mIsmSize, mTotalIsmCount);
            }
        }
    }
    else
    {
        FALCOR_PROFILE("ISM Render Raster");

        GraphicsState::Viewport viewport(0.0f, 0.0f, (float)mIsmSize, (float)mIsmSize, 0.0f, 1.0f);
        mIsmRenderPass.pState->setViewport(0, viewport);

        // Clear all counting buffer for each frame
        pRenderContext->clearUAV(mpCounterBuffer->getUAV().get(), uint4(0));
        pRenderContext->clearUAV(mpExtraPointsCountBuffer->getUAV().get(), uint4(0));

        for (uint i = 0; i < mSortedLightsISMs.size(); i++)
        {
            auto arraySize = mSortedLightsISMs[i]->getArraySize();

            mIsmRenderPass.pFbo->attachColorTarget(mSortedLightsISMs[i], i, 0, 0, arraySize);
            pRenderContext->clearTexture(mSortedLightsISMs[i].get(), float4(1.0f));

            mIsmRenderPass.pState->setFbo(mIsmRenderPass.pFbo);
            auto pStagingDepthTexture = Texture::create2D(mIsmSize, mIsmSize, ResourceFormat::D32Float, arraySize, 1, nullptr, kShadowMapFlags);
            pRenderContext->clearTexture(pStagingDepthTexture.get(), float4(1.0f));
            mIsmRenderPass.pFbo->attachDepthStencilTarget(pStagingDepthTexture, 0, 0, arraySize); // Depth test needs this
            mIsmRenderPass.pState->setDepthStencilState(mpDepthTestDS);


            auto ismVars = mIsmRenderPass.pVars->getRootVar();
            ismVars["ismCB"]["gTotalLightsCount"] = mTotalLightsCount;
            ismVars["ismCB"]["gIsmPerLight"] = mIsmPerLight;
            ismVars["ismCB"]["gScreenSize"] = mPassData.screenSize;
            ismVars["ismCB"]["gBaseTriSize"] = mBaseTriSize;
            ismVars["ismCB"]["gAdaptiveISM"] = mAdaptiveISM;
            ismVars["ismCB"]["gIsmLightSamplingMode"] = mIsmLightSamplingMode;
            ismVars["ismCB"]["gIsmLinearProjection"] = mIsmLinearProjection;
            ismVars["ismCB"]["gCurrIsmTexArray"] = i;
            ismVars["gLightShadowDataBuffer"] = mpLightShadowDataBuffer;
            ismVars["gExtraPointsCountBuffer"] = mpExtraPointsCountBuffer;
            ismVars["gLightPdfBuffer"] = mpLightPdfBuffer;
            ismVars["gLightCdfBuffer"] = mpLightCdfBuffer;
            ismVars["gCounterBuffer"] = mpCounterBuffer;

            mpPixelDebug->prepareProgram(mIsmRenderPass.pProgram, ismVars);

            // Since we are rendering points, it doesn't matter if we cull back or not
            mPassData.scene->rasterize(pRenderContext, mIsmRenderPass.pState.get(), mIsmRenderPass.pVars.get(), RasterizerState::CullMode::Back); 
        }
    }

    // Do pull pass on ISMs
    //if (false)  
    {
        FALCOR_PROFILE("ISM Pull");
        for (uint m = 0; m < mSortedLightsISMs[0]->getMipCount() - 1; m++)
        {
            for (uint i = 0; i < mSortedLightsISMs.size(); i++)
            {
                const auto& pDefaultBlockReflection = mpIsmPullPass->getProgram()->getReflector()->getDefaultParameterBlock();
                auto srcTexBindLoc = pDefaultBlockReflection->getResourceBinding("gIsmInput");
                auto dstTexBindLoc = pDefaultBlockReflection->getResourceBinding("gIsmOutput");

                // Get src and dst's texture shared pointer
                auto pSrc = mSortedLightsISMs[i]->getSRV(m, 1);
                auto pDst = mSortedLightsISMs[i]->getUAV(m + 1);
                const uint32_t srcMipLevel = pSrc->getViewInfo().mostDetailedMip;
                const uint32_t dstMipLevel = pDst->getViewInfo().mostDetailedMip;
                Texture::SharedPtr pSrcTexture = std::static_pointer_cast<Texture>(pSrc->getResource());
                Texture::SharedPtr pDstTexture = std::static_pointer_cast<Texture>(pDst->getResource());

                auto pullVars = mpIsmPullPass->getRootVar();
                pullVars["PullCB"]["gDepthDiffThreshold"] = mDepthThreshold /* * (float)pow(2, m)*/;
                mpIsmPullPass->getVars()->setSrv(srcTexBindLoc, pSrc);
                mpIsmPullPass->getVars()->setUav(dstTexBindLoc, pDst);
                mpPixelDebug->prepareProgram(mpIsmPullPass->getProgram(), pullVars);
                mpIsmPullPass->execute(pRenderContext, mSortedLightsISMs[i]->getWidth(m + 1), mSortedLightsISMs[i]->getHeight(m + 1),
                    mSortedLightsISMs[i]->getArraySize());
            }
        }
    }

    {
        visualizeShadowMapTex(mVisualizePass.pVisualizeSingleIsm, mSortedLightsISMs, renderData["ism after pull"]->asTexture(), pRenderContext, mVisualizeMipLevel,
            mVisLightFaceID, "singleSM");
    }

    // Do push pass on ISMs
    //if (false)
    {
        FALCOR_PROFILE("ISM Push");
        for (uint m = mSortedLightsISMs[0]->getMipCount() - 1; m > 0; m--)
        {
            for (uint i = 0; i < mSortedLightsISMs.size(); i++)
            {
                const auto& pDefaultBlockReflection = mpIsmPushPass->getProgram()->getReflector()->getDefaultParameterBlock();
                auto srcTexBindLoc = pDefaultBlockReflection->getResourceBinding("gInput");
                auto dstTexBindLoc = pDefaultBlockReflection->getResourceBinding("gOutput");
                auto pSrc = mSortedLightsISMs[i]->getSRV(m, 1);
                auto pDst = mSortedLightsISMs[i]->getUAV(m - 1);

                auto pushVars = mpIsmPushPass->getRootVar();
                pushVars["PushCB"]["gPushMode"] = mIsmPushMode;
                pushVars["PushCB"]["gDepthDiffThreshold"] = mDepthThreshold;
                mpIsmPushPass->getVars()->setSrv(srcTexBindLoc, pSrc);
                mpIsmPushPass->getVars()->setUav(dstTexBindLoc, pDst);
                //mpPixelDebug->prepareProgram(mpIsmPushPass->getProgram(), pushVars);
                mpIsmPushPass->execute(pRenderContext, mSortedLightsISMs[i]->getWidth(m - 1), mSortedLightsISMs[i]->getHeight(m - 1),
                    mSortedLightsISMs[i]->getArraySize());
            }
        }
    }

    // Visualize ISM pass
    {
        visualizeShadowMapTex(mVisualizePass.pVisualizeSingleIsm, mSortedLightsISMs, renderData["ism after push"]->asTexture(), pRenderContext, mVisualizeMipLevel,
            mVisLightFaceID, "singleSM");

        // Get the proper texture to display
        uint resolution = mIsmSize * mIsmSize * mTotalIsmCount; 
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
        pRenderContext->clearTexture(pOutputTex.get(), float4(1.0f));
        //visualizeShadowMapTex(mVisualizePass.pVisualizeAll, mpIsmTextureArray, pOutputTex, pRenderContext, mVisualizeMipLevel,
        //    mVisLightID, "all");
        visualizeShadowMapTex(mVisualizePass.pVisualizeAll2, mSortedLightsISMs, pOutputTex, pRenderContext, mVisualizeMipLevel, mVisLightFaceID, "all");
    }

    // Print resource data
    if (mGpuDataToGet.getIsmData)
    {
        mValuesToPrint.extraPointsCount = getDeviceResourceData<uint>(pRenderContext, mpExtraPointsCountBuffer);

        std::string output;
        mValuesToPrint.totalExtraPoints = 0;
        mValuesToPrint.totalPoints = 0;
        for (uint i = 0; i < mValuesToPrint.extraPointsCount.size(); i++)
        {
            output += std::to_string(mValuesToPrint.extraPointsCount[i]) + " ";
            mValuesToPrint.totalExtraPoints += i * mValuesToPrint.extraPointsCount[i];
            mValuesToPrint.totalPoints += (i + 3) * mValuesToPrint.extraPointsCount[i];
        }
        //logInfo(std::format("extra Points Count buffer = [{}]", output));
    }
}

void RTXDITutorialBase::prepareStochasticShadowMaps(RenderContext* pRenderContext, const RenderData& renderData)
{
    FALCOR_PROFILE("Prepare shadow maps");

    uint totalBinCount = mSortingRules == (uint)SortingRules::LightFaces ? mTotalLightsCount * kShadowMapsPerLight : mTotalLightsCount;

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

    // TODO: We also need to clear the light shadow map data buffer using a compute pass. Only clear the light whose shadow map type != 1


    // Because we are directly adding to the histogram buffer, so in each frame we need to clear this buffer instead of accumulating the result
    pRenderContext->clearUAV(mpLightHistogramBuffer->getUAV().get(), uint4(0));

    // Get the sorted light index list
    computeTopLightsPass(pRenderContext);

    // General update for light shadow map data. This includes...
    {
        // 1. light center & matrix update due to mesh light changes in animation (not every frame).
        if (mPassData.updateEmissiveTriangleGeom || mPassData.updateLightPosition)
        {
            //logInfo("update light center & matrix called");

            // Compute new light shadow data
            auto newShadowData = prepareLightShadowMapData();
            
            // Pass new buffer to device
            auto pNewLightShadowDataBuffer = Buffer::createStructured(mpUpdateLightShadowDataCenter->getRootVar()["gLightShadowDataBuffer"], mTotalLightsCount,
                kBufferBindFlags, Buffer::CpuAccess::None, newShadowData.data());

            auto vars = mpUpdateLightShadowDataCenter->getRootVar();
            vars["CB"]["gShadowMapsPerLight"] = kShadowMapsPerLight;
            vars["gNewLightShadowDataBuffer"] = pNewLightShadowDataBuffer;
            vars["gLightShadowDataBuffer"] = mpLightShadowDataBuffer;
            mpPixelDebug->prepareProgram(mpUpdateLightShadowDataCenter->getProgram(), vars);
            mpUpdateLightShadowDataCenter->execute(pRenderContext, mTotalLightsCount, 1);
        }

        // 2. light current frame ranking + other general udpates (every frame)
        auto vars = mpPerFrameGeneralUpdates->getRootVar();
        vars["CB"]["gSortingRules"] = mSortingRules;
        vars["CB"]["gShadowMapsPerLight"] = kShadowMapsPerLight;
        vars["gSortedLightsBuffer"] = mpSortedLightsBuffer;
        vars["gLightShadowDataBuffer"] = mpLightShadowDataBuffer;
        mpPerFrameGeneralUpdates->execute(pRenderContext, totalBinCount, 1);
    }

    // Update corresponding light mesh data for those top N shadow map lights. 
    {
        FALCOR_PROFILE("Update Light Data");

        pRenderContext->clearUAV(mpIsmLightIndexBuffer->getUAV().get(), uint4(-1));
        pRenderContext->clearUAV(mpShadowMapSizeBuffer->getUAV().get(), uint4(0));

        auto vars = mpUpdateLightMeshData.getRootVar();
        vars["CB"]["gTotalBinCount"] = totalBinCount;
        vars["CB"]["gMaxReusingCount"] = mTopN * mTemporalReusingLength;
        vars["CB"]["gTopN"] = mTopN;
        vars["CB"]["gTemporalReusingLength"] = mTemporalReusingLength;
        vars["CB"]["gShadowMapsPerLight"] = kShadowMapsPerLight;
        vars["CB"]["gCurrFrameReusingStartIdx"] = mCurrFrameReusingStartIdx;
        vars["CB"]["gIsmPerLight"] = mIsmPerLight;
        vars["CB"]["gIsmSize"] = mIsmSize;
        vars["CB"]["gVisibility"] = (uint)mVisibility;
        vars["CB"]["gFullSizeShadowMaps"] = mFullSizeShadowMaps;
        vars["CB"]["gSceneName"] = mSceneName;
        vars["CB"]["gAgeThreshold"] = mLightAgeThreshold;
        vars["CB"]["gLimitSwitchRegion"] = mLimitSwitchRegionInUpdating;
        vars["CB"]["gMaxIsmPerArray"] = kMaxIsmCountPerArray;
        vars["CB"]["gSortingRules"] = mSortingRules;
        vars["CB"]["gMaxSupportedShadowMaps1024"] = kMaxSupportedLights1024 * kShadowMapsPerLight;
        vars["CB"]["gMaxPsmPerArray"] = kMaxPsmCountPerArray;
        vars["CB"]["gTemporalReusingFix"] = mTemporalReusingFix;
        vars["gSortedLightsBuffer"] = mpSortedLightsBuffer;
        vars["gReusingLightIndexBuffer"] = mpReusingLightIndexBuffer;
        vars["gIsmLightIndexBuffer"] = mpIsmLightIndexBuffer;
        vars["gLightShadowDataBuffer"] = mpLightShadowDataBuffer;
        vars["gShadowMapSizeBuffer"] = mpShadowMapSizeBuffer;
        vars["gTotalValidPixels"] = mpTotalValidPixels;
        vars["gRecordReusingLightData"] = mpRecordReusingLightData;
        mpPixelDebug->prepareProgram(mpUpdateLightMeshData->getProgram(), vars);
        mpUpdateLightMeshData->execute(pRenderContext, 2, 1); // change back to 1
    }

    // Cubic shadow maps. TODO: we don't need to render the same light if it is still in the shadow map list
    //  -> create an array only to store the unique light added into reusing list
    if (mVisibility == Visibility::ShadowMap_FullyLit || mVisibility == Visibility::ShadowMap_ISM)
    {
        FALCOR_PROFILE("Cubic Shadow Maps");

        uint maxSupportedShadowMaps1024 = kMaxSupportedLights1024 * kShadowMapsPerLight;
        uint totalPasses = mSortingRules == (uint)SortingRules::LightFaces ? mTopN / kShadowMapsPerLight : mTopN;

        auto shadowVars = mShadowMapPass.pVars->getRootVar();
        shadowVars["shadowMapCB"]["gVisMode"] = (uint)mVisibility;
        shadowVars["shadowMapCB"]["gDepthBias"] = mSmDepthBias;
        shadowVars["shadowMapCB"]["gShadowMapsPerLight"] = kShadowMapsPerLight;
        shadowVars["shadowMapCB"]["gSortingRules"] = mSortingRules;
        shadowVars["shadowMapCB"]["gTemporalReusingFix"] = mTemporalReusingFix;
        shadowVars["shadowMapCB"]["gHasDynamicObjects"] = true;
        shadowVars["shadowMapCB"]["gShadowMapSize"] = mShadowMapSize;
        shadowVars["gReusingLightIndexBuffer"] = mpReusingLightIndexBuffer;
        shadowVars["gLightShadowDataBuffer"] = mpLightShadowDataBuffer;

        if (mTemporalReusingFix == (uint)TemporalReusingFix::None)
        {
            for (uint pass = 0; pass < totalPasses; pass++)
            {
                uint psmTexArrayOffset = pass * kShadowMapsPerLight;
                uint currPassReusingStartIdx = mCurrFrameReusingStartIdx + (mSortingRules == (uint)SortingRules::LightFaces ? psmTexArrayOffset : pass);

                uint psmGlobalStartIdx = mSortingRules == (uint)SortingRules::LightFaces ? currPassReusingStartIdx : currPassReusingStartIdx * kShadowMapsPerLight;
                uint whichPsmTexArray = psmGlobalStartIdx < maxSupportedShadowMaps1024 ? 0u : (psmGlobalStartIdx - maxSupportedShadowMaps1024) / kMaxPsmCountPerArray + 1u;
                uint psmTexArrayStartIdx = psmGlobalStartIdx < maxSupportedShadowMaps1024 ? psmGlobalStartIdx :
                    (psmGlobalStartIdx - maxSupportedShadowMaps1024) % kMaxPsmCountPerArray;

                //if (mRtxdiFrameParams.frameIndex >= 100 && mRtxdiFrameParams.frameIndex <= 100)
                //{
                //    logInfo(std::format("currPassReusingStartIdx = {}, psmGlobalStartIdx = {}, psmTexArrayStartIdx = {}, mTopN = {}", currPassReusingStartIdx, psmGlobalStartIdx, psmTexArrayStartIdx, mTopN));
                //}

                shadowVars["shadowMapCB"]["gCurrPassReusingStartIdx"] = currPassReusingStartIdx;

                // Depth test needs this
                auto pStagingDepthTexture = Texture::create2D(mShadowMapSize, mShadowMapSize, ResourceFormat::D32Float, kShadowMapsPerLight, 1, nullptr, kShadowMapFlags);
                mShadowMapPass.pFbo->attachDepthStencilTarget(pStagingDepthTexture, 0, 0, kShadowMapsPerLight);
                pRenderContext->clearDsv(mShadowMapPass.pFbo->getDepthStencilView().get(), 1.0f, 0);

                // Render all light's PSM faces selected in this frame
                mShadowMapPass.pFbo->attachColorTarget(mSortedLightsShadowMaps[whichPsmTexArray], 0, 0, psmTexArrayStartIdx, kShadowMapsPerLight);
                mShadowMapPass.pState->setFbo(mShadowMapPass.pFbo);
                //pRenderContext->clearRtv(mShadowMapPass.pFbo->getRenderTargetView(0).get(), float4(1.0f));

                mpPixelDebug->prepareProgram(mShadowMapPass.pProgram, shadowVars);
                mPassData.scene->rasterize(pRenderContext, mShadowMapPass.pState.get(), mShadowMapPass.pVars.get(), RasterizerState::CullMode::None,
                    RasterizerState::DrawOption::All);
            }
        }
        else
        {
            // Only render the static objects for new PSM face in this frame's top N light faces
            {
                FALCOR_PROFILE("Static Instances Pass");

                for (uint pass = 0; pass < totalPasses; pass++)
                {
                    uint psmTexArrayOffset = pass * kShadowMapsPerLight;
                    uint currPassReusingStartIdx = mCurrFrameReusingStartIdx + (mSortingRules == (uint)SortingRules::LightFaces ? psmTexArrayOffset : pass);

                    uint psmGlobalStartIdx = mSortingRules == (uint)SortingRules::LightFaces ? currPassReusingStartIdx : currPassReusingStartIdx * kShadowMapsPerLight;
                    uint whichPsmTexArray = psmGlobalStartIdx < maxSupportedShadowMaps1024 ? 0u : (psmGlobalStartIdx - maxSupportedShadowMaps1024) / kMaxPsmCountPerArray + 1u;
                    uint psmTexArrayStartIdx = psmGlobalStartIdx < maxSupportedShadowMaps1024 ? psmGlobalStartIdx :
                        (psmGlobalStartIdx - maxSupportedShadowMaps1024) % kMaxPsmCountPerArray;

                    shadowVars["shadowMapCB"]["gCurrPassReusingStartIdx"] = currPassReusingStartIdx;
                    shadowVars["shadowMapCB"]["gHasDynamicObjects"] = false;

                    // Depth test needs this
                    auto pStagingDepthTexture = Texture::create2D(mShadowMapSize, mShadowMapSize, ResourceFormat::D32Float, kShadowMapsPerLight, 1, nullptr, kShadowMapFlags);
                    mShadowMapPass.pFbo->attachDepthStencilTarget(pStagingDepthTexture, 0, 0, kShadowMapsPerLight);
                    pRenderContext->clearDsv(mShadowMapPass.pFbo->getDepthStencilView().get(), 1.0f, 0);

                    // Render all light's PSM faces selected in this frame
                    mShadowMapPass.pFbo->attachColorTarget(mSortedLightsShadowMapsStatic[whichPsmTexArray], 0, 0, psmTexArrayStartIdx, kShadowMapsPerLight);
                    mShadowMapPass.pState->setFbo(mShadowMapPass.pFbo);


                    mpPixelDebug->prepareProgram(mShadowMapPass.pProgram, shadowVars);
                    mPassData.scene->rasterize(pRenderContext, mShadowMapPass.pState.get(), mShadowMapPass.pVars.get(), RasterizerState::CullMode::None,
                        RasterizerState::DrawOption::Static);
                }
            }

            // Copy static shadow maps to final shadow maps 
            //for (uint i = 0; i < mSortedLightsShadowMaps.size(); i++)
            //{
            //    pRenderContext->copyResource(mSortedLightsShadowMaps[i].get(), mSortedLightsShadowMapsStatic[i].get());
            //}

            // Render dynamic objects for all the reusing PSM faces
            {
                FALCOR_PROFILE("Dynamic Instances Pass");

                //totalPasses = mTemporalReusingLength * (mSortingRules == (uint)SortingRules::LightFaces ? mTopN / kShadowMapsPerLight : mTopN);
                for (uint pass = 0; pass < totalPasses; pass++)
                {
                    uint psmTexArrayOffset = pass * kShadowMapsPerLight;
                    uint currPassReusingStartIdx = mCurrFrameReusingStartIdx + (mSortingRules == (uint)SortingRules::LightFaces ? psmTexArrayOffset : pass);
                    //uint currPassReusingStartIdx = mSortingRules == (uint)SortingRules::LightFaces ? psmTexArrayOffset : pass;

                    uint psmGlobalStartIdx = mSortingRules == (uint)SortingRules::LightFaces ? currPassReusingStartIdx : currPassReusingStartIdx * kShadowMapsPerLight;
                    uint whichPsmTexArray = psmGlobalStartIdx < maxSupportedShadowMaps1024 ? 0u : (psmGlobalStartIdx - maxSupportedShadowMaps1024) / kMaxPsmCountPerArray + 1u;
                    uint psmTexArrayStartIdx = psmGlobalStartIdx < maxSupportedShadowMaps1024 ? psmGlobalStartIdx :
                        (psmGlobalStartIdx - maxSupportedShadowMaps1024) % kMaxPsmCountPerArray;

                    shadowVars["shadowMapCB"]["gCurrPassReusingStartIdx"] = currPassReusingStartIdx;
                    shadowVars["shadowMapCB"]["gHasDynamicObjects"] = true;

                    // Copy the static shadow map to depth texture
                    auto pStagingDepthTexture = Texture::create2D(mShadowMapSize, mShadowMapSize, ResourceFormat::D32Float, kShadowMapsPerLight, 1, nullptr,
                        kShadowMapFlags);
                    {
                        FALCOR_PROFILE("Copy to Depth Textures");
                        for (uint subIdx = 0; subIdx < kShadowMapsPerLight; subIdx++)
                        {
                            pRenderContext->copySubresource(pStagingDepthTexture.get(), subIdx,
                                mSortedLightsShadowMapsStatic[whichPsmTexArray].get(), psmTexArrayStartIdx + subIdx);
                            pRenderContext->copySubresource(mSortedLightsShadowMaps[whichPsmTexArray].get(), psmTexArrayStartIdx + subIdx,
                                mSortedLightsShadowMapsStatic[whichPsmTexArray].get(), psmTexArrayStartIdx + subIdx);
                        }
                    }
                    mShadowMapPass.pFbo->attachDepthStencilTarget(pStagingDepthTexture, 0, 0, kShadowMapsPerLight);

                    // Render all light's PSM faces selected in this frame
                    mShadowMapPass.pFbo->attachColorTarget(mSortedLightsShadowMaps[whichPsmTexArray], 0, 0, psmTexArrayStartIdx, kShadowMapsPerLight);
                    if (mTemporalReusingFix == (uint)TemporalReusingFix::MotionVector)
                    {
                        mShadowMapPass.pFbo->attachColorTarget(mSortedLightsMotionTextures[whichPsmTexArray], 1, 0, psmTexArrayStartIdx, kShadowMapsPerLight);
                        pRenderContext->clearRtv(mShadowMapPass.pFbo->getRenderTargetView(1).get(), float4(0, 0, 0, 1));
                    }
                    mShadowMapPass.pState->setFbo(mShadowMapPass.pFbo);

                    mpPixelDebug->prepareProgram(mShadowMapPass.pProgram, shadowVars);
                    mPassData.scene->rasterize(pRenderContext, mShadowMapPass.pState.get(), mShadowMapPass.pVars.get(), RasterizerState::CullMode::None,
                        RasterizerState::DrawOption::Dynamic);
                }
            }
        }
    }

    {
        FALCOR_PROFILE("Generate Shadow Mipmaps");
        for (uint i = 0; i < mSortedLightsShadowMaps.size(); i++)
        {
            mSortedLightsShadowMaps[i]->generateMips(pRenderContext);
        }
    }

    // Reconstruct the rest PSMs based on their prior motion texture (not exactly in previous frame)
    if (mTemporalReusingFix == (uint)TemporalReusingFix::MotionVector && mUseReconstructPSMs)
    {
        FALCOR_PROFILE("Reconstruct PSMs Pass");

        for (uint i = 0; i < mSortedLightsShadowMaps.size(); i++)
        //for (uint i = 0; i < 1; i++)
        {
            auto vars = mpReconstructPSMs->getRootVar();
            vars["reconstructCB"]["gCurrFrameReusingStartIdx"] = mCurrFrameReusingStartIdx;
            vars["reconstructCB"]["gTopN"] = mTopN;
            vars["reconstructCB"]["gShadowMapSize"] = mShadowMapSize;
            vars["reconstructCB"]["gShadowMapsPerLight"] = kShadowMapsPerLight;
            vars["gSortedPSMs"] = mSortedLightsShadowMaps[i];
            vars["gSortedMotionTextures"] = mSortedLightsMotionTextures[i];
            vars["gLightShadowDataBuffer"] = mpLightShadowDataBuffer;
            vars["gReusingLightIndexBuffer"] = mpReusingLightIndexBuffer;

            mpPixelDebug->prepareProgram(mpReconstructPSMs->getProgram(), vars);
            mpReconstructPSMs->execute(pRenderContext, mShadowMapSize, mShadowMapSize, mSortedLightsShadowMaps[i]->getArraySize());
        }

    }

    // Visualize the textures for debugging
    {
        visualizeShadowMapTex(mVisualizePass.pVisualizeSinglePsm, mSortedLightsShadowMaps, renderData["debugSM"]->asTexture(), pRenderContext, 0,
            mVisLightFaceID, "singleSM");

        if (mTemporalReusingFix != (uint)TemporalReusingFix::None)
        {
            visualizeShadowMapTex(mVisualizePass.pVisualizeSinglePsm, mSortedLightsShadowMapsStatic, renderData["staticSM"]->asTexture(), pRenderContext, 0,
                mVisLightFaceID, "singleSM");

            if (mTemporalReusingFix == (uint)TemporalReusingFix::MotionVector)
            {
                visualizeShadowMapTex(mVisualizePass.pVisualizeSingleMotionTex, mSortedLightsMotionTextures, renderData["debugMotionTex"]->asTexture(), pRenderContext, 0,
                    mVisLightFaceID, "singleMotionTex");
            }
        }
    }

    // Imperfect shadow maps
    if (mVisibility == Visibility::AllISM || mVisibility == Visibility::ShadowMap_ISM)
    {
        prepareIsms(pRenderContext, renderData);
    }

    // Move the light start index to next location
    uint totalReusingCount = mTopN * mTemporalReusingLength;
    mCurrFrameReusingStartIdx = (mCurrFrameReusingStartIdx + mTopN) % totalReusingCount;

    // Copy current light view matrices to previous
    {
        FALCOR_PROFILE("Update Previous View Matrices");

        auto viewVars = mpUpdatePrevViewMatrices->getRootVar();
        viewVars["CB"]["gShadowMapsPerLight"] = kShadowMapsPerLight;
        viewVars["gLightShadowDataBuffer"] = mpLightShadowDataBuffer;
        mpUpdatePrevViewMatrices->execute(pRenderContext, totalBinCount, 1);
    }

    // GPU data to print in target frame
    if (mRtxdiFrameParams.frameIndex >= 100 && mRtxdiFrameParams.frameIndex <= 100)
    {
        // Print light sampling statistics histogram
        //auto lightHistogramBuffer = getDeviceResourceData<uint>(pRenderContext, mpLightHistogramBuffer);
        //for (uint i = 0; i < lightHistogramBuffer.size(); i++)
        //{
        //    logInfo(std::format("mpLightHistogramBuffer[{}] = {}", i, lightHistogramBuffer[i]));
        //}

        // Print sorted light data
        auto sortedLightsBuffer = getDeviceResourceData<uint2>(pRenderContext, mpSortedLightsBuffer);
        uint sum = 0;
        for (uint i = 0; i < sortedLightsBuffer.size(); i++)
        {
            sum += sortedLightsBuffer[i].y;
            logInfo(std::format("Light Mesh ID {} = {}", sortedLightsBuffer[i].x, sortedLightsBuffer[i].y));
        }
        logInfo(std::format("sum = {}, # of pixels = {}", sum, mPassData.screenSize.x * mPassData.screenSize.y));

        // Print total valid pixels on screen
        auto totalValidPixels = getDeviceResourceData<uint>(pRenderContext, mpTotalValidPixels);
        logInfo(std::format("Total Valid Pixels = {}", totalValidPixels[0]));

        // Compute all shadow maps memory size
        mValuesToPrint.totalSmTexSize = 0;
        for (uint i = 0; i < mSortedLightsShadowMaps.size(); i++)
        {
            mValuesToPrint.totalSmTexSize += mSortedLightsShadowMaps[i]->getTextureSizeInBytes();
        }

        // Compute all ISMs memory size
        mValuesToPrint.totalIsmTexSize = 0;
        for (uint i = 0; i < mSortedLightsISMs.size(); i++)
        {
            mValuesToPrint.totalIsmTexSize += mSortedLightsISMs[i]->getTextureSizeInBytes();
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
        //logInfo(std::format("PDF Array = [\n{}\n]", pdfOutput));
        //logInfo(std::format("CDF Array = [\n{}\n]", cdfOutput));

        // Light primitive distribution
        auto primCounterArray = getDeviceResourceData<uint>(pRenderContext, mpCounterBuffer);
        std::string output = "";
        for (uint i = 0; i < primCounterArray.size(); i++)
        {
            output += std::to_string(primCounterArray[i]) + " ";
            output += (i + 1) % 10 == 0 ? "\n" : "";
        }
        //logInfo(std::format("Light point distribution = [\n{}\n]", output));
    }

    if (mGpuDataToGet.getSortingData)
    {
        std::string output;

        // Sorted lights array in each frame
        auto sortedLightIndexArray = getDeviceResourceData<uint2>(pRenderContext, mpSortedLightsBuffer);
        output = "";
        mValuesToPrint.totalNonZeroLightOrFace = 0u;
        for (uint i = 0; i < sortedLightIndexArray.size(); i++)
        {
            mValuesToPrint.totalNonZeroLightOrFace += sortedLightIndexArray[i].y != 0 ? 1 : 0;
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

        // Trick the ranking for some lights in Bistro
        mValuesToPrint.lightRankingAcrossFrames = "";
        std::vector<uint> trackingLights = { 257, 261, 262, 488, 489, 490 };
        for (uint j = 0; j < trackingLights.size(); j++)
        {
            for (uint i = 0; i < sortedLightIndexArray.size(); i++)
            {
                if (sortedLightIndexArray[i].x == trackingLights[j])
                {
                    mValuesToPrint.lightRankingAcrossFrames += std::format("Light #{}, ranking = {}, number of pixels = {}\n", sortedLightIndexArray[i].x, i,
                        sortedLightIndexArray[i].y);
                }
            }
        }
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

    if (mGpuDataToGet.getReusingLightData)
    {
        auto recordReusingLightData = getDeviceResourceData<uint4>(pRenderContext, mpRecordReusingLightData);
        mValuesToPrint.reusingLightAge = ""; 
        mValuesToPrint.reusingLightRanking = "";
        mValuesToPrint.reusingLightFrequency = "";

        for (uint i = 0; i < recordReusingLightData.size(); i++)
        {
            mValuesToPrint.reusingLightAge += std::to_string(recordReusingLightData[i].x) + " ";
            mValuesToPrint.reusingLightAge += (i + 1) % 10 == 0 ? "\n" : "";

            mValuesToPrint.reusingLightRanking += std::to_string(recordReusingLightData[i].y) + " ";
            mValuesToPrint.reusingLightRanking += (i + 1) % 10 == 0 ? "\n" : "";

            mValuesToPrint.reusingLightFrequency += std::to_string(recordReusingLightData[i].z) + " ";
            mValuesToPrint.reusingLightFrequency += (i + 1) % 5 == 0 ? "\n" : "";
        }
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
        for (uint i = 0; i < mTopN; i++)
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
        baselineVars["BaselineCB"]["gHeroLightsCount"] = mTopN;
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

void RTXDITutorialBase::visualizeShadowMapTex(ComputePass::SharedPtr pPass, std::vector<Texture::SharedPtr>& pSrcTextureArray, Texture::SharedPtr pDstTexture,
    RenderContext* pRenderContext, uint mipLevel, uint2 lightFaceIndex, const std::string& mode)
{
    uint2 nThreads = uint2(pSrcTextureArray[0]->getWidth(), pSrcTextureArray[0]->getHeight());

    // Bind common shader variables
    auto visualizeVars = pPass->getRootVar();
    visualizeVars["VisCB"]["gPointSampler"] = mpPointSampler;
    visualizeVars["VisCB"]["gLinearSampler"] = mpLinearSampler;
    visualizeVars["VisCB"]["gInputSize"] = nThreads;
    visualizeVars["VisCB"]["gIsmPerLight"] = mIsmPerLight;
    visualizeVars["VisCB"]["gIsmLinearProj"] = mIsmLinearProjection;
    visualizeVars["VisCB"]["gShadowMapsPerLight"] = mIsmLinearProjection;
    visualizeVars["VisCB"]["gSortingRules"] = mSortingRules;
    visualizeVars["gLightShadowDataBuffer"] = mpLightShadowDataBuffer;
    visualizeVars["gOutput"] = pDstTexture;
    visualizeVars["gIsmLightIndexBuffer"] = mpIsmLightIndexBuffer;

    for (uint i = 0; i < pSrcTextureArray.size(); i++)
    {
        if (mode == "singleMotionTex")
            visualizeVars["gInputSortedMotionTex"][i] = pSrcTextureArray[i];
        else
            visualizeVars["gInputSortedSMs"][i] = pSrcTextureArray[i];
    }

    if (mode == "singleMotionTex" || mode == "singleSM")
    {
        visualizeVars["VisCB"]["gMipLevel"] = (float)mipLevel;
        visualizeVars["VisCB"]["gLightFaceIndex"] = lightFaceIndex;

        mpPixelDebug->prepareProgram(pPass->getProgram(), visualizeVars);
        pPass->execute(pRenderContext, nThreads.x, nThreads.y);
    }
    else
    {
        visualizeVars["VisCB"]["gOutputSize"] = uint2(pDstTexture->getWidth(), pDstTexture->getHeight());
        mpPixelDebug->prepareProgram(pPass->getProgram(), visualizeVars);
        pPass->execute(pRenderContext, nThreads.x, nThreads.y, mTotalIsmCount);
    }
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
