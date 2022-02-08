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
#include "AreaLightReSTIR.h"
#include "Helpers.h"
#include "ShadingDataLoader.h"
#include <RenderGraph\RenderPassHelpers.h>

namespace
{
    const char kDesc[] = "ReSTIR Pass";

    const ChannelList kInputChannels =
    {
        { "vbuffer",    "",     "Visibility buffer in packed format",   false, HitInfo::kDefaultFormat }, // ResourceFormat::RG32Uint
        { "motionVecs", "",     "Motion vectors in screen-space",       false, ResourceFormat::RG32Float }
    };

    const char kDepthName[] = "depth"; // For depth debug
    const char kLinearDepthName[] = "linearDepth";
    const ChannelList kOutputChannels =
    {
        { "color",      "",     "Output color", false, ResourceFormat::RGBA16Float},
    };

    const char kEnableTemporalResampling[] = "enableTemporalResampling";
    const char kEnableSpatialResampling[] = "enableSpatialResampling";
    const char kStoreFinalVisibility[] = "storeFinalVisibility";

    const uint32_t kShadowMapSize = 1 << 10;
}

// Don't remove this. it's required for hot-reload to function properly
extern "C" __declspec(dllexport) const char* getProjDir()
{
    return PROJECT_DIR;
}

extern "C" __declspec(dllexport) void getPasses(Falcor::RenderPassLibrary& lib)
{
    lib.registerClass("AreaLightReSTIR", kDesc, AreaLightReSTIR::create);
}

AreaLightReSTIR::SharedPtr AreaLightReSTIR::create(RenderContext* pRenderContext, const Dictionary& dict)
{
    SharedPtr pPass = SharedPtr(new AreaLightReSTIR);
    for (const auto& [key, value] : dict)
    {
        if (key == kEnableTemporalResampling) pPass->mEnableTemporalResampling = value;
        else if (key == kEnableSpatialResampling) pPass->mEnableSpatialResampling = value;
        else if (key == kStoreFinalVisibility) pPass->mStoreFinalVisibility = value;
        else logWarning("Unknown field '" + key + "' in SVGFPass dictionary");
    }
    return pPass;
}

std::string AreaLightReSTIR::getDesc() { return kDesc; }

Dictionary AreaLightReSTIR::getScriptingDictionary()
{
    Dictionary dict;
    dict[kEnableTemporalResampling] = mEnableTemporalResampling;
    dict[kEnableSpatialResampling] = mEnableSpatialResampling;
    dict[kStoreFinalVisibility] = mStoreFinalVisibility;
    return dict;
}

RenderPassReflection AreaLightReSTIR::reflect(const CompileData& compileData)
{
    // Define the required resources here
    RenderPassReflection reflector;
    addRenderPassInputs(reflector, kInputChannels);
    addRenderPassOutputs(reflector, kOutputChannels);
    reflector.addOutput(kDepthName, "Depth value").format(ResourceFormat::D32Float).bindFlags(Resource::BindFlags::DepthStencil | Resource::BindFlags::ShaderResource)
        .texture2D(kShadowMapSize, kShadowMapSize, 0);
    reflector.addOutput(kLinearDepthName, "Linear depth value");
    return reflector;
}

void AreaLightReSTIR::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    if (mpScene == nullptr)
    {
        return;
    }

    mpPixelDebug->beginFrame(pRenderContext, renderData.getDefaultTextureDims());

    if (mpVBufferPrev == nullptr)
    {
        auto VBuffer = renderData["vbuffer"]->asTexture();
        mpVBufferPrev = Texture::create2D(VBuffer->getWidth(), VBuffer->getHeight(), VBuffer->getFormat(), VBuffer->getArraySize(), VBuffer->getMipCount(), nullptr, Resource::BindFlags::ShaderResource);
        pRenderContext->copyResource(mpVBufferPrev.get(), VBuffer.get());
    }

    // Update when parameters change
    if (mNeedUpdateDefines)
    {
        setSceneAreaLight(mpScene, mCurrentScene, mLightPos, mLightSize, mRotation);
        calcLightSpaceMatrix();
        UpdateDefines();
    }

    // Get image width and height
    uint32_t renderWidth = gpFramework->getTargetFbo()->getWidth();
    uint32_t renderHeight = gpFramework->getTargetFbo()->getHeight();

    // Get number of blocks for each dimension
    uint32_t renderWidthBlocks = (renderWidth + 16 - 1) / 16;
    uint32_t renderHeightBlocks = (renderHeight + 16 - 1) / 16;

    // Distance between each rows (number of pixels in each row = blockDim * blockSize) 
    uint reservoirBlockRowPitch = renderWidthBlocks * (16 * 16);

    // The total size for reservoir buffer = row size * height
    uint reservoirArrayPitch = reservoirBlockRowPitch * renderHeightBlocks;

    if (mpReservoirBuffer == nullptr)
    {
        const uint32_t reservoirLayers = 2; // current + previous
        mpReservoirBuffer = Buffer::createStructured(mpShadingPass["gReservoirs"], reservoirArrayPitch * reservoirLayers);
        mpReservoirBuffer->setName("ReSTIR: Reservoir Buffer");
    }

    mLastFrameOutputReservoir = mCurrentFrameOutputReservoir; // 0 

    uint32_t initialOutputBufferIndex = !mLastFrameOutputReservoir; // 1
    uint32_t temporalInputBufferIndex = mLastFrameOutputReservoir; // 0
    uint32_t temporalOutputBufferIndex = initialOutputBufferIndex; // 1
    uint32_t spatialInputBufferIndex = temporalOutputBufferIndex; // 1
    uint32_t spatialOutputBufferIndex = !spatialInputBufferIndex; // 0
    uint32_t shadeInputBufferIndex = mEnableSpatialResampling ? spatialOutputBufferIndex : temporalOutputBufferIndex; // 0 / 1

    mCurrentFrameOutputReservoir = shadeInputBufferIndex;

    // Shadow pass
    if (mActiveTargetPdf == 1 || mActiveShadingMode != 0)
    {
        PROFILE("Shadow Map");
        const auto& pDepthTex = renderData[kDepthName]->asTexture();
        const auto& pLinearDepthTex = renderData[kLinearDepthName]->asTexture();

        auto& pFbo = mShadowMapPass.pFbo;
        pFbo->attachDepthStencilTarget(pDepthTex);
        pFbo->attachColorTarget(pLinearDepthTex, 0);
        pRenderContext->clearFbo(pFbo.get(), float4(0.0f), 1.0f, 0); // clear all components (RTV, DSV)
        mShadowMapPass.pState->setFbo(pFbo);

        auto vars = mShadowMapPass.pVars;
        vars["PerFrameCB"]["gLightSpaceMat"] = mLightSpaceMat;
        vars["PerFrameCB"]["nearZ"] = mLightNearPlane;
        vars["PerFrameCB"]["farZ"] = mLightFarPlane;

        //mpPixelDebug->prepareProgram(mShadowMapPass.pProgram, mShadowMapPass.pVars->getRootVar());
        mpScene->rasterize(pRenderContext, mShadowMapPass.pState.get(), mShadowMapPass.pVars.get(), RasterizerState::CullMode::None);
    }

    // PCSS shading pass
    {

    }

    {
        PROFILE("Initial Sampling");
        auto cb = mpInitialSamplingPass["CB"]; // it is a ParameterBlockSharedPtr, we can use [] to get shader var
        cb["gViewportDims"] = uint2(gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
        cb["gFrameIndex"] = (uint)gpFramework->getGlobalClock().getFrame();
        cb["gParams"]["reservoirBlockRowPitch"] = reservoirBlockRowPitch;
        cb["gParams"]["reservoirArrayPitch"] = reservoirArrayPitch;
        cb["gOutputBufferIndex"] = initialOutputBufferIndex;
        ShadingDataLoader::setShaderData(renderData, mpVBufferPrev, cb["gShadingDataLoader"]);
        setPCSSShaderData(cb["gPCSS"]);
        mpScene->setRaytracingShaderData(pRenderContext, mpInitialSamplingPass->getRootVar()); // for binding resources of inline ray tracing 
        mpInitialSamplingPass["gReservoirs"] = mpReservoirBuffer;
        mpPixelDebug->prepareProgram(mpInitialSamplingPass->getProgram(), mpInitialSamplingPass->getRootVar());
        mpInitialSamplingPass->execute(pRenderContext, gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
    }

    if (mEnableTemporalResampling)
    {
        PROFILE("Temporal Resampling");
        auto cb = mpTemporalResamplingPass["CB"];
        cb["gViewportDims"] = uint2(gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
        cb["gFrameIndex"] = (uint)gpFramework->getGlobalClock().getFrame();
        cb["gParams"]["reservoirBlockRowPitch"] = reservoirBlockRowPitch;
        cb["gParams"]["reservoirArrayPitch"] = reservoirArrayPitch;
        cb["gInputBufferIndex"] = initialOutputBufferIndex; // 1
        cb["gHistoryBufferIndex"] = temporalInputBufferIndex; // 0
        cb["gOutputBufferIndex"] = temporalOutputBufferIndex; // 1
        ShadingDataLoader::setShaderData(renderData, mpVBufferPrev, cb["gShadingDataLoader"]);
        setPCSSShaderData(cb["gPCSS"]);
        mpScene->setRaytracingShaderData(pRenderContext, mpTemporalResamplingPass->getRootVar());
        mpTemporalResamplingPass["gReservoirs"] = mpReservoirBuffer;
        //mpPixelDebug->prepareProgram(mpTemporalResamplingPass->getProgram(), mpTemporalResamplingPass->getRootVar());
        mpTemporalResamplingPass->execute(pRenderContext, gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
    }

    if (mEnableSpatialResampling)
    {
        PROFILE("Spatial Resampling");
        auto cb = mpSpatialResamplingPass["CB"];
        cb["gViewportDims"] = uint2(gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
        cb["gFrameIndex"] = (uint)gpFramework->getGlobalClock().getFrame();
        cb["gParams"]["reservoirBlockRowPitch"] = reservoirBlockRowPitch;
        cb["gParams"]["reservoirArrayPitch"] = reservoirArrayPitch;
        cb["gInputBufferIndex"] = spatialInputBufferIndex;
        cb["gOutputBufferIndex"] = spatialOutputBufferIndex;
        ShadingDataLoader::setShaderData(renderData, mpVBufferPrev, cb["gShadingDataLoader"]);
        mpScene->setRaytracingShaderData(pRenderContext, mpSpatialResamplingPass->getRootVar());
        setPCSSShaderData(cb["gPCSS"]);
        mpSpatialResamplingPass["gReservoirs"] = mpReservoirBuffer;
        mpSpatialResamplingPass["gNeighborOffsetBuffer"] = mpNeighborOffsetBuffer;
        //mpPixelDebug->prepareProgram(mpSpatialResamplingPass->getProgram(), mpSpatialResamplingPass->getRootVar());
        mpSpatialResamplingPass->execute(pRenderContext, gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
    }

    {
        PROFILE("Shading");
        auto cb = mpShadingPass["CB"];
        cb["gViewportDims"] = uint2(gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
        cb["gFrameIndex"] = (uint)gpFramework->getGlobalClock().getFrame();
        cb["gParams"]["reservoirBlockRowPitch"] = reservoirBlockRowPitch;
        cb["gParams"]["reservoirArrayPitch"] = reservoirArrayPitch;
        cb["gInputBufferIndex"] = shadeInputBufferIndex;
        ShadingDataLoader::setShaderData(renderData, mpVBufferPrev, cb["gShadingDataLoader"]);
        mpScene->setRaytracingShaderData(pRenderContext, mpShadingPass->getRootVar());
        setPCSSShaderData(cb["gPCSS"]);
        mpShadingPass["gReservoirs"] = mpReservoirBuffer;
        mpShadingPass["gShadingOutput"] = renderData[kOutputChannels[0].name]->asTexture();
        //mpPixelDebug->prepareProgram(mpShadingPass->getProgram(), mpShadingPass->getRootVar());
        mpShadingPass->execute(pRenderContext, gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
    }

    {
        auto VBuffer = renderData["vbuffer"]->asTexture();
        pRenderContext->copyResource(mpVBufferPrev.get(), VBuffer.get());
    }

    mpPixelDebug->endFrame(pRenderContext);
}

void AreaLightReSTIR::renderUI(Gui::Widgets& widget)
{
    bool dirty = false;

    dirty |= widget.checkbox("Brute Force", mBruteForce);
    dirty |= widget.var("Blocker search samples", mBlockerSearchSamples);
    dirty |= widget.var("PCF samples", mPCFSamples);
    dirty |= widget.var("Raster Alpha Test", mRasterAlphaTest);
    dirty |= widget.var("Minimum Visibility", mMinVisibility);
    dirty |= widget.var("Depth Bias", mDepthBias);
    dirty |= widget.var("New PCSS Samples", mNewPCSSSamples);
    
    const Gui::RadioButtonGroup targetPdfs = {
        {0, "Unshadowed", true},
        {1, "Shadow Map", true},
        {2, "Shadow Ray", true}
    };
    widget.text("Target Pdf: ");
    dirty |= widget.radioButtons(targetPdfs, mActiveTargetPdf);

    const Gui::RadioButtonGroup shadingModes = {
        {0, "Ray Tracing", true},
        {1, "New PCSS", true},
        {2, "PCSS", true}
    };
    widget.text("Shading Mode: ");
    dirty |= widget.radioButtons(shadingModes, mActiveShadingMode);


    // Light configuration
    if (auto group = widget.group("Area Light"))
    {
        dirty |= group.var("Light near plane", mLightNearPlane);
        dirty |= group.var("Light far plane", mLightFarPlane);
        dirty |= group.var("Light FovY", mFovY);
        dirty |= group.var("Light Size", mLightSize);
        dirty |= group.var("Light center position", mLightPos);
        dirty |= group.var("Light center up", mLightUp);
        dirty |= group.var("Light rotation", mRotation);
    }

    if (auto group = widget.group("Initial Sampling"))
    {
        /*dirty |= group.var("Initial Emissive Triangle Samples", mInitialEmissiveTriangleSamples, 1u, 32u);
        dirty |= group.var("Initial EnvMap Samples", mInitialEnvMapSamples, 1u, 32u);*/

        dirty |= group.var("Initial Area Light Samples", mInitialAreaLightSamples, 1u, 128u);
    }

    if (auto group = widget.group("Temporal Resampling"))
    {
        dirty |= group.checkbox("Enable Temporal Resampling", mEnableTemporalResampling);
    }

    if (auto group = widget.group("Spatial Resampling"))
    {
        dirty |= group.checkbox("Enable Spatial Resampling", mEnableSpatialResampling);
    }

    if (auto group = widget.group("Shading"))
    {
        dirty |= group.checkbox("Store Final Visibility", mStoreFinalVisibility);
    }

    mpPixelDebug->renderUI(widget);

    mNeedUpdateDefines = dirty;
}

void AreaLightReSTIR::setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene)
{
    mpScene = pScene;
    Scene::RenderSettings renderSettings;
    renderSettings.useEmissiveLights = false;
    mpScene->setRenderSettings(renderSettings);

    // Adjust rectangle light if exist
    setSceneAreaLight(mpScene, mCurrentScene, mLightPos, mLightSize, mRotation);

    mpSampleGenerator = SampleGenerator::create(SAMPLE_GENERATOR_UNIFORM);

    std::vector<uint8_t> offsets;
    offsets.resize(8192 * 2); // Q: what is this magic number?
    FillNeighborOffsetBuffer(offsets.data());
    mpNeighborOffsetBuffer = Buffer::createTyped(ResourceFormat::RG8Snorm, 8192, ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess, Buffer::CpuAccess::None, offsets.data());
    mpNeighborOffsetBuffer->setName("ReSTIR: Neighbor Offset Buffer");

    CreatePasses();
    calcLightSpaceMatrix();
}

AreaLightReSTIR::AreaLightReSTIR()
{
    mpPixelDebug = PixelDebug::create();
}

void AreaLightReSTIR::AddDefines(ComputePass::SharedPtr pass, Shader::DefineList& defines)
{
    pass->getProgram()->addDefines(defines);
    pass->setVars(nullptr);
}

void AreaLightReSTIR::RemoveDefines(ComputePass::SharedPtr pass, Shader::DefineList& defines)
{
    pass->getProgram()->removeDefines(defines);
    pass->setVars(nullptr);
}

// Create compute pass
void AreaLightReSTIR::CreatePass(ComputePass::SharedPtr& pass, const char* path, Program::DefineList& defines)
{
    Program::Desc desc;
    desc.addShaderLibrary(path).csEntry("main").setShaderModel("6_5");
    pass = ComputePass::create(desc, defines, false);
}

void AreaLightReSTIR::CreatePasses()
{
    // Create rasterizer pass
    {
        mShadowMapPass.pFbo = Fbo::create();

        Program::Desc desc;
        desc.addShaderLibrary("RenderPasses/AreaLightReSTIR/ShadowMap.slang").vsEntry("vsMain").psEntry("psMain");
        desc.setShaderModel("6_5");
        mShadowMapPass.pProgram = GraphicsProgram::create(desc);
        mShadowMapPass.pState = GraphicsState::create();
        mShadowMapPass.pState->setProgram(mShadowMapPass.pProgram);

        Program::DefineList defines;
        defines.add(mpScene->getSceneDefines());
        defines.add("_ALPHA_TEST_MODE", std::to_string(mRasterAlphaTest));

        mShadowMapPass.pProgram->addDefines(mpScene->getSceneDefines());
        mShadowMapPass.pVars = GraphicsVars::create(mShadowMapPass.pProgram->getReflector());
    }

    // Create compute passes 
    {
        Program::DefineList defines = mpScene->getSceneDefines();
        defines.add(mpSampleGenerator->getDefines());
        defines.add("_MS_DISABLE_ALPHA_TEST");
        defines.add("_DEFAULT_ALPHA_TEST");
        defines.add("_ALPHA_TEST_MODE", "1");
        defines.add("NEAR_PLANE", std::to_string(mLightNearPlane));
        defines.add("LIGHT_WORLD_SIZE", std::to_string(mLightSize));
        float frustumSize = 2.0f * mLightNearPlane * glm::tan(glm::radians(0.5f * mFovY));
        defines.add("LIGHT_FRUSTUM_SIZE", std::to_string(frustumSize));

        CreatePass(mpInitialSamplingPass, "RenderPasses/AreaLightReSTIR/InitialSampling.cs.slang", defines);
        //CreatePass(mpTemporalResamplingPass, "RenderPasses/AreaLightReSTIR/TemporalResampling.cs.slang", defines);
        //CreatePass(mpSpatialResamplingPass, "RenderPasses/AreaLightReSTIR/SpatialResampling.cs.slang", defines);
        CreatePass(mpShadingPass, "RenderPasses/AreaLightReSTIR/Shading.cs.slang", defines);
    }

    UpdateDefines();
}

void AreaLightReSTIR::UpdateDefines()
{
    Shader::DefineList PCSSDefines;
    const char* kTargetPdf = "_TARGET_PDF";
    PCSSDefines.add("_BLOCKER_SEARCH_SAMPLES", std::to_string(mBlockerSearchSamples));
    PCSSDefines.add("PCF_SAMPLES", std::to_string(mPCFSamples));
    PCSSDefines.add(kTargetPdf, std::to_string(mActiveTargetPdf));
    PCSSDefines.add("_MIN_VISIBILITY", std::to_string(mMinVisibility));
    PCSSDefines.add("_DEPTH_BIAS", std::to_string(mDepthBias));
    PCSSDefines.add("_NEW_PCSS_SAMPLES", std::to_string(mNewPCSSSamples));

    {
        const char* kInitialAreaLightSamples = "_INITIAL_AREA_LIGHT_SAMPLES";
        const char* kHasReusing = "_HAS_REUSING";
        Shader::DefineList defines;
        defines.add(kInitialAreaLightSamples, std::to_string(mInitialAreaLightSamples));
        defines.add(kHasReusing, std::to_string(mEnableSpatialResampling || mEnableTemporalResampling));
        defines.add(PCSSDefines);
        AddDefines(mpInitialSamplingPass, defines);
    }

    //{
    //    Shader::DefineList defines;
    //    defines.add(PCSSDefines);
    //    AddDefines(mpTemporalResamplingPass, defines);
    //}

    //{
    //    Shader::DefineList defines;
    //    defines.add(PCSSDefines);
    //    AddDefines(mpSpatialResamplingPass, defines);
    //}

    {
        const char* kStoreFinalVisibility = "_STORE_FINAL_VISIBILITY";
        const char* kBruteForce = "_BRUTE_FORCE";
        const char* kShadingMode = "_SHADING_MODE";
        PCSSDefines.add("_MIN_VISIBILITY", std::to_string(0.0f)); // No need to clamp  
        Shader::DefineList defines;
        defines.add(kStoreFinalVisibility, mStoreFinalVisibility ? "1" : "0");
        defines.add(kBruteForce, mBruteForce ? "1" : "0");
        defines.add(kShadingMode, std::to_string(mActiveShadingMode));
        defines.add(PCSSDefines);
        AddDefines(mpShadingPass, defines);
    }

    mNeedUpdateDefines = false;
}

void AreaLightReSTIR::calcLightSpaceMatrix()
{
    auto pLight = mpScene->getLightByName("Area Light");
    if (!pLight) return;

    // light pos and dir in world space
    auto lightData = pLight->getData();
    auto lightType = pLight->getType();
    float3 posW;
    float3 dirW;
    float4x4 lightView;
    float4x4 lightProj;

    if (lightType == LightType::Rect)
    {
        posW = mLightPos;
        dirW = lightData.transMatIT * float4(0.0f, 0.0f, 1.0f, 1.0f);
        lightView = glm::lookAt(posW, posW + dirW, mLightUp); // RH camera look direction is -z direction in view space
        lightProj = glm::perspective(glm::radians(mFovY), 1.0f, mLightNearPlane, mLightFarPlane);
    }
    else if (lightType == LightType::Distant)
    {
        posW = lightData.posW;
        dirW = lightData.dirW;
    }
    else if (lightType == LightType::Sphere)
    {
        logInfo("Sphere Light");
        posW = mLightPos;
        dirW = glm::rotate(glm::mat4(1.0f), glm::radians(mRotation.x), float3(mRotation.yzw)) * float4(0.0f, -1.0f, 0.0f, 1.0f);
        lightView = glm::lookAt(posW, posW + dirW, mLightUp); // RH camera look direction is -z direction in view space
        lightProj = glm::perspective(glm::radians(mFovY), 1.0f, mLightNearPlane, mLightFarPlane);
    }

    logInfo("light position: " + to_string(posW));
    logInfo("lightData position: " + to_string(lightData.posW));
    logInfo("light direction: " + to_string(dirW));
    logInfo("GLM_CONFIG_CLIP_CONTROL = " + std::to_string(GLM_CONFIG_CLIP_CONTROL)); // GLM_CLIP_CONTROL_RH_ZO   

    mLightSpaceMat = lightProj * lightView;
    mLightView = lightView;
    mLightProj = lightProj;
}

void AreaLightReSTIR::setPCSSShaderData(const ShaderVar& vars)
{
    vars["gLightSpaceMat"] = mLightSpaceMat;
    vars["gLightView"] = mLightView;
    vars["gLightProj"] = mLightProj;
    vars["gLightPos"] = mLightPos;
    vars["gShadowMapDim"] = float2(kShadowMapSize);
    vars["nearZ"] = mLightNearPlane;
    vars["farZ"] = mLightFarPlane;
    vars["gShadowMap"] = mShadowMapPass.pFbo->getDepthStencilTexture();

    // Create sampler for sampling shadow map texture
    Sampler::Desc samplerDesc;
    samplerDesc.setAddressingMode(Sampler::AddressMode::Border, Sampler::AddressMode::Border, Sampler::AddressMode::Border).setBorderColor(float4(1.0f)); // Outside sample points will not be shadowed
    samplerDesc.setFilterMode(Sampler::Filter::Linear, Sampler::Filter::Linear, Sampler::Filter::Point); // for comparsion texture
    samplerDesc.setComparisonMode(Sampler::ComparisonMode::Less); // lessEqual = no occluders
    vars["gSamplerCmp"] = Sampler::create(samplerDesc);

    samplerDesc.setFilterMode(Sampler::Filter::Linear, Sampler::Filter::Linear, Sampler::Filter::Linear);
    samplerDesc.setComparisonMode(Sampler::ComparisonMode::Disabled);
    vars["gSampler"] = Sampler::create(samplerDesc);
}
