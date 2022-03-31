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
    //const char kLinearDepthName[] = "linearDepth";
    const ChannelList kOutputChannels =
    {
        { "color",      "",     "Output color", false, ResourceFormat::RGBA16Float},
    };

    const char kEnableTemporalResampling[] = "enableTemporalResampling";
    const char kEnableSpatialResampling[] = "enableSpatialResampling";
    const char kStoreFinalVisibility[] = "storeFinalVisibility";

    const uint32_t kShadowMapSize = 1 << 10; // 1K
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
    //reflector.addOutput(kLinearDepthName, "Linear depth value");
    return reflector;
}

void AreaLightReSTIR::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    if (mpScene == nullptr)
    {
        return;
    }

    mpPixelDebug->beginFrame(pRenderContext, renderData.getDefaultTextureDims());

    auto pCamera = mpScene->getCamera();

    // Update when parameters change
    if (mNeedUpdate)
    {
        mLightParams.updateSceneAreaLight(mpScene);
        calcLightSpaceMatrix();
        UpdateDefines();
        setShadowMapPassFbo();

       /* pCamera->setPosition(mInitialCameraPos);
        pCamera->setTarget(mInitialCameraTarget);*/
    }

    if (pCamera->isAnimated())
    {
        uint currentFrame = (uint)gpFramework->getGlobalClock().getFrame();
        auto rotation = glm::rotate(glm::mat4(1.0f), glm::radians(0.1f), float3(0.0f, 1.0f, 0.0f));

        float3 cameraPos = pCamera->getPosition();
        float3 cameraForward = pCamera->getTarget() - cameraPos;
        float4 newCameraPos = rotation * float4(cameraPos, 1.0f);
        float4 newCameraForward = rotation * float4(cameraForward, 1.0f);
        float4x4 transform = float4x4(
            float4(1.0f, 0.0f, 0.0f, 0.0f),
            float4(pCamera->getUpVector(), 0.0f),
            -newCameraForward,
            newCameraPos
        );
        pCamera->updateFromAnimation(transform);
    }

    if (mpVBufferPrev == nullptr)
    {
        auto VBuffer = renderData["vbuffer"]->asTexture();
        mpVBufferPrev = Texture::create2D(VBuffer->getWidth(), VBuffer->getHeight(), VBuffer->getFormat(), VBuffer->getArraySize(), VBuffer->getMipCount(), nullptr, Resource::BindFlags::ShaderResource);
        pRenderContext->copyResource(mpVBufferPrev.get(), VBuffer.get());
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
    if (mShadowType != ShadowType::ShadowRay)
    {
        PROFILE("Shadow Map");
        const auto& pDepthTex = renderData[kDepthName]->asTexture();
        //const auto& pLinearDepthTex = renderData[kLinearDepthName]->asTexture();

        auto& pFbo = mShadowMapPass.pFbo;
        pFbo->attachDepthStencilTarget(pDepthTex);
        pRenderContext->clearFbo(pFbo.get(), float4(0.0f), 1.0f, 0); // clear all components (RTV, DSV)
        mShadowMapPass.pState->setFbo(pFbo);

        auto vars = mShadowMapPass.pVars;
        vars["PerFrameCB"]["gLightSpaceMat"] = mLightSpaceMat;
        vars["PerFrameCB"]["nearZ"] = mLightParams.mLightNearPlane;
        vars["PerFrameCB"]["farZ"] = mLightParams.mLightFarPlane;

        mpPixelDebug->prepareProgram(mShadowMapPass.pProgram, mShadowMapPass.pVars->getRootVar());
        mpScene->rasterize(pRenderContext, mShadowMapPass.pState.get(), mShadowMapPass.pVars.get(), RasterizerState::CullMode::None);
    }

    // Get shadow map texture and (generate mipmaps)
    auto shadowMapTex = mShadowMapPass.pFbo->getDepthStencilTexture();
    auto colorTex = mShadowMapPass.pFbo->getColorTexture(0); 
    if (colorTex != nullptr)
    {
        //colorTex->generateMips(pRenderContext); // TODO: remove this when using SAT
        if (mpRowSum == nullptr || mNeedUpdate)
        {
            mpRowSum = Texture::create2D(colorTex->getWidth(), colorTex->getHeight(), colorTex->getFormat(),
                colorTex->getArraySize(), 1, nullptr,
                Resource::BindFlags::ShaderResource | Resource::BindFlags::UnorderedAccess);
        }

        if (mpSAT == nullptr || mNeedUpdate)
        {
            mpSAT = Texture::create2D(colorTex->getWidth(), colorTex->getHeight(), colorTex->getFormat(),
                colorTex->getArraySize(), 1, nullptr,
                Resource::BindFlags::ShaderResource | Resource::BindFlags::UnorderedAccess);
        }

        //logInfo("colorTex = " + to_string(colorTex->getFormat()));
        //logInfo("mpRowSum = " + to_string(mpRowSum->getFormat()));
        //logInfo("mpSAT = " + to_string(mpSAT->getFormat()));
    }

    // SAT passes
    if (mShadowType == ShadowType::VSM || mShadowType == ShadowType::EVSM || mShadowType == ShadowType::MSM)
    {
        PROFILE("SAT Generation");

        auto vars = mSATScanPasses.mpSATScan1->getRootVar();
        vars["gInput"] = colorTex;
        vars["gOutput"] = mpRowSum;
        mpPixelDebug->prepareProgram(mSATScanPasses.mpSATScan1->getProgram(), mSATScanPasses.mpSATScan1->getRootVar());
        mSATScanPasses.mpSATScan1->execute(pRenderContext, kShadowMapSize, kShadowMapSize);

        vars = mSATScanPasses.mpSATScan2->getRootVar();
        vars["gInput"] = mpRowSum;
        vars["gOutput"] = mpSAT;
        //mpPixelDebug->prepareProgram(mSATScanPasses.mpSATScan2->getProgram(), mSATScanPasses.mpSATScan2->getRootVar());
        mSATScanPasses.mpSATScan2->execute(pRenderContext, kShadowMapSize, kShadowMapSize);
    }

    LightParams lightParams = { mLightSpaceMat, mLightView, mLightProj, mLightParams.mLightPos };

    // ReSTIR passes
    if (mShadowType == ShadowType::ShadowRay || mShadowType == ShadowType::NewPCSSReSTIR)
    {
        {
            PROFILE("Initial Sampling");
            auto cb = mpInitialSamplingPass["CB"]; // it is a ParameterBlockSharedPtr, we can use [] to get shader var
            cb["gViewportDims"] = uint2(gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
            cb["gFrameIndex"] = (uint)gpFramework->getGlobalClock().getFrame();
            cb["gParams"]["reservoirBlockRowPitch"] = reservoirBlockRowPitch;
            cb["gParams"]["reservoirArrayPitch"] = reservoirArrayPitch;
            cb["gOutputBufferIndex"] = initialOutputBufferIndex;
            cb["gSamplerCmp"] = mpSamplerCmp;
            cb["gLinearSampler"] = mpTrilinearSampler;
            cb["gPointSampler"] = mpPointSampler;
            mpInitialSamplingPass["gShadowMap"] = shadowMapTex;
            mpInitialSamplingPass["gReservoirs"] = mpReservoirBuffer;
            ShadingDataLoader::setShaderData(renderData, mpVBufferPrev, cb["gShadingDataLoader"]);
            lightParams.setShaderData(cb["gLightParams"]);
            mpScene->setRaytracingShaderData(pRenderContext, mpInitialSamplingPass->getRootVar()); // for binding resources of inline ray tracing 
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
            cb["gSamplerCmp"] = mpSamplerCmp;
            cb["gLinearSampler"] = mpTrilinearSampler;
            cb["gPointSampler"] = mpPointSampler;
            mpTemporalResamplingPass["gReservoirs"] = mpReservoirBuffer;
            mpTemporalResamplingPass["gShadowMap"] = shadowMapTex;
            ShadingDataLoader::setShaderData(renderData, mpVBufferPrev, cb["gShadingDataLoader"]);
            lightParams.setShaderData(cb["gLightParams"]);
            mpScene->setRaytracingShaderData(pRenderContext, mpTemporalResamplingPass->getRootVar());
            mpPixelDebug->prepareProgram(mpTemporalResamplingPass->getProgram(), mpTemporalResamplingPass->getRootVar());
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
            cb["gSamplerCmp"] = mpSamplerCmp;
            cb["gLinearSampler"] = mpTrilinearSampler;
            cb["gPointSampler"] = mpPointSampler;
            mpSpatialResamplingPass["gReservoirs"] = mpReservoirBuffer;
            mpSpatialResamplingPass["gNeighborOffsetBuffer"] = mpNeighborOffsetBuffer;
            mpSpatialResamplingPass["gShadowMap"] = shadowMapTex;
            ShadingDataLoader::setShaderData(renderData, mpVBufferPrev, cb["gShadingDataLoader"]);
            lightParams.setShaderData(cb["gLightParams"]);
            mpScene->setRaytracingShaderData(pRenderContext, mpSpatialResamplingPass->getRootVar());
            mpPixelDebug->prepareProgram(mpSpatialResamplingPass->getProgram(), mpSpatialResamplingPass->getRootVar());
            mpSpatialResamplingPass->execute(pRenderContext, gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
        }
    }

    // Need create another shading pass for VSM?
    {
        PROFILE("Shading");
        auto cb = mpShadingPass["CB"];
        cb["gViewportDims"] = uint2(gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
        cb["gFrameIndex"] = (uint)gpFramework->getGlobalClock().getFrame();
        cb["gParams"]["reservoirBlockRowPitch"] = reservoirBlockRowPitch;
        cb["gParams"]["reservoirArrayPitch"] = reservoirArrayPitch;
        cb["gInputBufferIndex"] = shadeInputBufferIndex;
        cb["gShadowType"] = (uint)mShadowType;
        cb["gSamplerCmp"] = mpSamplerCmp;
        cb["gLinearSampler"] = mpTrilinearSampler;
        cb["gPointSampler"] = mpPointSampler;
        mpShadingPass["gReservoirs"] = mpReservoirBuffer;
        mpShadingPass["gShadingOutput"] = renderData[kOutputChannels[0].name]->asTexture();
        mpShadingPass["gShadowMap"] = shadowMapTex;
        mpShadingPass["gVSM"] = colorTex;
        mpShadingPass["gSAT"] = mpSAT;
        ShadingDataLoader::setShaderData(renderData, mpVBufferPrev, cb["gShadingDataLoader"]);
        lightParams.setShaderData(cb["gLightParams"]);
        mpScene->setRaytracingShaderData(pRenderContext, mpShadingPass->getRootVar());
        mpPixelDebug->prepareProgram(mpShadingPass->getProgram(), mpShadingPass->getRootVar());
        mpShadingPass->execute(pRenderContext, gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());
    }

    {
        auto VBuffer = renderData["vbuffer"]->asTexture();
        pRenderContext->copyResource(mpVBufferPrev.get(), VBuffer.get());
    }

    mNeedUpdate = false;

    mpPixelDebug->endFrame(pRenderContext);
}

void AreaLightReSTIR::renderUI(Gui::Widgets& widget)
{
    bool dirty = false;

    // Light configuration
    if (auto group = widget.group("Area Light"))
    {
        dirty |= group.var("Light near plane", mLightParams.mLightNearPlane);
        dirty |= group.var("Light far plane", mLightParams.mLightFarPlane);
        dirty |= group.var("Light FovY", mLightParams.mFovY);
        dirty |= group.var("Light Size", mLightParams.mLightSize);
        dirty |= group.var("Light center position", mLightParams.mLightPos);
        dirty |= group.var("Light center up", mLightParams.mLightUp);
        dirty |= group.var("Light rotation", mLightParams.mRotation);
    }

    Gui::DropdownList samplesOption = {
        {4, "4"}, {8, "8"}, {16, "16"}, {32, "32"},
        {64, "64"}, {128, "128"}, {256, "256"}, {512, "512"},
        {1024, "1024"}
    };
    dirty |= widget.dropdown("Blocker search samples", samplesOption, mBlockerSearchSamples);
    dirty |= widget.dropdown("PCF samples", samplesOption, mPCFSamples);
    //dirty |= widget.var("Depth Bias", mDepthBias);
    
    if (mShadowType == ShadowType::ShadowRay || mShadowType == ShadowType::NewPCSSReSTIR)
    {
        dirty |= widget.checkbox("Brute Force", mBruteForce);    
        const Gui::RadioButtonGroup targetPdfs = {
            {0, "Unshadowed", true},
            {1, "New PCSS", true},
            {2, "Shadow Ray", true}
        };
        widget.text("Target Pdf: ");
        dirty |= widget.radioButtons(targetPdfs, mActiveTargetPdf);

        if (auto group = widget.group("Initial Sampling"))
        {
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
    }
    else
    {
        dirty |= widget.var("Shading Light Samples", mShadingLightSamples);
        dirty |= widget.var("Light Bleeding Reduction", mLBRThreshold, 0.0f, 1.0f);
    }

    mpPixelDebug->renderUI(widget);

    mNeedUpdate = dirty;
}

void AreaLightReSTIR::setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene)
{
    mpScene = pScene;
    Scene::RenderSettings renderSettings;
    renderSettings.useEmissiveLights = false;
    mpScene->setRenderSettings(renderSettings);
    
    mLightParams.setSceneSettings(pScene);
    mLightParams.updateSceneAreaLight(pScene);

    auto pCamera = mpScene->getCamera();
    pCamera->setHasAnimation(true);
    pCamera->setIsAnimated(false);
    mInitialCameraPos = mpScene->getCamera()->getPosition();
    mInitialCameraTarget = mpScene->getCamera()->getTarget();

    std::vector<uint8_t> offsets;
    offsets.resize(8192 * 2); // Q: what is this magic number?
    FillNeighborOffsetBuffer(offsets.data());
    mpNeighborOffsetBuffer = Buffer::createTyped(ResourceFormat::RG8Snorm, 8192, ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess, Buffer::CpuAccess::None, offsets.data());
    mpNeighborOffsetBuffer->setName("ReSTIR: Neighbor Offset Buffer");

    CreatePasses();
    calcLightSpaceMatrix();
}

bool AreaLightReSTIR::onKeyEvent(const KeyboardEvent& keyEvent)
{
    if (keyEvent.type == KeyboardEvent::Type::KeyPressed && keyEvent.key == KeyboardEvent::Key::Key1)
    {
        mNeedUpdate = true;
        mShadowType = ShadowType::ShadowRay;
        return true;
    }

    if (keyEvent.type == KeyboardEvent::Type::KeyPressed && keyEvent.key == KeyboardEvent::Key::Key2)
    {
        mNeedUpdate = true;
        mShadowType = ShadowType::NewPCSSReSTIR;
        return true;
    }

    if (keyEvent.type == KeyboardEvent::Type::KeyPressed && keyEvent.key == KeyboardEvent::Key::Key3)
    {
        mNeedUpdate = true;
        mShadowType = ShadowType::NewPCSS;
        return true;
    }

    if (keyEvent.type == KeyboardEvent::Type::KeyPressed && keyEvent.key == KeyboardEvent::Key::Key4)
    {
        mNeedUpdate = true;
        mShadowType = ShadowType::PCSS;
        return true;
    }

    if (keyEvent.type == KeyboardEvent::Type::KeyPressed && keyEvent.key == KeyboardEvent::Key::Key5)
    {
        mNeedUpdate = true;
        mShadowType = ShadowType::VSM;
        return true;
    }

    if (keyEvent.type == KeyboardEvent::Type::KeyPressed && keyEvent.key == KeyboardEvent::Key::Key6)
    {
        mNeedUpdate = true;
        mShadowType = ShadowType::EVSM;
        return true;
    }

    if (keyEvent.type == KeyboardEvent::Type::KeyPressed && keyEvent.key == KeyboardEvent::Key::Key7)
    {
        mNeedUpdate = true;
        mShadowType = ShadowType::MSM;
        return true;
    }

    return false;
}

AreaLightReSTIR::AreaLightReSTIR()
{
    mpPixelDebug = PixelDebug::create();
    mpSampleGenerator = SampleGenerator::create(SAMPLE_GENERATOR_UNIFORM);

    // Create sampler for sampling different texture
    Sampler::Desc samplerDesc;
    samplerDesc.setAddressingMode(Sampler::AddressMode::Border, Sampler::AddressMode::Border, Sampler::AddressMode::Border).setBorderColor(float4(1.0f)); // Outside sample points will not be shadowed
    samplerDesc.setFilterMode(Sampler::Filter::Linear, Sampler::Filter::Linear, Sampler::Filter::Point); // for comparsion texture
    samplerDesc.setComparisonMode(Sampler::ComparisonMode::Less); // lessEqual = no occluders
    mpSamplerCmp = Sampler::create(samplerDesc);

    //samplerDesc.setMaxAnisotropy(maxAnisotropy);
    //samplerDesc.setAddressingMode(Sampler::AddressMode::Clamp, Sampler::AddressMode::Clamp, Sampler::AddressMode::Clamp);
    samplerDesc.setFilterMode(Sampler::Filter::Linear, Sampler::Filter::Linear, Sampler::Filter::Linear);
    samplerDesc.setComparisonMode(Sampler::ComparisonMode::Disabled);
    mpTrilinearSampler = Sampler::create(samplerDesc);

    //samplerDesc.setAddressingMode(Sampler::AddressMode::Clamp, Sampler::AddressMode::Clamp, Sampler::AddressMode::Clamp);
    samplerDesc.setFilterMode(Sampler::Filter::Point, Sampler::Filter::Point, Sampler::Filter::Point);
    mpPointSampler = Sampler::create(samplerDesc);
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
        // Create Fbo
        setShadowMapPassFbo();

        Program::Desc desc;
        desc.addShaderLibrary("RenderPasses/AreaLightReSTIR/ShadowMap.3d.slang").vsEntry("vsMain").psEntry("psMain");
        desc.setShaderModel("6_5");
        mShadowMapPass.pProgram = GraphicsProgram::create(desc);
        mShadowMapPass.pState = GraphicsState::create();
        mShadowMapPass.pState->setProgram(mShadowMapPass.pProgram);

        Shader::DefineList defines;
        defines.add("_LBR_THRESHOLD", std::to_string(mLBRThreshold));
        mShadowMapPass.pProgram->addDefines(mpScene->getSceneDefines());
        mShadowMapPass.pProgram->addDefines(defines);
        mShadowMapPass.pVars = GraphicsVars::create(mShadowMapPass.pProgram->getReflector());
    }

    // Create SAT passes
    {
        Program::Desc desc;
        Shader::DefineList defines;
        desc.addShaderLibrary("RenderPasses/ShadowMap/SATScans.cs.slang").csEntry("main").setShaderModel("6_5");
        defines.add("_ROW_SCAN");
        mSATScanPasses.mpSATScan1 = ComputePass::create(desc, defines);
        mSATScanPasses.mpSATScan2 = ComputePass::create(desc);
    }

    // Create ReSTIR passes 
    {
        Program::DefineList defines = mpScene->getSceneDefines();
        defines.add(mpSampleGenerator->getDefines());
        defines.add("_MS_DISABLE_ALPHA_TEST");
        defines.add("_DEFAULT_ALPHA_TEST");
        defines.add("_LIGHT_WORLD_SIZE", std::to_string(mLightParams.mLightSize));
        float frustumSize = 2.0f * mLightParams.mLightNearPlane * glm::tan(glm::radians(0.5f * mLightParams.mFovY));
        defines.add("_LIGHT_FRUSTUM_SIZE", std::to_string(frustumSize));

        CreatePass(mpInitialSamplingPass, "RenderPasses/AreaLightReSTIR/InitialSampling.cs.slang", defines);
        CreatePass(mpTemporalResamplingPass, "RenderPasses/AreaLightReSTIR/TemporalResampling.cs.slang", defines);
        CreatePass(mpSpatialResamplingPass, "RenderPasses/AreaLightReSTIR/SpatialResampling.cs.slang", defines);
        CreatePass(mpShadingPass, "RenderPasses/AreaLightReSTIR/Shading.cs.slang", defines);

        //CreatePass(mpSimplePathTracing, "RenderPasses/AreaLightReSTIR/SimplePathTracing.cs.slang", defines);
    }

    UpdateDefines();
}

void AreaLightReSTIR::UpdateDefines()
{
    Shader::DefineList PCSSDefines;
    PCSSDefines.add("_BLOCKER_SEARCH_SAMPLES", std::to_string(mBlockerSearchSamples));
    PCSSDefines.add("_PCF_SAMPLES", std::to_string(mPCFSamples));
    PCSSDefines.add("_TARGET_PDF", std::to_string(mActiveTargetPdf));
    PCSSDefines.add("_DEPTH_BIAS", std::to_string(mLightParams.mDepthBias));
    PCSSDefines.add("_SHADOW_TYPE", std::to_string((uint)mShadowType));

    Shader::DefineList sharedDefines;
    sharedDefines.add("_LIGHT_NEAR_PLANE", std::to_string(mLightParams.mLightNearPlane));
    sharedDefines.add("_LIGHT_FAR_PLANE", std::to_string(mLightParams.mLightFarPlane));
    sharedDefines.add("_LBR_THRESHOLD", std::to_string(mLBRThreshold));
    PCSSDefines.add(sharedDefines);

    switch (mShadowType)
    {
    case ShadowType::NewPCSSReSTIR:
        sharedDefines.add("_NEW_PCSS_ReSTIR");
        break;
    case ShadowType::NewPCSS:
        sharedDefines.add("_NEW_PCSS");
        break;
    case ShadowType::PCSS:
        sharedDefines.add("_PCSS");
        break;
    case ShadowType::VSM:
        sharedDefines.add("_VSM");
        break;
    case ShadowType::EVSM:
        sharedDefines.add("_EVSM");
        break;
    case ShadowType::MSM:
        sharedDefines.add("_MSM");
        break;
    }

    Shader::DefineList allShadowTypeDefines;
    allShadowTypeDefines.add("_NEW_PCSS");
    allShadowTypeDefines.add("_PCSS");
    allShadowTypeDefines.add("_VSM");
    allShadowTypeDefines.add("_EVSM");
    allShadowTypeDefines.add("_MSM");


    // Shadow Map Pass
    {
        mShadowMapPass.pProgram->removeDefines(allShadowTypeDefines);
        mShadowMapPass.pProgram->addDefines(sharedDefines);
    }

    bool hasResampling = mEnableSpatialResampling || mEnableTemporalResampling;
    bool visibilityReuse = mActiveTargetPdf == 0 && hasResampling;
    bool initialBlockerSearch, shadingBlockerSearch;
    if (mShadowType == ShadowType::ShadowRay)
    {
        initialBlockerSearch = false;
        shadingBlockerSearch = false;
        visibilityReuse = mActiveTargetPdf == 0 && hasResampling;
    }
    else
    {
        if (mShadowType == ShadowType::NewPCSSReSTIR)
        {
            initialBlockerSearch = hasResampling || mActiveTargetPdf == 1;
        }
        else
        {
            initialBlockerSearch = false;
            visibilityReuse = false;
        }
        shadingBlockerSearch = !initialBlockerSearch;
    }

    // Initial Sampling Pass
    {
        Shader::DefineList defines;
        defines.add("_INITIAL_AREA_LIGHT_SAMPLES", std::to_string(mInitialAreaLightSamples));
        defines.add("_NEED_BLOCKER_SEARCH", std::to_string(initialBlockerSearch));
        defines.add("_NEED_VISIBILITY_REUSE", std::to_string(visibilityReuse));
        defines.add(PCSSDefines);
        AddDefines(mpInitialSamplingPass, defines);
    }

    // Temporal Resampling Pass
    {
        Shader::DefineList defines;
        defines.add(PCSSDefines);
        AddDefines(mpTemporalResamplingPass, defines);
    }

    // Spatial Resampling Pass
    {
        Shader::DefineList defines;
        defines.add(PCSSDefines);
        AddDefines(mpSpatialResamplingPass, defines);
    }

    // Final Shading Pass
    {
        const char* kStoreFinalVisibility = "_STORE_FINAL_VISIBILITY";
        const char* kBruteForce = "_BRUTE_FORCE";

        Shader::DefineList defines;
        defines.add(kStoreFinalVisibility, mStoreFinalVisibility ? "1" : "0");
        defines.add(kBruteForce, mBruteForce ? "1" : "0");
        defines.add("_NEED_BLOCKER_SEARCH", std::to_string(shadingBlockerSearch));
        defines.add("_LIGHT_SAMPLES", std::to_string(mShadingLightSamples));
        defines.add(PCSSDefines);
        AddDefines(mpShadingPass, defines);
    }
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
        posW = mLightParams.mLightPos;
        dirW = lightData.transMatIT * float4(0.0f, 0.0f, 1.0f, 1.0f);
        lightView = glm::lookAt(posW, posW + dirW, mLightParams.mLightUp); // RH camera look direction is -z direction in view space
        lightProj = glm::perspective(glm::radians(mLightParams.mFovY), 1.0f, mLightParams.mLightNearPlane, mLightParams.mLightFarPlane);
    }
    else if (lightType == LightType::Distant)
    {
        posW = lightData.posW;
        dirW = lightData.dirW;
    }
    else if (lightType == LightType::Sphere)
    {
        posW = mLightParams.mLightPos;
        dirW = lightData.transMatIT * float4(0.0f, -1.0f, 0.0f, 1.0f); // assume sphere light look at -y direction. A proper way should look at all 6 faces
        lightView = glm::lookAt(posW, posW + dirW, mLightParams.mLightUp); // RH camera look direction is -z direction in view space
        lightProj = glm::perspective(glm::radians(mLightParams.mFovY), 1.0f, mLightParams.mLightNearPlane, mLightParams.mLightFarPlane);
    }

    //logInfo("light position: " + to_string(posW));
    //logInfo("lightData position: " + to_string(lightData.posW));
    //logInfo("light direction: " + to_string(dirW));
    //logInfo("GLM_CONFIG_CLIP_CONTROL = " + std::to_string(GLM_CONFIG_CLIP_CONTROL)); // GLM_CLIP_CONTROL_RH_ZO   

    mLightSpaceMat = lightProj * lightView;
    mLightView = lightView;
    mLightProj = lightProj;
}

void AreaLightReSTIR::setShadowMapPassFbo()
{
    ResourceFormat depthFormat = ResourceFormat::D32Float;
    ResourceFormat colorFormat = ResourceFormat::Unknown; // this makes colorTex = nullptr
    switch (mShadowType)
    {
    case ShadowType::VSM:
        colorFormat = ResourceFormat::RG32Float;
        break;
    case ShadowType::EVSM:
    case ShadowType::MSM:
        colorFormat = ResourceFormat::RGBA32Float;
        break;
    }

    Fbo::Desc fboDesc;
    fboDesc.setDepthStencilTarget(depthFormat);
    fboDesc.setColorTarget(0, colorFormat);

    uint mipLevel = colorFormat == ResourceFormat::Unknown ? 1 : Texture::kMaxPossible; // Texture::kMaxPossible

    mShadowMapPass.pFbo = Fbo::create2D(kShadowMapSize, kShadowMapSize, fboDesc, 1, mipLevel);
}
