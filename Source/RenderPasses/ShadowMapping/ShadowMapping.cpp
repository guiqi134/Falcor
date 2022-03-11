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


#include "ShadowMapping.h"

#include <RenderGraph/RenderPassHelpers.h> // 


namespace
{
    const char kDesc[] = "shadow mapping pass";

    // Render graph output names
    const char kDepthName[] = "depth";
    const char kColorName[] = "color";
    const char kLinearDepthName[] = "linearDepth";

    const char kPCSSPassFile[] = "RenderPasses/ShadowMap/ShadowMap.slang";
    const char kShadingPassFile[] = "RenderPasses/ShadowMap/Shading.slang";

    const uint kShadowMapSize = 1 << 12;
}

// Don't remove this. it's required for hot-reload to function properly
extern "C" __declspec(dllexport) const char* getProjDir()
{
    return PROJECT_DIR;
}

extern "C" __declspec(dllexport) void getPasses(Falcor::RenderPassLibrary& lib)
{
    lib.registerClass("ShadowMapping", kDesc, ShadowMapping::create);
}

ShadowMapping::SharedPtr ShadowMapping::create(RenderContext* pRenderContext, const Dictionary& dict)
{
    SharedPtr pPass = SharedPtr(new ShadowMapping);
    return pPass;
}

std::string ShadowMapping::getDesc() { return kDesc; }

Dictionary ShadowMapping::getScriptingDictionary()
{
    return Dictionary();
}

RenderPassReflection ShadowMapping::reflect(const CompileData& compileData)
{
    // Define the required resources here
    RenderPassReflection reflector;
    reflector.addOutput(kDepthName, "Depth value").format(ResourceFormat::D32Float).bindFlags(Resource::BindFlags::DepthStencil | Resource::BindFlags::ShaderResource).texture2D(
        kShadowMapSize, kShadowMapSize, 0);
    //reflector.addOutput(kVSM, "VSM").format(ResourceFormat::RG32Float).texture2D(kShadowMapSize, kShadowMapSize, 0);

    reflector.addOutput(kColorName, "Output color");

    return reflector;
}

void ShadowMapping::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    if (mpScene == nullptr)
    {
        return;
    }

    if (mNeedUpdate)
    {
        mLightParams.updateSceneAreaLight(mpScene);
        updateDefines();
        calcLightSpaceMatrix();
    }

    mpScene->update(pRenderContext, gpFramework->getGlobalClock().getTime());

    const auto& pDepthTex = renderData[kDepthName]->asTexture();

    // Ensure pFbo->width()/height() have already set the value
    uint32_t screenWidth = gpFramework->getTargetFbo()->getWidth();
    uint32_t screenHeight = gpFramework->getTargetFbo()->getHeight();

    mpPixelDebug->beginFrame(pRenderContext, uint2(screenWidth, screenHeight));

    // Shadow map pass
    // TODO: add MSAA for VSM 
    {
        PROFILE("Shadow Map");
        auto& pFbo = mShadowMapPass.pFbo;
        pFbo->attachDepthStencilTarget(pDepthTex);
        //pFbo->attachColorTarget(mpVSM, 0);
        pRenderContext->clearFbo(pFbo.get(), float4(0.0f), 1.0f, 0); // clear all components (RTV, DSV)
        mShadowMapPass.pState->setFbo(pFbo);

        auto vars = mShadowMapPass.pVars->getRootVar();
        vars["PerFrameCB"]["gLightSpaceMat"] = mLightSpaceMat;
        vars["PerFrameCB"]["nearZ"] = mLightParams.mLightNearPlane;
        vars["PerFrameCB"]["farZ"] = mLightParams.mLightFarPlane;

        mpPixelDebug->prepareProgram(mShadowMapPass.pProgram, mShadowMapPass.pVars->getRootVar());
        mpScene->rasterize(pRenderContext, mShadowMapPass.pState.get(), mShadowMapPass.pVars.get(), RasterizerState::CullMode::None);
    }


    // Use rasterizer to shade the scene
    const auto& pColorTex = renderData[kColorName]->asTexture();
    {
        PROFILE("PCSS Shading");
        auto& pFbo = mShadingPass.pFbo;
        if (pFbo->getDepthStencilTexture() == nullptr)
        {
            auto pShadingDepthTex = Texture::create2D(screenWidth, screenHeight, ResourceFormat::D32Float, 1, 1, nullptr, Resource::BindFlags::DepthStencil);
            pFbo->attachDepthStencilTarget(pShadingDepthTex);
        }
        pFbo->attachColorTarget(pColorTex, 0);
        pRenderContext->clearFbo(pFbo.get(), float4(0.0f), 1.0f, 0); // clear all components (RTV, DSV)
        mShadingPass.pState->setFbo(pFbo);

        auto vars = mShadingPass.pVars;
        vars["PerFrameCB"]["gLightSpaceMat"] = mLightSpaceMat;
        vars["PerFrameCB"]["gLightView"] = mLightView;
        vars["PerFrameCB"]["gLightProj"] = mLightProj;
        vars["PerFrameCB"]["gLightPos"] = mLightParams.mLightPos;
        vars["PerFrameCB"]["gShadowMapDim"] = uint2(kShadowMapSize);
        vars["PerFrameCB"]["gEnableShadow"] = mEnableShadow;
        vars["PerFrameCB"]["nearZ"] = mLightParams.mLightNearPlane;
        vars["PerFrameCB"]["farZ"] = 10000.0f;
        vars["PerFrameCB"]["gFrameIndex"] = (uint)gpFramework->getGlobalClock().getFrame();
        vars["gShadowMap"] = mShadowMapPass.pFbo->getDepthStencilTexture();

        mpPixelDebug->prepareProgram(mShadingPass.pProgram, mShadingPass.pVars->getRootVar());
        mpScene->rasterize(pRenderContext, mShadingPass.pState.get(), mShadingPass.pVars.get(), RasterizerState::CullMode::None);
    }

    mpPixelDebug->endFrame(pRenderContext);
}

void ShadowMapping::renderUI(Gui::Widgets& widget)
{
}

void ShadowMapping::setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene)
{
    mpScene = pScene;
    mpSampleGenerator = SampleGenerator::create(SAMPLE_GENERATOR_UNIFORM);

    // Adjust area light in the scene
    mLightParams.updateSceneAreaLight(pScene);

    if (mpScene)
    {
        mShadowMapPass.pProgram->addDefines(pScene->getSceneDefines());
        mShadingPass.pProgram->addDefines(pScene->getSceneDefines());
    }

    // Update shading pass defines
    updateDefines();

    // Create program vars
    mShadowMapPass.pVars = GraphicsVars::create(mShadowMapPass.pProgram->getReflector());
    mShadingPass.pVars = GraphicsVars::create(mShadingPass.pProgram->getReflector());

    // Create sampler for sampling shadow map texture
    Sampler::Desc samplerDesc;
    samplerDesc.setAddressingMode(Sampler::AddressMode::Border, Sampler::AddressMode::Border, Sampler::AddressMode::Border).setBorderColor(float4(1.0f)); // Outside sample points will not be shadowed
    samplerDesc.setFilterMode(Sampler::Filter::Linear, Sampler::Filter::Linear, Sampler::Filter::Point); // for comparsion texture
    samplerDesc.setComparisonMode(Sampler::ComparisonMode::Less); // lessEqual = no occluders
    mShadingPass.pVars["gSamplerCmp"] = Sampler::create(samplerDesc);

    samplerDesc.setFilterMode(Sampler::Filter::Linear, Sampler::Filter::Linear, Sampler::Filter::Linear);
    samplerDesc.setComparisonMode(Sampler::ComparisonMode::Disabled);
    mShadingPass.pVars["gSampler"] = Sampler::create(samplerDesc);

    // Calculate light space transform matrix
    calcLightSpaceMatrix();
}

bool ShadowMapping::onKeyEvent(const KeyboardEvent& keyEvent)
{
    if (keyEvent.type == KeyboardEvent::Type::KeyPressed && keyEvent.key == KeyboardEvent::Key::Key1)
    {
        mNeedUpdate = true;
        mShadowType = ShadowType::PCSS;
        return true;
    }

    if (keyEvent.type == KeyboardEvent::Type::KeyPressed && keyEvent.key == KeyboardEvent::Key::Key2)
    {
        mNeedUpdate = true;
        mShadowType = ShadowType::VSM;
        return true;
    }

    return false;
}

ShadowMapping::ShadowMapping()
{
    // Shadow map pass
    {
        ResourceFormat depthFormat = ResourceFormat::D32Float;
        ResourceFormat colorFormat = ResourceFormat::Unknown;
        switch (mShadowType)
        {
        case ShadowType::VSM:
            colorFormat = ResourceFormat::RG32Float;
            break;
        case ShadowType::EVSM:
            break;
        case ShadowType::MSM:
            break;
        }

        Fbo::Desc fboDesc;
        fboDesc.setDepthStencilTarget(depthFormat);
        fboDesc.setColorTarget(0, colorFormat);

        uint mipLevel = colorFormat == ResourceFormat::Unknown ? 1 : Texture::kMaxPossible;

        // Create Fbo
        mShadowMapPass.pFbo = Fbo::create2D(kShadowMapSize, kShadowMapSize, fboDesc, 1, mipLevel);

        // Create raster program
        Program::Desc desc;
        desc.addShaderLibrary(kPCSSPassFile).vsEntry("vsMain").psEntry("psMain");
        desc.setShaderModel("6_5");
        mShadowMapPass.pProgram = GraphicsProgram::create(desc);

        // Initialize graphics state
        mShadowMapPass.pState = GraphicsState::create();
        mShadowMapPass.pState->setProgram(mShadowMapPass.pProgram);
    }

    // Shading pass
    {
        mShadingPass.pFbo = Fbo::create();

        // Create raster program
        Program::Desc desc;
        desc.addShaderLibrary(kShadingPassFile).psEntry("psMain");
        desc.setShaderModel("6_5");
        mShadingPass.pProgram = GraphicsProgram::create(desc);

        // Initialize graphics state
        mShadingPass.pState = GraphicsState::create();
        mShadingPass.pState->setProgram(mShadingPass.pProgram);
    }

    mpPixelDebug = PixelDebug::create();
}

void ShadowMapping::calcLightSpaceMatrix()
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
        logInfo("Sphere Light");
        posW = mLightParams.mLightPos;
        dirW = lightData.transMatIT * float4(0.0f, -1.0f, 0.0f, 1.0f); // assume sphere light look at -y direction. A proper way should look at all 6 faces
        lightView = glm::lookAt(posW, posW + dirW, mLightParams.mLightUp); // RH camera look direction is -z direction in view space
        lightProj = glm::perspective(glm::radians(mLightParams.mFovY), 1.0f, mLightParams.mLightNearPlane, mLightParams.mLightFarPlane);
    }

    logInfo("light position: " + to_string(posW));
    logInfo("lightData position: " + to_string(lightData.posW));
    logInfo("light direction: " + to_string(dirW));
    logInfo("GLM_CONFIG_CLIP_CONTROL = " + std::to_string(GLM_CONFIG_CLIP_CONTROL)); // GLM_CLIP_CONTROL_RH_ZO   

    mLightSpaceMat = lightProj * lightView;
    mLightView = lightView;
    mLightProj = lightProj;
}

void ShadowMapping::updateDefines()
{
    Shader::DefineList sharedDefines;
    switch (mShadowType)
    {
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

    {
        mShadowMapPass.pProgram->addDefines(sharedDefines);
    }

    {
        Shader::DefineList shadingPassDefines;
        shadingPassDefines.add(mpSampleGenerator->getDefines());
        shadingPassDefines.add("_LIGHT_SAMPLES", std::to_string(mLightSamples));
        shadingPassDefines.add("_DEPTH_BIAS", std::to_string(mDepthBias));
        shadingPassDefines.add("BLOCKER_SEARCH_SAMPLES", std::to_string(mBlockerSearchSamples));
        shadingPassDefines.add("PCF_SAMPLES", std::to_string(mPCFSamples));
        shadingPassDefines.add("NEAR_PLANE", std::to_string(mLightParams.mLightNearPlane));
        shadingPassDefines.add("LIGHT_WORLD_SIZE", std::to_string(mLightParams.mLightSize));
        float frustumSize = 2.0f * mLightParams.mLightNearPlane * glm::tan(glm::radians(0.5f * mLightParams.mFovY));
        shadingPassDefines.add("LIGHT_FRUSTUM_SIZE", std::to_string(frustumSize));
        shadingPassDefines.add(sharedDefines);

        mShadingPass.pProgram->addDefines(shadingPassDefines);
    }
}
