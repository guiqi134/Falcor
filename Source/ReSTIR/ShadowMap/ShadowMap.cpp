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
#include "ShadowMap.h"
#include <RenderGraph/RenderPassHelpers.h>
#include <glm/gtx/string_cast.hpp>

namespace
{
    const char kDesc[] = "shadow map pass";

    // Render graph output names
    const char kDepthName[] = "depth";
    const char kColorName[] = "color";
    const char kLinearDepthName[] = "linearDepth";

    const char kPCSSPassFile[] = "RenderPasses/ShadowMap/PCSSPass.slang";
    const char kShadingPassFile[] = "RenderPasses/ShadowMap/ShadingPass.slang";
}

// Don't remove this. it's required for hot-reload to function properly
extern "C" __declspec(dllexport) const char* getProjDir()
{
    return PROJECT_DIR;
}

extern "C" __declspec(dllexport) void getPasses(Falcor::RenderPassLibrary& lib)
{
    lib.registerClass("ShadowMap", kDesc, ShadowMap::create);
}

ShadowMap::SharedPtr ShadowMap::create(RenderContext* pRenderContext, const Dictionary& dict)
{
    SharedPtr pPass = SharedPtr(new ShadowMap);
    return pPass;
}

std::string ShadowMap::getDesc() { return kDesc; }

Dictionary ShadowMap::getScriptingDictionary()
{
    return Dictionary();
}

RenderPassReflection ShadowMap::reflect(const CompileData& compileData)
{
    // Define the required resources here
    RenderPassReflection reflector;
    reflector.addOutput(kDepthName, "Depth value").format(ResourceFormat::D32Float).bindFlags(Resource::BindFlags::DepthStencil | Resource::BindFlags::ShaderResource).texture2D(1024, 1024, 0);
    reflector.addOutput(kColorName, "Output color");
    reflector.addOutput(kLinearDepthName, "Linear depth value");

    return reflector;
}

void ShadowMap::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    if (mpScene == nullptr)
    {
        return;
    }

    if (mNeedUpdate)
    {
        setSceneAreaLight(mpScene, SceneName::TestScene, mLightPos, mLightSize, mRotation);
        updateDefines();
        calcLightSpaceMatrix();

        // Change mesh by adjusting the corresponding transform in global matrices
        auto pAnimationController = mpScene->getAnimationController();
        auto globalMatrices = const_cast<std::vector<glm::mat4>&>(pAnimationController->getLocalMatrices()); // remove constness
    }

    mpScene->update(pRenderContext, gpFramework->getGlobalClock().getTime());

    const auto& pDepthTex = renderData[kDepthName]->asTexture();
    const auto& pColorTex = renderData[kColorName]->asTexture();
    const auto& pLinearDepthTex = renderData[kLinearDepthName]->asTexture();

    // Ensure pFbo->width()/height() have already set the value
    uint32_t screenWidth = gpFramework->getTargetFbo()->getWidth();
    uint32_t screenHeight = gpFramework->getTargetFbo()->getHeight();

    mpPixelDebug->beginFrame(pRenderContext, uint2(screenWidth, screenHeight));

    // Shadow map pass
    {
        auto& pFbo = mPCSSPass.pFbo;
        pFbo->attachDepthStencilTarget(pDepthTex);
        pFbo->attachColorTarget(pLinearDepthTex, 0);
        pRenderContext->clearFbo(pFbo.get(), float4(0.0f), 1.0f, 0); // clear all components (RTV, DSV)
        mPCSSPass.pState->setFbo(pFbo);

        auto vars = mPCSSPass.pVars->getRootVar();
        vars["PerFrameCB"]["gLightSpaceMat"] = mLightSpaceMat;
        vars["PerFrameCB"]["nearZ"] = mpScene->getCamera()->getNearPlane();
        vars["PerFrameCB"]["farZ"] = mpScene->getCamera()->getFarPlane();

        mpPixelDebug->prepareProgram(mPCSSPass.pProgram, mPCSSPass.pVars->getRootVar());
        mpScene->rasterize(pRenderContext, mPCSSPass.pState.get(), mPCSSPass.pVars.get());

    }

    // TODO: Store the depth map to dictionary
    mpShadowMap = mPCSSPass.pFbo->getDepthStencilTexture();

    // Use rasterizer to shade the scene
    {
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
        vars["PerFrameCB"]["gLightPos"] = mLightPos;
        vars["PerFrameCB"]["gShadowMapDim"] = float2(1024);
        vars["PerFrameCB"]["gEnableShadow"] = mEnableShadow;
        vars["PerFrameCB"]["nearZ"] = mLightNearPlane;
        vars["PerFrameCB"]["farZ"] = 10000.0f;
        vars["gShadowMap"] = mpShadowMap;

        mpPixelDebug->prepareProgram(mShadingPass.pProgram, mShadingPass.pVars->getRootVar());
        mpScene->rasterize(pRenderContext, mShadingPass.pState.get(), mShadingPass.pVars.get());
    }

    mpPixelDebug->endFrame(pRenderContext);
}

void ShadowMap::renderUI(Gui::Widgets& widget)
{
    bool dirty = false;

    dirty |= widget.checkbox("Enable shadow", mEnableShadow);
    dirty |= widget.var("Blocker search samples", mBlockerSearchSamples);
    dirty |= widget.var("PCF samples", mPCFSamples);

    // Light configuration
    if (auto group = widget.group("Area Light"))
    {
        dirty |= group.var("Light near plane", mLightNearPlane);
        dirty |= group.var("Light FovY", mFovY);
        dirty |= group.var("Light Size", mLightSize);
        dirty |= group.var("Light center position", mLightPos);
        dirty |= group.var("Light center up", mLightUp);
        dirty |= group.var("Light rotation", mRotation);
    }

    mNeedUpdate = dirty;

    mpPixelDebug->renderUI(widget);
}

void ShadowMap::setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene)
{
    mpScene = pScene;

    // Adjust area light in the scene
    setSceneAreaLight(mpScene, SceneName::TestScene, mLightPos, mLightSize, mRotation);

    if (mpScene)
    {
        mPCSSPass.pProgram->addDefines(pScene->getSceneDefines());
        mShadingPass.pProgram->addDefines(pScene->getSceneDefines());
    }

    // Update shading pass defines
    updateDefines();

    // Create program vars
    mPCSSPass.pVars = GraphicsVars::create(mPCSSPass.pProgram->getReflector());
    mShadingPass.pVars = GraphicsVars::create(mShadingPass.pProgram->getReflector());

    // Create sampler for sampling shadow map texture
    Sampler::Desc samplerDesc;
    samplerDesc.setAddressingMode(Sampler::AddressMode::Border, Sampler::AddressMode::Border, Sampler::AddressMode::Border).setBorderColor(float4(1.0f)); // Outside sample points will not be shadowed
    samplerDesc.setFilterMode(Sampler::Filter::Point, Sampler::Filter::Point, Sampler::Filter::Point); // for comparsion texture
    samplerDesc.setComparisonMode(Sampler::ComparisonMode::LessEqual); // lessEqual = no occluders
    mShadingPass.pVars["gSamplerCmp"] = Sampler::create(samplerDesc);

    samplerDesc.setFilterMode(Sampler::Filter::Linear, Sampler::Filter::Linear, Sampler::Filter::Linear);
    samplerDesc.setComparisonMode(Sampler::ComparisonMode::Disabled);
    mShadingPass.pVars["gSampler"] = Sampler::create(samplerDesc);

    // Calculate light space transform matrix
    calcLightSpaceMatrix();
}

ShadowMap::ShadowMap()
{
    // PCSS pass
    {
        mPCSSPass.pFbo = Fbo::create();

        // Create raster program
        Program::Desc desc;
        desc.addShaderLibrary(kPCSSPassFile).vsEntry("vsMain").psEntry("psMain");
        desc.setShaderModel("6_5");
        mPCSSPass.pProgram = GraphicsProgram::create(desc);

        // Initialize graphics state
        mPCSSPass.pState = GraphicsState::create();
        mPCSSPass.pState->setProgram(mPCSSPass.pProgram);
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

void ShadowMap::calcLightSpaceMatrix()
{
    auto pRectLight = mpScene->getLightByName("Rect Light");
    auto lightData = pRectLight->getData();

    // light pos and dir in world space
    float3 posW = mLightPos;
    float3 dirW = lightData.transMatIT * float4(0.0f, 0.0f, 1.0f, 1.0f);

    logInfo("light position: " + to_string(posW));
    logInfo("light direction: " + to_string(dirW));
    logInfo("GLM_CONFIG_CLIP_CONTROL = " + std::to_string(GLM_CONFIG_CLIP_CONTROL)); // GLM_CLIP_CONTROL_RH_ZO

    // Build light space transform matrix
    float4x4 lightView = glm::lookAt(posW, posW + dirW, mLightUp);
    float4x4 lightProj = mpScene->getCamera()->getProjMatrix(); // use camera projection matrix
    lightView = glm::lookAt(posW, posW + dirW, glm::vec3(0.0f, 1.0f, 0.0f)); // RH camera look direction is -z direction in view space
    lightProj = glm::perspective(glm::radians(mFovY), 1.0f, mLightNearPlane, 10000.0f);

    mLightSpaceMat = lightProj * lightView;
    mLightView = lightView;
}

void ShadowMap::updateDefines()
{
    Shader::DefineList shadingPassDefines;
    shadingPassDefines.add("BLOCKER_SEARCH_SAMPLES", std::to_string(mBlockerSearchSamples));
    shadingPassDefines.add("PCF_SAMPLES", std::to_string(mPCFSamples));
    shadingPassDefines.add("NEAR_PLANE", std::to_string(mLightNearPlane));
    shadingPassDefines.add("LIGHT_WORLD_SIZE", std::to_string(mLightSize));
    float frustumSize = 2.0f * mLightNearPlane * glm::tan(glm::radians(0.5f * mFovY));
    shadingPassDefines.add("LIGHT_FRUSTUM_SIZE", std::to_string(frustumSize));
    mShadingPass.pProgram->addDefines(shadingPassDefines);
}
