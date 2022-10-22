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
#include "ISM.h"
#include "HostDeviceSharedDefinitions.h"

namespace
{
    const std::string kShaderDirectory = "RenderPasses/ISM/";
    const ResourceBindFlags kBufferBindFlags = ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess;
    const ResourceBindFlags kDepthTexFlags = ResourceBindFlags::ShaderResource | ResourceBindFlags::DepthStencil;
    const ResourceBindFlags kAutoMipBindFlags = ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess | ResourceBindFlags::RenderTarget;
};

const RenderPass::Info ISM::kInfo { "ISM", "Insert pass description here." };

// Don't remove this. it's required for hot-reload to function properly
extern "C" FALCOR_API_EXPORT const char* getProjDir()
{
    return PROJECT_DIR;
}

extern "C" FALCOR_API_EXPORT void getPasses(Falcor::RenderPassLibrary& lib)
{
    lib.registerPass(ISM::kInfo, ISM::create);
}

ISM::ISM(const Dictionary& dict)
    : RenderPass(kInfo)
{
    for (const auto& [key, value] : dict)
    {
        //if (key == kOptions) mOptions = value;
        //else logWarning("Unknown field '{}' in RTXDIPass dictionary.", key);
    }

    mpSampleGenerator = SampleGenerator::create(SAMPLE_GENERATOR_TINY_UNIFORM);
    mpPixelDebug = PixelDebug::create();
    mpFence = GpuFence::create();

    // Create samplers
    Sampler::Desc samplerDesc;
    samplerDesc.setAddressingMode(Sampler::AddressMode::Clamp, Sampler::AddressMode::Clamp, Sampler::AddressMode::Clamp);
    samplerDesc.setFilterMode(Sampler::Filter::Point, Sampler::Filter::Point, Sampler::Filter::Point);
    mpPointSampler = Sampler::create(samplerDesc);

    samplerDesc.setFilterMode(Sampler::Filter::Linear, Sampler::Filter::Linear, Sampler::Filter::Linear);
    mpLinearSampler = Sampler::create(samplerDesc);
}

ISM::SharedPtr ISM::create(RenderContext* pRenderContext, const Dictionary& dict)
{
    SharedPtr pPass = SharedPtr(new ISM(dict));
    return pPass;
}

Dictionary ISM::getScriptingDictionary()
{
    return Dictionary();
}

RenderPassReflection ISM::reflect(const CompileData& compileData)
{
    // Define the required resources here
    RenderPassReflection reflector;
    reflector.addInput("vbuffer", "");
    reflector.addInput("mvec", "");

    reflector.addOutput("color", "").format(ResourceFormat::RGBA32Float);;
    // debug the ISM texture
    reflector.addOutput("ism after push", "").texture2D(mIsmSize, mIsmSize).format(ResourceFormat::RGBA32Float);;
    reflector.addOutput("ism after pull", "").texture2D(mIsmSize, mIsmSize).format(ResourceFormat::RGBA32Float);;

    reflector.addOutput("entireIsmTex", "").texture2D(mIsmTextureSize, mIsmTextureSize).format(ResourceFormat::RGBA32Float);;
    return reflector;
}

void ISM::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    if (!mpScene) return;

    mpPixelDebug->beginFrame(pRenderContext, mScreenSize);

    // Update lights transform matrix
    Scene::UpdateFlags updates = mpScene->getUpdates();
    if (bool(updates & Scene::UpdateFlags::LightsMoved)) prepareLightShadowMapData();

    // Update resources
    if (mUpdateResources)
    {
        mpIsmTextureArray = Texture::create2D(mIsmSize, mIsmSize, ResourceFormat::R32Float, mIsmPerLight * mTotalLightsCount, mIsmMipLevels, nullptr, kAutoMipBindFlags);

        mUpdateResources = false;
    }

    if (mUpdateDefines)
    {
        ismPass.pProgram->addDefine("GS_OUT_STREAM", std::to_string(mIsmSceneType));
    }

    //auto clearColor = float4(float3(mpScene->getCamera()->getFarPlane()), 1.0f);
    ismPass.pFbo->attachColorTarget(mpIsmTextureArray, 0, 0, 0, mIsmPerLight * mTotalLightsCount);
    pRenderContext->clearTexture(mpIsmTextureArray.get(), float4(1.0f));

    if (mDrawWireframe && mTotalLightsCount == 1)
    {
        mWireframePass.pState->setFbo(ismPass.pFbo);
        mWireframePass.pState->setDepthStencilState(mpNoDepthDS);
        auto wireframeVars = mWireframePass.pVars->getRootVar();
        wireframeVars["PerFrameCB"]["gColor"] = float4(0, 1, 0, 1);
        wireframeVars["PerFrameCB"]["gLightID"] = uint(0);
        wireframeVars["PerFrameCB"]["gNearFarPlane"] = mLightNearFarPlane;
        wireframeVars["gLightShadowDataBuffer"] = mpLightShadowDataBuffer;
        mpScene->rasterize(pRenderContext, mWireframePass.pState.get(), mWireframePass.pVars.get(), mpWireframeRS, mpWireframeRS);
    }

    pRenderContext->clearUAV(mpCounterBuffer->getUAV().get(), uint4(0));

    // This rasterization stage simplifies the scene into point representation (vs + hs + ds + gs)
    {
        GraphicsState::Viewport viewport(0.0f, 0.0f, (float)mIsmSize, (float)mIsmSize, 0.0f, 1.0f);
        ismPass.pState->setViewport(0, viewport);

        ismPass.pState->setFbo(ismPass.pFbo);
        auto pStagingDepthTexture = Texture::create2D(mIsmSize, mIsmSize, ResourceFormat::D32Float, mIsmPerLight * mTotalLightsCount, 1, nullptr, kDepthTexFlags);
        pRenderContext->clearTexture(pStagingDepthTexture.get(), float4(1.0f));
        ismPass.pFbo->attachDepthStencilTarget(pStagingDepthTexture, 0, 0, mIsmPerLight * mTotalLightsCount); // Depth test needs this
        ismPass.pState->setDepthStencilState(mpDepthTestDS);


        auto ismVars = ismPass.pVars->getRootVar();
        ismVars["ismCB"]["gTotalLightCount"] = mTotalLightsCount;
        ismVars["ismCB"]["gScreenSize"] = mScreenSize;
        ismVars["ismCB"]["gConstColor"] = float4(1, 1, 1, 1);
        ismVars["ismCB"]["gLightNearFar"] = mLightNearFarPlane;
        ismVars["ismCB"]["gCameraNearFar"] = float2(mpScene->getCamera()->getNearPlane(), mpScene->getCamera()->getFarPlane());
        ismVars["ismCB"]["gCameraView"] = mpScene->getCamera()->getViewMatrix();
        ismVars["gPointsBuffer"] = mpPointsBuffer;
        ismVars["gLightShadowDataBuffer"] = mpLightShadowDataBuffer;
        ismVars["gCounterBuffer"] = mpCounterBuffer;

        mpPixelDebug->prepareProgram(ismPass.pProgram, ismVars);
        mpScene->rasterize(pRenderContext, ismPass.pState.get(), ismPass.pVars.get(), RasterizerState::CullMode::None);
    }

    //auto pStagingIsmTextureArray = mpIsmTextureArray;
    //pStagingIsmTextureArray->generateMips(pRenderContext);

    // Get the max extend of the scene
    float3 sceneExtend = mpScene->getSceneBounds().extent();
    float maxExtend = std::max(std::max(sceneExtend.x, sceneExtend.y), sceneExtend.z);
    float depthThreshold = 0.025f * (maxExtend - mLightNearFarPlane.x) / (mLightNearFarPlane.y - mLightNearFarPlane.x);
    if (mFrameIndex == 50) logInfo(std::format("Scene extend = ({}, {}, {}), depth threshold = {}", sceneExtend.x, sceneExtend.y, sceneExtend.z, depthThreshold));

    // Do pull pass on ISMs
    //if (false)
    {
        FALCOR_PROFILE("ISM Pull");
        for (uint m = 0; m < mpIsmTextureArray->getMipCount() - 1; m++)
        {
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

                if (mFrameIndex == 50)
                {
                    logInfo(std::format("ISM texture mip levels = {}", mpIsmTextureArray->getMipCount()));
                    logInfo(std::format("ISM src and dst texture mip levels = {}, {}", srcMipLevel, dstMipLevel));
                    logInfo(std::format("source texture dim = ({}, {})", pSrcTexture->getWidth(srcMipLevel), pSrcTexture->getHeight(srcMipLevel)));
                    logInfo(std::format("destination texture dim = ({}, {})", pDstTexture->getWidth(dstMipLevel), pDstTexture->getHeight(dstMipLevel)));
                }

                auto pullVars = mpIsmPullPass->getRootVar();
                pullVars["PullCB"]["gDepthDiffThreshold"] = depthThreshold /* * (float)pow(2, m)*/; 
                mpIsmPullPass->getVars()->setSrv(srcTexBindLoc, pSrc);
                mpIsmPullPass->getVars()->setUav(dstTexBindLoc, pDst);
                mpPixelDebug->prepareProgram(mpIsmPullPass->getProgram(), pullVars);
                mpIsmPullPass->execute(pRenderContext, mpIsmTextureArray->getWidth(m + 1), mpIsmTextureArray->getHeight(m + 1));
            }
        }
    }

    {
        visualizeIsmTexture(mpIsmTextureArray, renderData["ism after pull"]->asTexture(), pRenderContext, mVisualizeMipLevel, mVisLightID);
    }

    // Do push pass on ISMs
    //if (false)
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
                mpIsmPushPass->getVars()->setSrv(srcTexBindLoc, pSrc);
                mpIsmPushPass->getVars()->setUav(dstTexBindLoc, pDst);
                mpPixelDebug->prepareProgram(mpIsmPushPass->getProgram(), pushVars);
                mpIsmPushPass->execute(pRenderContext, mpIsmTextureArray->getWidth(m - 1), mpIsmTextureArray->getHeight(m - 1));
            }
        }
    }

    // Visualize ISM pass
    {
        visualizeIsmTexture(mpIsmTextureArray, renderData["ism after push"]->asTexture(), pRenderContext, mVisualizeMipLevel, mVisLightID);
    }

    // Final shading
    {
        auto shadingVars = mpShadingPass->getRootVar();
        shadingVars["ShadingCB"]["gFrameIndex"] = mFrameIndex;
        shadingVars["ShadingCB"]["gScreenSize"] = mScreenSize;
        shadingVars["ShadingCB"]["gLightNearFar"] = mLightNearFarPlane;
        shadingVars["ShadingCB"]["gLinearSampler"] = mpLinearSampler;
        shadingVars["gVbuffer"] = renderData["vbuffer"]->asTexture();
        shadingVars["gIsmTextureArray"] = mpIsmTextureArray;
        shadingVars["gLightShadowDataBuffer"] = mpLightShadowDataBuffer;
        shadingVars["gOutColor"] = renderData["color"]->asTexture();
        mpScene->setRaytracingShaderData(pRenderContext, shadingVars);
        mpPixelDebug->prepareProgram(mpShadingPass->getProgram(), shadingVars);
        mpShadingPass->execute(pRenderContext, mScreenSize.x, mScreenSize.y);
    }



    // Print to debug
    if (false && mFrameIndex == 50)
    {
        auto counterBufferData = getDeviceResourceData<uint>(pRenderContext, mpCounterBuffer);
        for (uint i = 0; i < counterBufferData.size(); i++)
        {
            logInfo(std::format("counterBufferData[{}] = {}", i, counterBufferData[i]));
        }

        auto pointsBufferData = getDeviceResourceData<float4>(pRenderContext, mpPointsBuffer);
        for (uint i = 0; i < std::min(100u, (uint)pointsBufferData.size()); i++)
        {
            logInfo(std::format("pointsBufferData[{}] = ({}, {}, {}, {})", i, pointsBufferData[i].x,
                pointsBufferData[i].y, pointsBufferData[i].z, pointsBufferData[i].w));
        }
    }

    mFrameIndex++;

    mpPixelDebug->endFrame(pRenderContext);
}

void ISM::renderUI(Gui::Widgets& widget)
{
    widget.checkbox("Wireframe", mDrawWireframe);
    mUpdateResources = widget.var("ISM Pull&Push Mip Levels", mIsmMipLevels);
    widget.var("ISM Push Mode", mIsmPushMode);
    mUpdateDefines = widget.var("ISM Scene Type", mIsmSceneType);
    widget.var("Mip level to visualize", mVisualizeMipLevel);
    widget.var("Light's ISM to visualize", mVisLightID);

    mpPixelDebug->renderUI(widget);
}

void ISM::setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene)
{
    mpScene = pScene;

    auto pLightCollection = mpScene->getLightCollection(pRenderContext);
    //mTotalLightsCount = (uint)pLightCollection->getMeshLights().size();
    mScreenSize = uint2(gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());

    // Get and count all the point lights
    auto lights = mpScene->getLights();
    for (uint i = 0; i < mpScene->getLightCount(); i++)
    {
        auto pointLight = dynamic_cast<PointLight*>(lights[i].get());
        if (pointLight != nullptr)
        {
            mPointLights.push_back(pointLight);
            mTotalLightsCount++;
        }
    }

    logInfo(std::format("Total point light count = {}", mTotalLightsCount));


    // Load wireframe pass shader
    {
        Program::Desc desc;
        desc.addShaderLibrary(kShaderDirectory + "Wireframe.3d.slang").vsEntry("vsMain").psEntry("psMain");
        mWireframePass.pProgram = GraphicsProgram::create(desc);
        mWireframePass.pProgram->addDefines(mpScene->getSceneDefines());
        mWireframePass.pProgram->setTypeConformances(mpScene->getTypeConformances());
        mWireframePass.pState = GraphicsState::create();
        mWireframePass.pState->setProgram(mWireframePass.pProgram);
        mWireframePass.pVars = GraphicsVars::create(mWireframePass.pProgram->getReflector());
    }

    // Load ISM shaders
    {
        Program::Desc desc;
        //desc.addShaderLibrary(kShaderDirectory + "ISM.3d.slang").vsEntry("vsMain").hsEntry("hsMain").dsEntry("dsMain").psEntry("psMain");
        desc.addShaderLibrary(kShaderDirectory + "ISM.3d.slang").vsEntry("vsMain").gsEntry("gsMain").psEntry("psMain");
        //desc.addShaderLibrary(kShaderDirectory + "ISM.3d.slang").vsEntry("vsMain").psEntry("psMain");
        ismPass.pProgram = GraphicsProgram::create(desc);
        ismPass.pProgram->addDefines(mpScene->getSceneDefines());
        ismPass.pProgram->addDefine("GS_OUT_STREAM", std::to_string(mIsmSceneType));
        ismPass.pProgram->setTypeConformances(mpScene->getTypeConformances());

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

        ismPass.pState = GraphicsState::create();
        ismPass.pState->setProgram(ismPass.pProgram);
        //ismPass.pState->setPrimType(GraphicsStateObject::PrimitiveType::Patch);

        ismPass.pVars = GraphicsVars::create(ismPass.pProgram->getReflector());

        ismPass.pFbo = Fbo::create();

        //mpScene->getMeshVao();
        //GraphicsStateObject::Desc gsoDesc;
        //gsoDesc.setPrimitiveType(GraphicsStateObject::PrimitiveType::Patch);
        //ismPass.pGSO->create(gsoDesc);
    }

    // Load ISM render pass
    Program::DefineList defines;
    mpIsmPullPass = createComputeShader("Pull.cs.slang", defines);
    mpIsmPushPass = createComputeShader("Push.cs.slang", defines);
    mpIsmVisualizePass = createComputeShader("ISM_Visualize.cs.slang", defines);
    mpShadingPass = createComputeShader("Shading.cs.slang", defines, "main", true);

    // Create resources
    {
        auto totalVertexCount = mpScene->getSceneStats().instancedVertexCount;
        auto totalTriangleCount = mpScene->getSceneStats().instancedTriangleCount;

        logInfo(std::format("Total Vertex Count = {}", totalVertexCount));
        logInfo(std::format("Total Triangle Count = {}", totalTriangleCount));
        logInfo(std::format("Total Instance Count = {}", mpScene->getGeometryInstanceCount()));
        

        // We need to write our own mipmap generatation method
        mpIsmTexture = Texture::create2D(mIsmTextureSize, mIsmTextureSize, ResourceFormat::R32Float, 1, 1, nullptr, kAutoMipBindFlags);
        mpPointsBuffer = Buffer::createTyped<float4>((uint32_t)std::min((uint64_t)Buffer::kMaxPossible, totalTriangleCount), kBufferBindFlags, Buffer::CpuAccess::Read);
        mpCounterBuffer = Buffer::createTyped<uint>(1, kBufferBindFlags, Buffer::CpuAccess::Read);

        if (mTotalLightsCount > 0)
        {
            mpIsmTextureArray = Texture::create2D(mIsmSize, mIsmSize, ResourceFormat::R32Float, mIsmPerLight * mTotalLightsCount, mIsmMipLevels, nullptr, kAutoMipBindFlags);
            prepareLightShadowMapData();
        }
    }
}

ComputePass::SharedPtr ISM::createComputeShader(const std::string& file, const Program::DefineList& defines, const std::string& entryPoint, bool hasScene)
{
    // Where is the shader?  What shader model to use?
    Program::Desc risDesc(kShaderDirectory + file);
    risDesc.setShaderModel("6_5");

    // Create a Falcor shader wrapper, with specified entry point and scene-specific #defines (for common Falcor HLSL routines)
    auto csDefines = defines;
    if (hasScene)
    {
        csDefines.add(mpScene->getSceneDefines());
    }
    ComputePass::SharedPtr pShader = ComputePass::create(risDesc.csEntry(entryPoint), csDefines);

    // Type conformances are needed for specific Slang language constructs used in common Falcor shaders, no need to worry
    // about this unless you're using advanced Slang functionality in a very specific way.
    pShader->getProgram()->setTypeConformances(mpScene->getTypeConformances());

    // Zero out structures to make sure they're regenerated with correct settings (conformances, scene defines, etc)
    pShader->setVars(nullptr);

    if (hasScene) pShader->getRootVar()["gScene"] = mpScene->getParameterBlock();

    return pShader;
}

void ISM::prepareLightShadowMapData()
{
    std::vector<LightShadowData> lightShadowDataList(mTotalLightsCount);

    for (uint i = 0; i < mTotalLightsCount; i++)
    {
        LightShadowData lightShadowData;
        auto pointLight = mPointLights[i];
        auto centerPosW = pointLight->getWorldPosition();
        lightShadowData.centerPosW = centerPosW;

        lightShadowData.viewMats[0] = glm::lookAt(centerPosW, centerPosW + glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f, 0.0f, -1.0f));
        lightShadowData.viewMats[1] = glm::lookAt(centerPosW, centerPosW + glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f));
        lightShadowData.persProjMat = glm::perspective(glm::radians(90.0f), 1.0f, mLightNearFarPlane.x, mLightNearFarPlane.y);

        lightShadowDataList[i] = lightShadowData;
    }

    mpLightShadowDataBuffer = Buffer::createStructured(ismPass.pVars->getRootVar()["gLightShadowDataBuffer"], mTotalLightsCount, kBufferBindFlags,
        Buffer::CpuAccess::None, lightShadowDataList.data());
}

void ISM::visualizeIsmTexture(Texture::SharedPtr pSrcTexture, Texture::SharedPtr pDstTexture, RenderContext* pRenderContext, uint mipLevel, uint arrayIndex)
{
    //auto pStagingTexIn = Texture::create2D(pSrcTexture->getWidth(), pSrcTexture->getHeight(), pSrcTexture->getFormat(), 1, 1, nullptr, kAutoMipBindFlags);
    //pRenderContext->copySubresource(pStagingTexIn.get(), 0, pSrcTexture.get(), arrayIndex);

    // Visualize single ISM
    uint2 nThreads = uint2(pSrcTexture->getWidth(), pSrcTexture->getHeight());
    auto visualizeVars = mpIsmVisualizePass->getRootVar();
    visualizeVars["VisCB"]["gLightNear"] = mLightNearFarPlane.x;
    visualizeVars["VisCB"]["gLightFar"] = mLightNearFarPlane.y;
    visualizeVars["VisCB"]["gPointSampler"] = mpPointSampler;
    visualizeVars["VisCB"]["gMipLevel"] = (float)mipLevel;
    visualizeVars["VisCB"]["gArrayIndex"] = (float)arrayIndex;
    visualizeVars["VisCB"]["gThreadDim"] = nThreads;
    visualizeVars["VisCB"]["gPointSampler"] = mpPointSampler;
    visualizeVars["VisCB"]["gLinearSampler"] = mpLinearSampler;
    visualizeVars["gIsmInput"] = pSrcTexture;
    visualizeVars["gSingleIsmOutput"] = pDstTexture;
    mpPixelDebug->prepareProgram(mpIsmVisualizePass->getProgram(), visualizeVars);
    mpIsmVisualizePass->execute(pRenderContext, nThreads.x, nThreads.y);

    // Combine all ISMs into one texture and visualize it
}
