#include "GBufferPass.h"

namespace
{
    // If we want to jitter the camera to antialias using traditional a traditional 8x MSAA pattern,
    //     use these positions (which are in the range [-8.0...8.0], so divide by 16 before use)
    const float kMSAA[8][2] = { {1,-3}, {-1,3}, {5,1}, {-3,-5}, {-5,5}, {-7,-1}, {3,7}, {7,-7} };
}

GBufferPass::SharedPtr GBufferPass::Create(uint32_t width, uint32_t height, Scene::SharedPtr pScene)
{
    GBufferPass::SharedPtr result = SharedPtr(new GBufferPass());

    result->Resize(width, height);
    result->InitPipelineResource(pScene);

    return result;
}

/*
Create FBO And Relavant Resources
*/
void GBufferPass::Resize(uint32_t width, uint32_t height)
{
    /*if (m_DepthPass)
    {
        m_DepthPass->Resize(width, height);
    }*/

    Fbo::Desc desc;
    desc.setSampleCount(0);
    desc.setColorTarget(0, Falcor::ResourceFormat::RGBA16Float);    // Diffuse
    desc.setColorTarget(1, Falcor::ResourceFormat::RGBA16Float);    // Spec
    desc.setColorTarget(2, Falcor::ResourceFormat::RGBA32Float);    // Pos
    desc.setColorTarget(3, Falcor::ResourceFormat::RGBA32Float);    // Normal
    desc.setColorTarget(4, Falcor::ResourceFormat::RGBA16Float);    // Extra
    desc.setColorTarget(5, Falcor::ResourceFormat::RGBA16Float);    // Emissive
    desc.setColorTarget(6, Falcor::ResourceFormat::RGBA16Float);    // FaceN
    desc.setDepthStencilTarget(ResourceFormat::D32Float);

    m_Fbo = Fbo::create2D(width, height, desc);

    m_MotionTexture = Texture::create2D(width, height, ResourceFormat::RG32Float, 1U, 1U, nullptr, ResourceBindFlags::UnorderedAccess | ResourceBindFlags::ShaderResource);
    m_LinearZAndDriv = Texture::create2D(width, height, ResourceFormat::RGBA32Float, 1U, 1U, nullptr, ResourceBindFlags::UnorderedAccess | ResourceBindFlags::ShaderResource);
    m_PositionAndNormalFWidth = Texture::create2D(width, height, ResourceFormat::RGBA32Float, 1U, 1U, nullptr, ResourceBindFlags::UnorderedAccess | ResourceBindFlags::ShaderResource);

    //Fbo::Desc ZNDesc;
    //ZNDesc.setColorTarget(0, Falcor::ResourceFormat::RGBA32Float);
    //m_LinearZAndNormalFbo = Fbo::create2D(width, height, ZNDesc);

    //m_InternalPreviousLinearZAndNormalRT = Texture::create2D(width, height, ResourceFormat::RGBA32Float, 1, 1, nullptr, Resource::BindFlags::RenderTarget | Resource::BindFlags::ShaderResource);

    m_Width = width;
    m_Height = height;
}

void GBufferPass::renderGui(Gui* pGui)
{

}

void GBufferPass::Execute(RenderContext* renderContext)
{
    Camera::SharedPtr camera = mpScene->getCamera();
    camera->setAspectRatio((float)m_Width / (float)m_Height);

    // Are we jittering?  If so, update our camera with the current jitter
    if (mUseJitter && mpScene && mpScene->getCamera())
    {
        // Increase our frame count
        mFrameCount++;

        // Determine our offset in the pixel in the range [-0.5...0.5]
        float xOff = mUseRandom ? mRngDist(mRng) - 0.5f : kMSAA[mFrameCount % 8][0] * 0.0625f;
        float yOff = mUseRandom ? mRngDist(mRng) - 0.5f : kMSAA[mFrameCount % 8][1] * 0.0625f;

        // Give our jitter to the scene camera
        mpScene->getCamera()->setJitter(xOff / float(m_Fbo->getWidth()), yOff / float(m_Fbo->getHeight()));
    }

    m_ProgramVars["motionVectors"] = m_MotionTexture;
    m_ProgramVars["gPosNormalFwidth"] = m_PositionAndNormalFWidth;
    m_ProgramVars["gLinearZAndDeriv"] = m_LinearZAndDriv;

    m_ProgramVars["PerFrameCB"]["frameSize"] = float2(m_Width, m_Height);
    m_ProgramVars["PerFrameCB"]["cameraPos"] = camera->getPosition();

    const float4 clearColor(0.0f, 0.0f, 0.0f, 0.0f);
    renderContext->clearFbo(m_Fbo.get(), clearColor, 1.0f, 0, FboAttachmentType::All);

    renderContext->clearTexture(m_MotionTexture.get());
    renderContext->clearTexture(m_PositionAndNormalFWidth.get());
    renderContext->clearTexture(m_LinearZAndDriv.get());

    // Update Viewport
    m_GraphicsState->setFbo(m_Fbo);

    mpScene->rasterize(renderContext, m_GraphicsState.get(), m_ProgramVars.get(), Scene::RenderFlags::UserRasterizerState);

    //auto perImageCB = m_PackLinearZAndNormal["PerImageCB"];
    //perImageCB["gLinearZ"] = GetLinearZTexture();
    //perImageCB["gNormal"] = GetShadingNormalTexture();

    //renderContext->blit(m_LinearZAndNormalFbo->getColorTexture(0)->getSRV(),
    //    m_InternalPreviousLinearZAndNormalRT->getRTV());


    //m_PackLinearZAndNormal->execute(renderContext, m_LinearZAndNormalFbo);
}

// set shader data in other passes, like when doing a second light shading pass
void GBufferPass::SetShaderData(const ShaderVar& var) const
{
    var["_TexAlbedo"].setTexture(m_Fbo->getColorTexture(0));
    var["_TexSpec"].setTexture(m_Fbo->getColorTexture(1));
    var["_TexPosition"].setTexture(m_Fbo->getColorTexture(2));
    var["_TexShadingNormal"].setTexture(m_Fbo->getColorTexture(3));
    var["_TexExtra"].setTexture(m_Fbo->getColorTexture(4));
    var["_TexEmissive"].setTexture(m_Fbo->getColorTexture(5));
    var["_TexFaceNormal"].setTexture(m_Fbo->getColorTexture(6));
}

void GBufferPass::InitPipelineResource(Scene::SharedPtr pScene)
{
    //m_DepthPass = DepthPass::Create(m_Width, m_Height);

    mpScene = pScene;

    //Program::DefineList defines = { { "_DEFAULT_ALPHA_TEST", "" } };
    Program::Desc desc;
    desc.addShaderLibrary("Samples/ProjectTemplate/GBufferPass.3d.slang").vsEntry("vsMain").psEntry("psMain");
    desc.setShaderModel("6_2");
    //m_Program = GraphicsProgram::create(desc, defines);
    m_Program = GraphicsProgram::create(desc);

    m_GraphicsState = GraphicsState::create();

    RasterizerState::Desc rasterStateDesc;
    rasterStateDesc.setCullMode(RasterizerState::CullMode::Back);
    m_RasterizerState = RasterizerState::create(rasterStateDesc);

    DepthStencilState::Desc depthStancilDesc;
    depthStancilDesc.setDepthFunc(ComparisonFunc::Less).setDepthEnabled(true);
    m_DepthStencilState = DepthStencilState::create(depthStancilDesc);

    m_Program->addDefines(mpScene->getSceneDefines());
    m_ProgramVars = GraphicsVars::create(m_Program->getReflector());

    m_GraphicsState->setRasterizerState(m_RasterizerState);
    m_GraphicsState->setDepthStencilState(m_DepthStencilState);

    m_GraphicsState->setProgram(m_Program);

    m_PackLinearZAndNormal = FullScreenPass::create("Samples/ProjectTemplate/GBufferPackLinearZAndNormal.ps.slang");

    // Set up our random number generator by seeding it with the current time 
    auto currentTime = std::chrono::high_resolution_clock::now();
    auto timeInMillisec = std::chrono::time_point_cast<std::chrono::milliseconds>(currentTime);
    mRng = std::mt19937(uint32_t(timeInMillisec.time_since_epoch().count()));
}
