#include "AmbientOcclusionPass.h"

AmbientOcclusionPass::SharedPtr AmbientOcclusionPass::create(uint32_t width, uint32_t height, Scene::SharedPtr pScene)
{
    AmbientOcclusionPass::SharedPtr result = SharedPtr(new AmbientOcclusionPass());

    result->resize(width, height);
    result->initialize(pScene);

    return result;
}

void AmbientOcclusionPass::resize(uint32_t width, uint32_t height)
{

    mpRtOut = Texture::create2D(width, height, ResourceFormat::RGBA32Float, 1U, 1U, nullptr, ResourceBindFlags::AllColorViews);

    mWidth = width;
    mHeight = height;
}

void AmbientOcclusionPass::initialize(Scene::SharedPtr pScene)
{
    mpScene = pScene;
    mpGBufferPass = GBufferPass::Create(mWidth, mHeight, mpScene);

    if (!mpScene) return;

    RtProgram::Desc rtProgDesc;
    rtProgDesc.addShaderLibrary("Samples/ProjectTemplate/aoTracing.rt.slang").setRayGen("AoRayGen");
    rtProgDesc.addHitGroup(0, "", "AoAnyHit").addMiss(0, "AoMiss");
    rtProgDesc.addDefines(mpScene->getSceneDefines());

    mpRaytraceProgram = RtProgram::create(rtProgDesc);
    mpRtVars = RtProgramVars::create(mpRaytraceProgram, mpScene);
    mpRaytraceProgram->setScene(mpScene);
}

void AmbientOcclusionPass::renderGui(Gui* pGui)
{
    Gui::Window w(pGui, "Config", {300, 200});
    w.var("AO radius", mAORadius, 1e-4f, 1e38f, mAORadius * 0.01f);
    w.var("Num AO Rays", mNumRaysPerPixel, 1, 200);
}

void AmbientOcclusionPass::execute(RenderContext* pRenderContext)
{
    mpGBufferPass->Execute(pRenderContext);

    Camera::SharedPtr camera = mpScene->getCamera();
    camera->setAspectRatio((float)mWidth / (float)mHeight);

    // Set our ray tracing shader variables for our ray generation shader
    mpRtVars["RayGenCB"]["gFrameCount"] = mFrameCount++;
    mpRtVars["RayGenCB"]["gAORadius"] = mAORadius;
    mpRtVars["RayGenCB"]["gMinT"] = 1.0e-4f;  // From the UI dropdown
    mpRtVars["RayGenCB"]["gNumRays"] = uint32_t(mNumRaysPerPixel);
    mpRtVars["gPos"] = mpGBufferPass->GetPositionTexture();
    mpRtVars["gNorm"] = mpGBufferPass->GetShadingNormalTexture();
    mpRtVars["gOutput"] = mpRtOut;

    mpScene->raytrace(pRenderContext, mpRaytraceProgram.get(), mpRtVars, uint3(mWidth, mHeight, 1));
}

