#include "diffuseGIPass.h"

// Some global vars, used to simplify changing shader location & entry points
namespace {
    // Where is our shaders located?
    const char* kFileRayTrace = "Samples/ProjectTemplate/diffuseGI.rt.slang";

    // What are the entry points in that shader for various ray tracing shaders?
    const char* kEntryPointRayGen = "SimpleDiffuseGIRayGen";

    const char* kEntryPointMiss0 = "ShadowMiss";
    const char* kEntryShadowAnyHit = "ShadowAnyHit";
    const char* kEntryShadowClosestHit = "ShadowClosestHit";

    const char* kEntryPointMiss1 = "IndirectMiss";
    const char* kEntryIndirectAnyHit = "IndirectAnyHit";
    const char* kEntryIndirectClosestHit = "IndirectClosestHit";
};

DiffuseGIPass::SharedPtr DiffuseGIPass::create(uint32_t width, uint32_t height, Scene::SharedPtr pScene)
{
    DiffuseGIPass::SharedPtr result = SharedPtr(new DiffuseGIPass());

    result->resize(width, height);
    result->initialize(pScene);

    return result;
}

void DiffuseGIPass::resize(uint32_t width, uint32_t height)
{
    mpRtOut = Texture::create2D(width, height, ResourceFormat::RGBA32Float, 1U, 1U, nullptr, ResourceBindFlags::AllColorViews);

    mWidth = width;
    mHeight = height;
}

void DiffuseGIPass::initialize(Scene::SharedPtr pScene)
{
    mpScene = pScene;
    mpGBufferPass = GBufferPass::Create(mWidth, mHeight, mpScene);
    if (!mpScene) return;

    RtProgram::Desc rtProgDesc;
    rtProgDesc.addShaderLibrary(kFileRayTrace).setRayGen(kEntryPointRayGen);
    rtProgDesc.addHitGroup(0, kEntryShadowClosestHit, kEntryShadowAnyHit).addMiss(0, kEntryPointMiss0);
    rtProgDesc.addHitGroup(1, kEntryIndirectClosestHit, kEntryIndirectAnyHit).addMiss(1, kEntryPointMiss1);
    rtProgDesc.addDefines(mpScene->getSceneDefines());

    mpRaytraceProgram = RtProgram::create(rtProgDesc);
    mpRtVars = RtProgramVars::create(mpRaytraceProgram, mpScene);
    mpRaytraceProgram->setScene(mpScene);
}

void DiffuseGIPass::renderGui(Gui* pGui)
{
    mpGBufferPass->renderGui(pGui);
}

void DiffuseGIPass::execute(RenderContext* pRenderContext)
{
    mpGBufferPass->Execute(pRenderContext);

    Camera::SharedPtr camera = mpScene->getCamera();
    camera->setAspectRatio((float)mWidth / (float)mHeight);

    // Set our shader variables for the ray generation shader
    mpRtVars["RayGenCB"]["gMinT"] = 1.0e-4f;
    mpRtVars["RayGenCB"]["gFrameCount"] = mFrameCount++;
    mpRtVars["RayGenCB"]["gDoIndirectGI"] = mDoIndirectGI;
    mpRtVars["RayGenCB"]["gCosSampling"] = mDoCosSampling;
    mpRtVars["RayGenCB"]["gDirectShadow"] = mDoDirectShadows;

    // Pass our G-buffer textures down to the HLSL so we can shade
    mpRtVars["gPos"] = mpGBufferPass->GetPositionTexture();
    mpRtVars["gNorm"] = mpGBufferPass->GetShadingNormalTexture();
    mpRtVars["gDiffuseMatl"] = mpGBufferPass->GetAlbedoTexture();
    mpRtVars["gOutput"] = mpRtOut;


    mpScene->raytrace(pRenderContext, mpRaytraceProgram.get(), mpRtVars, uint3(mWidth, mHeight, 1));
}

