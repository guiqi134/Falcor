#include "AccumulationPass.h"

AccumulationPass::SharedPtr AccumulationPass::create(uint32_t width, uint32_t height, Scene::SharedPtr pScene)
{
    AccumulationPass::SharedPtr result = SharedPtr(new AccumulationPass());


    result->resize(width, height);
    result->initialize(pScene);

    return result;
}

void AccumulationPass::resize(uint32_t width, uint32_t height)
{
    //mpAmbientOcclusionPass = AmbientOcclusionPass::create(width, height);

    // Create / resize a texture to store the previous frame.
    //    Parameters: width, height, texture format, texture array size, #mip levels, initialization data, how we expect to use it.
    mpLastFrame = Texture::create2D(width, height, ResourceFormat::RGBA32Float, 1, 1, nullptr, ResourceBindFlags::AllColorViews);

    // We need a framebuffer to attach to our graphics pipe state (when running our full-screen pass).  We can ask our
    //    resource manager to create one for us, with specified width, height, and format and one color buffer.
    mpInternalFbo = Fbo::create2D(width, height, ResourceFormat::RGBA32Float);

    // Whenever we resize, we'd better force accumulation to restart
    mAccumCount = 0;

    mWidth = width;
    mHeight = height;
}

void AccumulationPass::initialize(Scene::SharedPtr pScene)
{
    mpScene = pScene;
    mpDiffuseGIPass = DiffuseGIPass::create(mWidth, mHeight, mpScene);

    mpAccumShader = FullScreenPass::create("Samples/ProjectTemplate/accumulate.ps.slang");

    // Grab a copy of the current scene's camera matrix (if it exists)
    if (mpScene && mpScene->getCamera())
        mpLastCameraMatrix = mpScene->getCamera()->getViewMatrix();
}

void AccumulationPass::renderGui(Gui* pGui)
{
    //mpAmbientOcclusionPass->renderGui(pGui);
    mpDiffuseGIPass->renderGui(pGui);

    Gui::Window w(pGui, "AccumulationPass", { 400, 200 });
    if (w.checkbox(mDoAccumulation ? "Accumulating samples temporally" : "No temporal accumulation", mDoAccumulation))
        mAccumCount = 0;

    w.text("");
    w.text((std::string("Frames accumulated: ") + std::to_string(mAccumCount)).c_str());
}

bool AccumulationPass::hasCameraMoved()
{
    // Has our camera moved?
    return mpScene &&                      // No scene?  Then the answer is no
        mpScene->getCamera() &&   // No camera in our scene?  Then the answer is no
        (mpLastCameraMatrix != mpScene->getCamera()->getViewMatrix());   // Compare the current matrix with the last one
}

void AccumulationPass::execute(RenderContext* pRenderContext)
{
    //mpAmbientOcclusionPass->execute(pRenderContext);
    mpDiffuseGIPass->execute(pRenderContext);

    if (!mDoAccumulation) return;

    // If the camera in our current scene has moved, we want to reset accumulation
    if (hasCameraMoved())
    {
        mAccumCount = 0;
        mpLastCameraMatrix = mpScene->getCamera()->getViewMatrix();
    }

    // Set shader parameters for our accumulation pass
    mpAccumShader["PerFrameCB"]["gAccumCount"] = mAccumCount++;
    mpAccumShader["gLastFrame"] = mpLastFrame;
    mpAccumShader["gCurFrame"] = mpDiffuseGIPass->getOutputTexture();

    // Execute the accumulation shader
    mpAccumShader->execute(pRenderContext, mpInternalFbo);

    // We've accumulated our result.  Copy that back to the input/output buffer
    //pRenderContext->blit(mpInternalFbo->getColorTexture(0)->getSRV(), inputTexture->getRTV());

    // Also keep a copy of the current accumulation for use next frame 
    pRenderContext->blit(mpInternalFbo->getColorTexture(0)->getSRV(), mpLastFrame->getRTV());
}

