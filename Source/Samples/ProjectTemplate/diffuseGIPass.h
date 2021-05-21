#pragma once

#include <Falcor.h>
#include "GBufferPass.h"

using namespace Falcor;


class DiffuseGIPass : public std::enable_shared_from_this<DiffuseGIPass>
{
public:
    using SharedPtr = std::shared_ptr<DiffuseGIPass>;

    static SharedPtr create(uint32_t width, uint32_t height, Scene::SharedPtr pScene);
    void resize(uint32_t width, uint32_t height);
    void execute(RenderContext* pRenderContext);
    void renderGui(Gui* pGui);

    inline Texture::SharedPtr getOutputTexture() { return mpRtOut; }

private:
    void initialize(Scene::SharedPtr pScene);

    GBufferPass::SharedPtr mpGBufferPass;

    Scene::SharedPtr mpScene;

    RtProgram::SharedPtr mpRaytraceProgram = nullptr;
    RtProgramVars::SharedPtr mpRtVars;

    Texture::SharedPtr mpRtOut;

    // Recursive ray tracing can be slow.  Add a toggle to disable, to allow you to manipulate the scene
    bool mDoIndirectGI = false;
    bool mDoCosSampling = true;
    bool mDoDirectShadows = false;

    // Various internal parameters
    uint32_t mFrameCount = 0x1337u;  ///< A frame counter to vary random numbers over time

    uint32_t mWidth;
    uint32_t mHeight;
};
