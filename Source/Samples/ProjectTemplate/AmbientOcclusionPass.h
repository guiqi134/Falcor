#pragma once

#include <Falcor.h>
#include "GBufferPass.h"

using namespace Falcor;


class AmbientOcclusionPass : public std::enable_shared_from_this<AmbientOcclusionPass>
{
public:
    using SharedPtr = std::shared_ptr<AmbientOcclusionPass>;

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

    float mAORadius = 1.0f;
    uint32_t mFrameCount = 0;
    int32_t mNumRaysPerPixel = 1;

    uint32_t mWidth;
    uint32_t mHeight;
};
