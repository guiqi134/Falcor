#pragma once

#include <Falcor.h>
#include "AmbientOcclusionPass.h"
#include "diffuseGIPass.h"

using namespace Falcor;


class AccumulationPass : public std::enable_shared_from_this<AccumulationPass>
{
public:
    using SharedPtr = std::shared_ptr<AccumulationPass>;

    static SharedPtr create(uint32_t width, uint32_t height, Scene::SharedPtr pScene);
    void resize(uint32_t width, uint32_t height);
    void execute(RenderContext* pRenderContext);
    void renderGui(Gui* pGui);

    inline Texture::SharedPtr getOutputTexture() { return mpLastFrame; }

private:
    void initialize(Scene::SharedPtr pScene);
    bool hasCameraMoved();

    // get the output texture from ambient occlusion pass
    //AmbientOcclusionPass::SharedPtr mpAmbientOcclusionPass;
    DiffuseGIPass::SharedPtr mpDiffuseGIPass;

    // State for our accumulation shader
    FullScreenPass::SharedPtr mpAccumShader;
    Texture::SharedPtr mpLastFrame;
    Fbo::SharedPtr mpInternalFbo;

    // We stash a copy of our current scene.  Why?  To detect if changes have occurred.
    Scene::SharedPtr mpScene;
    float4x4 mpLastCameraMatrix;

    // Is our accumulation enabled?
    bool mDoAccumulation = true;

    // How many frames have we accumulated so far?
    uint32_t mAccumCount = 0;

    uint32_t mWidth;
    uint32_t mHeight;
};
