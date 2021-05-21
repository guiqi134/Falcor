#pragma once
#include "Falcor.h"
using namespace Falcor;

class GBufferPass : public std::enable_shared_from_this<GBufferPass>
{
public:
    using SharedPtr = std::shared_ptr<GBufferPass>;
    static GBufferPass::SharedPtr Create(uint32_t width, uint32_t height, Scene::SharedPtr pScene);

    void Resize(uint32_t width, uint32_t height);
    void renderGui(Gui* pGui);
    void Execute(RenderContext* renderContext);

    void SetShaderData(const ShaderVar& var) const;

    inline Fbo::SharedPtr GetFbo() { return m_Fbo; }

    /* albedo(rgb), opacity */
    inline Texture::SharedPtr GetAlbedoTexture() { return m_Fbo->getColorTexture(0); }
    /* spcular(rgb), linear roughness */
    inline Texture::SharedPtr GetSpecTexture() { return m_Fbo->getColorTexture(1); }
    /* position(rgb), packer */
    inline Texture::SharedPtr GetPositionTexture() { return m_Fbo->getColorTexture(2); }
    /* normal(rgb), distance to camera */
    inline Texture::SharedPtr GetShadingNormalTexture() { return m_Fbo->getColorTexture(3); }
    /* ior, double sided, packer, packer */
    inline Texture::SharedPtr GetExtraTexture() { return m_Fbo->getColorTexture(4); }
    /* Emissive RGB, packer */
    inline Texture::SharedPtr GetEmissiveTexture() { return m_Fbo->getColorTexture(5); }
    inline Texture::SharedPtr GetFaceNormal() { return m_Fbo->getColorTexture(6); }
    /* RWTexture2D<float2> MotionVectors */
    inline Texture::SharedPtr GetMotionTexture() { return m_MotionTexture; }
    inline Texture::SharedPtr GetDepthTexture()
    {
        return m_Fbo->getDepthStencilTexture();
        //return m_DepthPass->GetDepthTexture();
    }

    inline Texture::SharedPtr   GetLinearZTexture() { return m_LinearZAndDriv; }
    inline Texture::SharedPtr   GetPositionAndNormalFWidthTexture() { return m_PositionAndNormalFWidth; }

    inline Texture::SharedPtr   GetLinearNormalTexture() { return m_LinearZAndNormalFbo->getColorTexture(0); }
    inline Texture::SharedPtr   GetPrevLinearNormalTexture() { return m_InternalPreviousLinearZAndNormalRT; }

private:

    void InitPipelineResource(Scene::SharedPtr pScene);

    Scene::SharedPtr mpScene;
    Fbo::SharedPtr                  m_Fbo;
    Texture::SharedPtr              m_MotionTexture;
    Texture::SharedPtr              m_LinearZAndDriv;
    Texture::SharedPtr              m_PositionAndNormalFWidth;

    Fbo::SharedPtr                  m_LinearZAndNormalFbo;
    Texture::SharedPtr              m_InternalPreviousLinearZAndNormalRT;

    /*
    For Rasterized GBuffer
    */
    GraphicsProgram::SharedPtr      m_Program = nullptr;
    GraphicsVars::SharedPtr         m_ProgramVars = nullptr;
    GraphicsState::SharedPtr        m_GraphicsState = nullptr;

    RasterizerState::SharedPtr      m_RasterizerState = nullptr;
    DepthStencilState::SharedPtr    m_DepthStencilState = nullptr;

    FullScreenPass::SharedPtr       m_PackLinearZAndNormal;

    // Jittering
    bool                        mUseJitter = true;      ///< Jitter the camera?
    bool                        mUseRandom = false;     ///< If jittering, use random samples or 8x MSAA pattern?
    int                         mFrameCount = 0;        ///< If jittering the camera, which frame in our jitter are we on?

    // Our random number generator (if we're doing randomized samples)
    std::uniform_real_distribution<float> mRngDist;     ///< We're going to want random #'s in [0...1] (the default distribution)
    std::mt19937 mRng;                                  ///< Our random number generator.  Set up in initialize()


    uint32_t m_Width;
    uint32_t m_Height;
};

