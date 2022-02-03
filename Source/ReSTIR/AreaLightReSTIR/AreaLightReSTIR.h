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
#pragma once
#include "Falcor.h"
#include "FalcorExperimental.h"
#include "Experimental/Scene/Lights/EmissivePowerSampler.h"
#include "Utils/Sampling/SampleGenerator.h"
#include "RenderPasses/Shared/AreaLight/SceneHelpers.h"


using namespace Falcor;

class AreaLightReSTIR : public RenderPass
{
public:
    using SharedPtr = std::shared_ptr<AreaLightReSTIR>;

    /** Create a new render pass object.
        \param[in] pRenderContext The render context.
        \param[in] dict Dictionary of serialized parameters.
        \return A new object, or an exception is thrown if creation failed.
    */
    static SharedPtr create(RenderContext* pRenderContext = nullptr, const Dictionary& dict = {});

    virtual std::string getDesc() override;
    virtual Dictionary getScriptingDictionary() override;
    virtual RenderPassReflection reflect(const CompileData& compileData) override;
    virtual void compile(RenderContext* pContext, const CompileData& compileData) override {}
    virtual void execute(RenderContext* pRenderContext, const RenderData& renderData) override;
    virtual void renderUI(Gui::Widgets& widget) override;
    virtual void setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene) override;
    virtual bool onMouseEvent(const MouseEvent& mouseEvent) override { return mpPixelDebug->onMouseEvent(mouseEvent); }
    virtual bool onKeyEvent(const KeyboardEvent& keyEvent) override { return false; }

private:
    AreaLightReSTIR();

    void AddDefines(ComputePass::SharedPtr pass, Shader::DefineList& defines);
    void RemoveDefines(ComputePass::SharedPtr pass, Shader::DefineList& defines);
    void CreatePass(ComputePass::SharedPtr& pass, const char* path, Shader::DefineList& defines);
    void CreatePasses();
    void UpdateDefines();
    void calcLightSpaceMatrix();
    void setPCSSShaderData(const ShaderVar& var);

    PixelDebug::SharedPtr mpPixelDebug;

    Scene::SharedPtr                    mpScene;
    SampleGenerator::SharedPtr          mpSampleGenerator;

    // Passes
    struct
    {
        GraphicsProgram::SharedPtr pProgram;
        GraphicsVars::SharedPtr pVars;
        GraphicsState::SharedPtr pState;
        Fbo::SharedPtr pFbo;
    } mShadowMapPass;

    ComputePass::SharedPtr              mpInitialSamplingPass;
    ComputePass::SharedPtr              mpTemporalResamplingPass;
    ComputePass::SharedPtr              mpSpatialResamplingPass;
    ComputePass::SharedPtr              mpShadingPass;

    // Resources
    Texture::SharedPtr                  mpVBufferPrev;
    Buffer::SharedPtr                   mpReservoirBuffer; // contain both current & previous reservoirs?
    Buffer::SharedPtr                   mpNeighborOffsetBuffer;

    // Parameters
    uint                                mLastFrameOutputReservoir = 0;
    uint                                mCurrentFrameOutputReservoir = 0;

    bool                                mBruteForce = false;
    bool                                mEnableSpatialResampling = false;
    bool                                mEnableTemporalResampling = false;
    bool                                mStoreFinalVisibility = false; // Cause bugs in temporal reuse 
    bool                                mNeedUpdateDefines = false;

    uint mInitialAreaLightSamples = 32u; // M candiates 
    uint mBlockerSearchSamples = 32u;
    uint mPCFSamples = 64u;
    uint mActiveTargetPdf = 1;
    uint mActiveShadingMode = 0;
    uint mRasterAlphaTest = 1;

    float mMinVisibility = 0.001f; 

    // Area light parameters
    float3 mLightUp = float3(0.0f, 0.0f, -1.0f);
    float3 mLightPos = float3(14.1f, 1500.0f, 15.0f);
    float4 mRotation = float4(0.3f, 1.0f, 0.0f, 0.0f);
    float mLightSize = 250.0f; // for diameter
    float mFovY = 0.8f;
    float mLightNearPlane = mLightPos.y - 20.0f;

    /*float3 mLightPos = float3(14.1f, 50.0f, 15.0f);
    float4 mRotation = float4(0.3f, 1.0f, 0.0f, 0.0f);
    float mLightSize = 10.0f;
    float mFovY = 30.0f;*/

    float4x4 mLightSpaceMat; // light space transform matrix
    float4x4 mLightView;
    float4x4 mLightProj;
};
