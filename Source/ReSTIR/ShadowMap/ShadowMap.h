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
#include "RenderPasses/Shared/AreaLight/SceneHelpers.h"

using namespace Falcor;

class ShadowMap : public RenderPass
{
public:
    using SharedPtr = std::shared_ptr<ShadowMap>;

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
    ShadowMap();
    void calcLightSpaceMatrix();
    void updateDefines();

    PixelDebug::SharedPtr mpPixelDebug;

    Scene::SharedPtr mpScene;
    Texture::SharedPtr mpShadowMap;

    float4x4 mLightSpaceMat; // light space transform matrix, it is static now
    float4x4 mLightView;
    float3 mLightPos = float3(0.0f, 6.0f, -9.0f);
    float3 mLightUp = float3(0.0f, 1.0f, 0.0f);
    float mLightSize = 1.0f;
    float mLightNearPlane = 8.0f;
    float mFovY = 45.0f;
    float4 mRotation = float4(30.0f, 1.0f, 0.0f, 0.0f);

    bool mNeedUpdate = false;
    bool mEnableShadow = true;
    uint mBlockerSearchSamples = 32u;
    uint mPCFSamples = 64u;

    struct
    {
        GraphicsProgram::SharedPtr pProgram;
        GraphicsVars::SharedPtr pVars;
        GraphicsState::SharedPtr pState;
        Fbo::SharedPtr pFbo; 
    } mPCSSPass;

    struct
    {
        GraphicsProgram::SharedPtr pProgram;
        GraphicsVars::SharedPtr pVars;
        GraphicsState::SharedPtr pState;
        Fbo::SharedPtr pFbo; 
    } mShadingPass;
};
