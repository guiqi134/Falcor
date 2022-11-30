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
#include "Utils/Sampling/SampleGenerator.h"

using namespace Falcor;

class ISM : public RenderPass
{
public:
    using SharedPtr = std::shared_ptr<ISM>;

    static const Info kInfo;

    /** Create a new render pass object.
        \param[in] pRenderContext The render context.
        \param[in] dict Dictionary of serialized parameters.
        \return A new object, or an exception is thrown if creation failed.
    */
    static SharedPtr create(RenderContext* pRenderContext = nullptr, const Dictionary& dict = {});

    virtual Dictionary getScriptingDictionary() override;
    virtual RenderPassReflection reflect(const CompileData& compileData) override;
    virtual void compile(RenderContext* pRenderContext, const CompileData& compileData) override {}
    virtual void execute(RenderContext* pRenderContext, const RenderData& renderData) override;
    virtual void renderUI(Gui::Widgets& widget) override;
    virtual void setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene);
    virtual bool onMouseEvent(const MouseEvent& mouseEvent) override { mpPixelDebug->onMouseEvent(mouseEvent); return false; };
    virtual bool onKeyEvent(const KeyboardEvent& keyEvent) override;

private:
    ISM(const Dictionary& dict);

    enum class ISMSceneType { Triangle, ThreePoints, CenterPoint };
    enum class ISMPushSamplingMode { Point, Interpolation };
    enum class VisibilityMode { ShadowRay, ISM };

    Scene::SharedPtr mpScene;
    SampleGenerator::SharedPtr mpSampleGenerator;
    PixelDebug::SharedPtr mpPixelDebug;
    GpuFence::SharedPtr mpFence;
    Sampler::SharedPtr mpPointSampler;
    Sampler::SharedPtr mpLinearSampler;

    Texture::SharedPtr mpIsmTexture;
    Texture::SharedPtr mpIsmTextureArray;
    Buffer::SharedPtr mpPointsBuffer;
    Buffer::SharedPtr mpLightShadowDataBuffer;
    Buffer::SharedPtr mpCounterBuffer;

    std::vector<PointLight*> mPointLights;

    uint mTotalLightsCount = 0;
    uint mFrameIndex = 0;
    uint2 mScreenSize;
    float2 mLightNearFarPlane = float2(0.001f, 50.0f);
    float mDepthBias = 0.003f;
    VisibilityMode mVisibilityMode = VisibilityMode::ISM;
    bool mDrawWireframe = false;
    bool mUpdateResources = false;
    bool mUpdateDefines = false;

    // ISM pass parameters
    uint mIsmTextureSize = 1024u; // max light = 16 * 8
    uint mIsmSize = 128u;
    uint mIsmPerLight = 2u;
    uint mIsmMipLevels = 3;
    float mSceneDepthThreasholdScale = 0.01f;
    uint mIsmPushMode = (uint)ISMPushSamplingMode::Point;
    uint mIsmSceneType = (uint)ISMSceneType::ThreePoints;

    // Visualize pass parameters
    uint mVisualizeMipLevel = 0;
    uint mVisLightID = 0;


    // Wireframe pass
    struct
    {
        GraphicsProgram::SharedPtr pProgram;
        GraphicsState::SharedPtr pState;
        GraphicsVars::SharedPtr pVars;
        Fbo::SharedPtr pFbo;
    } mWireframePass;

    // ISM pass
    struct
    {
        GraphicsProgram::SharedPtr pProgram;
        GraphicsState::SharedPtr pState;
        GraphicsStateObject::SharedPtr pGSO;
        GraphicsVars::SharedPtr pVars;
        Fbo::SharedPtr pFbo;
    } ismPass;

    ComputePass::SharedPtr mpIsmPullPass;
    ComputePass::SharedPtr mpIsmPushPass;
    ComputePass::SharedPtr mpShadingPass;

    struct
    {
        ComputePass::SharedPtr pVisualizeSingle;
        ComputePass::SharedPtr pVisualizeAll;
    } mIsmVisualizePass;

    RasterizerState::SharedPtr mpWireframeRS;
    DepthStencilState::SharedPtr mpNoDepthDS;
    DepthStencilState::SharedPtr mpDepthTestDS;


    ComputePass::SharedPtr createComputeShader(const std::string& file, const Program::DefineList& defines, const std::string& entryPoint = "main", bool hasScene = false);
    void prepareLightShadowMapData();
    void visualizeIsmTexture(ComputePass::SharedPtr pPass, Texture::SharedPtr pSrcTexture, Texture::SharedPtr pDstTexture, RenderContext* pRenderContext, uint mipLevel,
        uint arrayIndex, const std::string& mode);

    // This function only supports getting the typed buffer data
    template <typename T>
    std::vector<T> getDeviceResourceData(RenderContext* pRenderContext, const Resource::SharedPtr& pResource)
    {
        auto resourceType = pResource->getType();
        auto size = pResource->getSize();
        std::vector<T> deviceResult;
        if (resourceType == Resource::Type::Buffer)
        {
            auto pBuffer = pResource->asBuffer();
            auto elementCount = pBuffer->getElementCount();

            if (pBuffer->getCpuAccess() != Buffer::CpuAccess::Read)
            {
                logError("Cannot read the buffer");
            }

            // Must use a staging buffer which has no bind flags to map data
            auto pStagingBuffer = Buffer::createTyped<T>(elementCount, ResourceBindFlags::None, Buffer::CpuAccess::Read);
            pRenderContext->copyBufferRegion(pStagingBuffer.get(), 0, pBuffer.get(), 0, size);

            // Flush GPU and wait for results to be available.
            pRenderContext->flush(false);
            mpFence->gpuSignal(pRenderContext->getLowLevelData()->getCommandQueue());
            mpFence->syncCpu();

            // Read back GPU results
            T* pData = reinterpret_cast<T*>(pStagingBuffer->map(Buffer::MapType::Read));
            deviceResult.assign(pData, pData + elementCount); // invoke range constructor (linear time)
            pStagingBuffer->unmap();
        }

        return deviceResult;
    }
};
