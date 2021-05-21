/***************************************************************************
 # Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
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
#include "ProjectTemplate.h"
#include "AccumulationPass.h"
//#include "AmbientOcclusionPass.h"

uint32_t mSampleGuiWidth = 250;
uint32_t mSampleGuiHeight = 200;
uint32_t mSampleGuiPositionX = 20;
uint32_t mSampleGuiPositionY = 40;

static const float4 kClearColor(0.38f, 0.52f, 0.10f, 1);

void ProjectTemplate::onGuiRender(Gui* pGui)
{
    Gui::Window w(pGui, "Falcor", { 250, 200 });
    gpFramework->renderGlobalUI(pGui);
    w.text("Hello from ProjectTemplate");
    if (w.button("Click Here"))
    {
        msgBox("Now why would you do that?");
    }

    mpAccumulationPass->renderGui(pGui);
    mpScene->renderUI(w);
}

void ProjectTemplate::onLoad(RenderContext* pRenderContext)
{
    //mpScene = Scene::create("living_room/livingRoom.pyscene");
    mpScene = Scene::create("VPLMedia/materialBall/materialBall.pyscene");
    mpAccumulationPass = AccumulationPass::create(1600, 900, mpScene);
}

void ProjectTemplate::onFrameRender(RenderContext* pRenderContext, const Fbo::SharedPtr& pTargetFbo)
{
    pRenderContext->clearFbo(pTargetFbo.get(), kClearColor, 1.0f, 0, FboAttachmentType::All);
    mpAccumulationPass->execute(pRenderContext);

    pRenderContext->blit(mpAccumulationPass->getOutputTexture()->getSRV(), pTargetFbo->getRenderTargetView(0)); // src -> dst
    TextRenderer::render(pRenderContext, gpFramework->getFrameRate().getMsg(), pTargetFbo, { 20, 20 });
}

void ProjectTemplate::onShutdown()
{
}

bool ProjectTemplate::onKeyEvent(const KeyboardEvent& keyEvent)
{
    return false;
}

bool ProjectTemplate::onMouseEvent(const MouseEvent& mouseEvent)
{
    return false;
}

void ProjectTemplate::onHotReload(HotReloadFlags reloaded)
{
}

void ProjectTemplate::onResizeSwapChain(uint32_t width, uint32_t height)
{
}

int WINAPI WinMain(_In_ HINSTANCE hInstance, _In_opt_ HINSTANCE hPrevInstance, _In_ LPSTR lpCmdLine, _In_ int nShowCmd)
{
    ProjectTemplate::UniquePtr pRenderer = std::make_unique<ProjectTemplate>();
    SampleConfig config;
    config.windowDesc.title = "Falcor Project Template";
    config.windowDesc.resizableWindow = false;
    config.windowDesc.width = 1600;
    config.windowDesc.height = 900;
    Sample::run(config, pRenderer);
    return 0;
}
