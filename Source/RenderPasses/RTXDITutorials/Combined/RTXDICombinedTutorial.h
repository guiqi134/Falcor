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

/** This render pass can be used to instantiate a renderer that allows the user
    to choose which of the RTXDI tutorial render modes are used dynamically (and
    to switch between them easily for comparison).

    This is not necessarily meant to be easy to read, it is meant to be easy to
    *use* when you run it.  But if you understand the individual tutorials, this
    class simply combines them all together into one render pass.
*/

#include "../Common/RTXDITutorialBase.h"
#include "../Common/RTXDITutorialConstants.h"
#include "../Tutorial 1/RTXDITutorial1.h"
#include "../Tutorial 2/RTXDITutorial2.h"
#include "../Tutorial 3/RTXDITutorial3.h"
#include "../Tutorial 4/RTXDITutorial4.h"
#include "../Tutorial 5/RTXDITutorial5.h"

using namespace Falcor;

class RTXDICombinedTutorial : public RTXDITutorial5, RTXDITutorial4, RTXDITutorial3, RTXDITutorial2, RTXDITutorial1
{
public:
    // Simplify notation. 
    using SharedPtr = std::shared_ptr<RTXDICombinedTutorial>;

    // Used to help expose this RenderPass outside the RTXDITutorials.dll library.  These aren't
    //    particularly important (see them used in RTXDITutorials.cpp).  But they help provide names to
    //    interface this RenderPass using Falcor's Python-based system for executing passes via Mogwai.exe
    static const RenderPass::Info getClassDescription();
    static SharedPtr create(RenderContext* pRenderContext = nullptr, const Dictionary& dict = {});

    // Overrides to the standard RenderPass callback to describe how rendering behaves 
    virtual void execute(RenderContext* pRenderContext, const RenderData& renderData) override;

    // Overrides to the standard RenderPass callback construct our Imgui GUI user interface 
    virtual void renderUI(Gui::Widgets& widget) override;

protected:
    RTXDICombinedTutorial(const Dictionary& dict);

};
