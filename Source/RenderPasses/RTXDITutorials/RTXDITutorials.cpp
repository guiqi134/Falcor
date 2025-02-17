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
#include "Common/RTXDITutorialBase.h"
#include "Tutorial 1/RTXDITutorial1.h"
#include "Tutorial 2/RTXDITutorial2.h"
#include "Tutorial 3/RTXDITutorial3.h"
#include "Tutorial 4/RTXDITutorial4.h"
#include "Tutorial 5/RTXDITutorial5.h"
#include "Combined/RTXDICombinedTutorial.h"


using namespace Falcor;


// This is required for DLL and shader hot-reload to function properly
extern "C" __declspec(dllexport) const char* getProjDir()
{
    return PROJECT_DIR;
}

// What passes does this DLL expose?  Register them here
extern "C" __declspec(dllexport) void getPasses(Falcor::RenderPassLibrary& lib)
{
    lib.registerPass(RTXDITutorial1::getClassDescription(), RTXDITutorial1::create);
    lib.registerPass(RTXDITutorial2::getClassDescription(), RTXDITutorial2::create);
    lib.registerPass(RTXDITutorial3::getClassDescription(), RTXDITutorial3::create);
    lib.registerPass(RTXDITutorial4::getClassDescription(), RTXDITutorial4::create);
    lib.registerPass(RTXDITutorial5::getClassDescription(), RTXDITutorial5::create);

    lib.registerPass(RTXDICombinedTutorial::getClassDescription(), RTXDICombinedTutorial::create);
}


