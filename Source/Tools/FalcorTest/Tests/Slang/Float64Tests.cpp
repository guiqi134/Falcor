/***************************************************************************
 # Copyright (c) 2015-23, NVIDIA CORPORATION. All rights reserved.
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
#include "Testing/UnitTest.h"
#include "Utils/HostDeviceShared.slangh"
#include <fstd/bit.h> // TODO C++20: Replace with <bit>
#include <random>

namespace Falcor
{
namespace
{
std::vector<std::string> kShaderModels = {
    {"6_2"},
    {"6_3"},
};

const uint32_t kNumElems = 256;
std::mt19937 r;
std::uniform_real_distribution u;

void test(GPUUnitTestContext& ctx, const std::string& shaderModel, bool useUav)
{
    ref<Device> pDevice = ctx.getDevice();

    Program::DefineList defines = {{"USE_UAV", useUav ? "1" : "0"}};

    ctx.createProgram("Tests/Slang/Float64Tests.cs.slang", "testFloat64", defines, Shader::CompilerFlags::None, shaderModel);
    ctx.allocateStructuredBuffer("result", kNumElems);

    std::vector<uint64_t> elems(kNumElems);
    for (auto& v : elems)
        v = fstd::bit_cast<uint64_t>(u(r));
    auto var = ctx.vars().getRootVar();
    auto pBuf = Buffer::createStructured(
        pDevice, var["data"], kNumElems, ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess, Buffer::CpuAccess::None,
        elems.data()
    );
    var["data"] = pBuf;

    ctx.runProgram(kNumElems, 1, 1);

    // Verify results.
    const uint64_t* result = ctx.mapBuffer<const uint64_t>("result");
    for (uint32_t i = 0; i < kNumElems; i++)
    {
        EXPECT_EQ(result[i], elems[i]) << "i = " << i << " shaderModel=" << shaderModel;
    }
    ctx.unmapBuffer("result");
}
} // namespace

GPU_TEST(StructuredBufferLoadFloat64)
{
    for (auto sm : kShaderModels)
        test(ctx, sm, false);
}

GPU_TEST(RWStructuredBufferLoadFloat64)
{
    for (auto sm : kShaderModels)
        test(ctx, sm, true);
}
} // namespace Falcor
