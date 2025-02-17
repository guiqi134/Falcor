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

namespace Falcor
{
/** GPU test for reading/writing a 3D texture.
 */
GPU_TEST(RWTexture3D)
{
    ref<Device> pDevice = ctx.getDevice();

    auto pTex = Texture::create3D(
        pDevice, 16, 16, 16, ResourceFormat::R32Uint, 1, nullptr, ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess
    );
    EXPECT(pTex);

    // Step 1: Write to 3D texture bound as UAV.
    ctx.createProgram("Tests/Core/TextureTests.cs.slang", "testTexture3DWrite");
    ctx["tex3D_uav"] = pTex;
    ctx.runProgram(16, 16, 16);

    // Step 2: Read from 3D texture bound as SRV.
    ctx.createProgram("Tests/Core/TextureTests.cs.slang", "testTexture3DRead");
    ctx.allocateStructuredBuffer("result", 4096);
    ctx["tex3D_srv"] = pTex;
    ctx.runProgram(16, 16, 16);

    // Verify result.
    const uint32_t* result = ctx.mapBuffer<const uint32_t>("result");
    size_t i = 0;
    for (uint32_t z = 0; z < 16; z++)
    {
        for (uint32_t y = 0; y < 16; y++)
        {
            for (uint32_t x = 0; x < 16; x++)
            {
                EXPECT_EQ(result[i], x * y * z + 577) << "i = " << i;
                ++i;
            }
        }
    }
    ctx.unmapBuffer("result");
}

/** GPU test for creating a min/max MIP pyramid.
 */
GPU_TEST(TextureMinMaxMip)
{
    ref<Device> pDevice = ctx.getDevice();

    // Generate test texture.
    const uint32_t texWidth = 16;
    const uint32_t texHeight = 16;
    const uint32_t numChannels = 4;
    const uint32_t colorInc = 128 / texWidth;

    std::vector<uint8_t> textureBase(texWidth * texHeight * numChannels);
    std::vector<uint8_t> textureMip(texWidth * texHeight * numChannels / 4);

    for (uint32_t y = 0; y < texHeight; y++)
    {
        for (uint32_t x = 0; x < texWidth; x++)
        {
            // Test values could be improved, simple horizontal gradient here.
            textureBase[(x + y * texWidth) * numChannels + 0] = x * colorInc + y * colorInc;
            textureBase[(x + y * texWidth) * numChannels + 1] = x * colorInc + y * colorInc;
            textureBase[(x + y * texWidth) * numChannels + 2] = x * colorInc + y * colorInc;
            textureBase[(x + y * texWidth) * numChannels + 3] = x * colorInc + y * colorInc;
        }
    }

    // Create texture.
    auto pTex = Texture::create2D(
        pDevice, texWidth, texHeight, ResourceFormat::RGBA8Unorm, 1, Resource::kMaxPossible, textureBase.data(),
        ResourceBindFlags::ShaderResource | ResourceBindFlags::UnorderedAccess
    );
    EXPECT(pTex) << "Texture was not created";

    // Generate new MIPS using MIN/MAX Filtering.
    pTex->generateMips(ctx.getRenderContext(), true);
    uint32_t numMips = pTex->getMipCount();

    // Prepare ping/pong buffers with reference values.
    uint8_t* calculatedValuesPrev = textureBase.data();
    uint8_t* calculatedValues = textureMip.data();

    // Iterate over the MIPs.
    for (uint32_t m = 1; m < numMips; m++)
    {
        uint32_t sri = pTex->getSubresourceIndex(0, m);
        std::vector<uint8_t> levelVals = ctx.getRenderContext()->readTextureSubresource(pTex.get(), sri);

        uint32_t levelWidth = texWidth >> m;
        uint32_t levelHeight = texHeight >> m;

        for (uint32_t y = 0; y < levelHeight; y++)
        {
            for (uint32_t x = 0; x < levelWidth; x++)
            {
                // Calculate new reference values from previous level reference values.
                float expectedAvgF = 0.0f;
                float expectedMinF = INFINITY;
                float expectedMaxF = -INFINITY;
                float expectedAlphaF = 0.0f;
                for (uint32_t dy = 0; dy < 2; dy++)
                {
                    for (uint32_t dx = 0; dx < 2; dx++)
                    {
                        uint32_t texelOffsetPrev = ((x * 2 + dx) + (y * 2 + dy) * levelWidth * 2) * numChannels;

                        expectedAvgF += float(calculatedValuesPrev[texelOffsetPrev + 0]) / 255.0f / 4.0f;
                        expectedMinF = std::min(expectedMinF, float(calculatedValuesPrev[texelOffsetPrev + 1]) / 255.0f);
                        expectedMaxF = std::max(expectedMaxF, float(calculatedValuesPrev[texelOffsetPrev + 2]) / 255.0f);
                        expectedAlphaF += float(calculatedValuesPrev[texelOffsetPrev + 3]) / 255.0f / 4.0f;
                    }
                }
                uint8_t expectedAvg = uint8_t(expectedAvgF * 255.0f);
                uint8_t expectedMin = uint8_t(expectedMinF * 255.0f);
                uint8_t expectedMax = uint8_t(expectedMaxF * 255.0f);
                uint8_t expectedAlpha = uint8_t(expectedAlphaF * 255.0f);

                // Compare with texture values.
                uint32_t texelOffset = (x + y * levelWidth) * numChannels;

                uint8_t valAvg = levelVals[texelOffset + 0];
                uint8_t valMin = levelVals[texelOffset + 1];
                uint8_t valMax = levelVals[texelOffset + 2];
                uint8_t valAlpha = levelVals[texelOffset + 3];

                EXPECT_EQ((uint32_t)valAvg, (uint32_t)expectedAvg) << " AVG - MIP level: " << m << " pixel: [" << x << " - " << y << "]";
                EXPECT_EQ((uint32_t)valMin, (uint32_t)expectedMin) << " MIN - MIP level: " << m << " pixel: [" << x << " - " << y << "]";
                EXPECT_EQ((uint32_t)valMax, (uint32_t)expectedMax) << " MAX - MIP level: " << m << " pixel: [" << x << " - " << y << "]";
                EXPECT_EQ((uint32_t)valAlpha, (uint32_t)expectedAlpha)
                    << " ALPHA - MIP level: " << m << " pixel: [" << x << " - " << y << "]";

                // Save calculated ref values for next iteration.
                calculatedValues[texelOffset + 0] = expectedAvg;
                calculatedValues[texelOffset + 1] = expectedMin;
                calculatedValues[texelOffset + 2] = expectedMax;
                calculatedValues[texelOffset + 3] = expectedAlpha;
            }
        }

        // Swap calculated values buffers.
        uint8_t* ptr = calculatedValuesPrev;
        calculatedValuesPrev = calculatedValues;
        calculatedValues = ptr;
    }
}

// Test loading texture data both in the native format returned by `Bitmap` for 8-bit textures (BGRX8Unorm),
// and explicitly as an 8-bit integer format (RGBA8Uint).
GPU_TEST(Texture_Load8Bit)
{
    ref<Device> pDevice = ctx.getDevice();

    // Create test data.
    uint8_t data[1024];
    for (uint32_t i = 0; i < 256; i++)
    {
        data[4 * i + 0] = (uint8_t)i; // B
        data[4 * i + 1] = (uint8_t)i; // G
        data[4 * i + 2] = (uint8_t)i; // R
        data[4 * i + 3] = 0;          // X
    }

    // Create texture in BGRX8Unorm format.
    // This is what Bitmap::createFromFile currently returns (see BitmapTests.cpp).
    auto texUnorm = Texture::create2D(pDevice, 256, 1, ResourceFormat::BGRX8Unorm, 1, 1, data);
    EXPECT(texUnorm != nullptr);

    // Create texture in RGBA8Uint format.
    auto texUint = Texture::create2D(pDevice, 256, 1, ResourceFormat::RGBA8Uint, 1, 1, data);
    EXPECT(texUint != nullptr);

    ctx.createProgram("Tests/Core/TextureLoadTests.cs.slang", "testLoadFormat");
    ctx.allocateStructuredBuffer("result", 256);
    ctx["texUnorm"] = texUnorm;
    ctx["texUnormAsUint"] = texUnorm;
    ctx["texUint"] = texUint;
    ctx.runProgram(256);

    const uint4* result = ctx.mapBuffer<const uint4>("result");

    for (uint32_t i = 0; i < 256; i++)
    {
        // The texture is 8-bit unorm format so [0,255] is interpreted as floating point values in [0,1].
        // Binding the unorm texture as uint format means we'll see the integer representation of the floating point values in [0,1].
        // Scaling unorm values by 255.0 is expected to get back the original 8-bit values.
        float expectedUnorm = i / 255.0f;
        EXPECT_EQ(asfloat(result[i].x), expectedUnorm);
        EXPECT_EQ(result[i].y, asuint(expectedUnorm));
        EXPECT_EQ(result[i].z, i);
        EXPECT_EQ(result[i].w, i);
    }

    ctx.unmapBuffer("result");
}

GPU_TEST(Texture2D_LoadMips)
{
    ref<Device> pDevice = ctx.getDevice();

    std::filesystem::path paths[] = {
        getRuntimeDirectory() / "data/tests/tiny_mip0.png",
        getRuntimeDirectory() / "data/tests/tiny_mip1.png",
        getRuntimeDirectory() / "data/tests/tiny_mip2.png",
    };

    auto tex = Texture::createMippedFromFiles(pDevice, paths, false);
    ASSERT(tex != nullptr);

    EXPECT_EQ(tex->getMipCount(), 3);

    ctx.createProgram("Tests/Core/TextureLoadTests.cs.slang", "testLoadMips");
    ctx.allocateStructuredBuffer("result", 3);
    ctx["texUnorm"] = tex;
    ctx.runProgram(1, 1, 1);

    const uint4* result = ctx.mapBuffer<const uint4>("result");

    EXPECT_EQ(result[0].x, 255);
    EXPECT_EQ(result[0].y, 0);
    EXPECT_EQ(result[0].z, 0);
    EXPECT_EQ(result[0].w, 255);

    EXPECT_EQ(result[1].x, 0);
    EXPECT_EQ(result[1].y, 255);
    EXPECT_EQ(result[1].z, 0);
    EXPECT_EQ(result[1].w, 255);

    EXPECT_EQ(result[2].x, 0);
    EXPECT_EQ(result[2].y, 0);
    EXPECT_EQ(result[2].z, 255);
    EXPECT_EQ(result[2].w, 255);

    ctx.unmapBuffer("result");
}
} // namespace Falcor
