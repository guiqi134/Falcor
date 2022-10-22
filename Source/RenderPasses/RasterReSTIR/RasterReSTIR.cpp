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
#include "RasterReSTIR.h"

namespace
{
    // In our tutorials, we use Falcor buffers as read-write objects
    const Resource::BindFlags kBufferBindFlags =
        Resource::BindFlags::ShaderResource |        // Allows reading a buffer via a Buffer<> object in HLSL
        Resource::BindFlags::UnorderedAccess;        // Allows reading/writing a buffer via a RWBuffer<> object in HLSL

    // In our tutorials, we use Falcor's automatic mipmap generation, which requires the following access types
    const Resource::BindFlags kAutoMipBindFlags =
        Resource::BindFlags::ShaderResource |        // Allows using the mipmapped texture as a Texture2D in HLSL
        Resource::BindFlags::UnorderedAccess |       // Allows writing the base mip layer as a RWTexture2D in HLSL
        Resource::BindFlags::RenderTarget;           // Needed for the automatic mipmap generation

    const Resource::BindFlags kShadowMapFlags = Resource::BindFlags::ShaderResource | Resource::BindFlags::DepthStencil;

    const std::string kShaderDirectory = "RenderPasses/RasterReSTIR/";
}

const RenderPass::Info RasterReSTIR::kInfo { "RasterReSTIR", "Insert pass description here." };

// Don't remove this. it's required for hot-reload to function properly
extern "C" FALCOR_API_EXPORT const char* getProjDir()
{
    return PROJECT_DIR;
}

extern "C" FALCOR_API_EXPORT void getPasses(Falcor::RenderPassLibrary& lib)
{
    lib.registerPass(RasterReSTIR::kInfo, RasterReSTIR::create);
}

RasterReSTIR::SharedPtr RasterReSTIR::create(RenderContext* pRenderContext, const Dictionary& dict)
{
    SharedPtr pPass = SharedPtr(new RasterReSTIR(dict));
    return pPass;
}

Dictionary RasterReSTIR::getScriptingDictionary()
{
    return Dictionary();
}

RenderPassReflection RasterReSTIR::reflect(const CompileData& compileData)
{
    RenderPassReflection reflector;

    // What buffers does this render pass expect as inputs from prior render passes?
    //reflector.addInput("vbuffer", "");    // We expect an input V-Buffer, allowing look ups of surface data at primary hits
    //reflector.addInput("mvec", "");       // The V-buffer pass should also give us a texture with motion vectors

    // This pass outputs a shaded color for display or for other passes to run postprocessing
    reflector.addOutput("color", "").format(ResourceFormat::RGBA16Float);

    return reflector;
}

void RasterReSTIR::compile(RenderContext* pRenderContext, const CompileData& compileData)
{
    // If our screen resolution has changed, we need to remake our RTXDI context
    if (screenSize != compileData.defaultTexDims)
    {
        screenSize = compileData.defaultTexDims;
    }
}

void RasterReSTIR::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    if (!mpScene) return;

    mpPixelDebug->beginFrame(pRenderContext, screenSize);

    // Initialize buffer with random numbers for testing
    {
        auto vars = mpInitializeBuffer->getRootVar();
        vars["CB"]["gScreenSize"] = screenSize;
        vars["CB"]["gFrameIndex"] = mFrameCount;
        vars["CB"]["gTotalLightMeshCount"] = totalLightMeshCount;
        vars["gInputBuffer"] = mpPrevLightMeshSelectionBuffer;
        //mpPixelDebug->prepareProgram(mpInitializeBuffer->getProgram(), vars);
        mpInitializeBuffer->execute(pRenderContext, screenSize.x * screenSize.y, 1);
    }

    // Because we are directly adding to the histogram buffer, so in each frame we need to clear this buffer instead of accumulating the result
    pRenderContext->clearUAV(mpLightMeshHistogramBuffer->getUAV().get(), uint4(0));

    // Counting pass
    {
        FALCOR_PROFILE("Counting Lights");

        auto histogramVars = mpComputeLightMeshHistogram->getRootVar();
        histogramVars["CB"]["gTotalLightMeshCount"] = totalLightMeshCount;
        histogramVars["gPrevLightMeshSelectionBuffer"] = mpPrevLightMeshSelectionBuffer; // input
        histogramVars["gFinalHistogramBuffer"] = mpLightMeshHistogramBuffer; // output
        //histogramVars["gSortBuffer"] = mpSortedLightMeshBuffer;
        //mpPixelDebug->prepareProgram(mpComputeLightMeshHistogram->getProgram(), histogramVars);
        mpComputeLightMeshHistogram->execute(pRenderContext, screenSize.x * screenSize.y, 1);
    }

    //if (false)
    {
        FALCOR_PROFILE("Sorting Lights");

        // Get the max iteration for inner and outer sort pass. This excludes the pre-sort pass.
        const uint alignedTotalLightCount = (uint)pow(2, (uint)log2(totalLightMeshCount) + 1);
        const uint maxIterations = (uint)log2(std::max(2048u, alignedTotalLightCount)) - 10u;
        const uint nThreads = alignedTotalLightCount / 2;

        //logInfo(std::format("alignedTotalLightCount = {}", alignedTotalLightCount));
        //logInfo(std::format("maxIterations = ", maxIterations));

        // Pre-Sort the buffer up to k = 2048.  This also pads the list with invalid indices
        // that will drift to the end of the sorted list.
        auto preSortVars = mBitonicSort.preSort->getRootVar();
        preSortVars["sortCB"]["gTotalLightMeshCount"] = totalLightMeshCount;
        preSortVars["sortCB"]["gNullItem"] = uint(0);
        preSortVars["gSortBuffer"] = mpSortedLightMeshBuffer;
        //mpPixelDebug->prepareProgram(mBitonicSort.preSort->getProgram(), preSortVars);
        mBitonicSort.preSort->execute(pRenderContext, nThreads, 1);

        // We have already pre-sorted up through k = 2048 when first writing our list, so
        // we continue sorting with k = 4096.  For unnecessarily large values of k, these
        // indirect dispatches will be skipped over with thread counts of 0.
        for (uint k = 4096; k <= alignedTotalLightCount; k <<= 1)
        {
            // Outer sort on j >= 2048
            auto outerSortVars = mBitonicSort.outerSort->getRootVar();
            outerSortVars["sortCB"]["gTotalLightMeshCount"] = totalLightMeshCount;
            outerSortVars["outerCB"]["k"] = k;
            for (uint j = k / 2; j >= 2048; j >>= 1)
            {
                outerSortVars["outerCB"]["j"] = j;
                outerSortVars["gSortBuffer"] = mpSortedLightMeshBuffer;

                mBitonicSort.outerSort->execute(pRenderContext, nThreads, 1);
            }

            // Inner sort on j <= 1024
            auto innerSortVars = mBitonicSort.innerSort->getRootVar();
            innerSortVars["sortCB"]["gTotalLightMeshCount"] = totalLightMeshCount;
            innerSortVars["sortCB"]["gNullItem"] = uint(0);
            innerSortVars["gSortBuffer"] = mpSortedLightMeshBuffer;
            mBitonicSort.innerSort->execute(pRenderContext, nThreads, 1);
        }
    }

    // Check GPU reuslt with CPU
    if (mFrameCount == 50)
    {
        // Must use a staging buffer which has no bind flags to map data
        auto pStagingBuffer1 = Buffer::createTyped<uint>(totalLightMeshCount, ResourceBindFlags::None, Buffer::CpuAccess::Read);
        auto pStagingBuffer2 = Buffer::createTyped<uint2>(totalLightMeshCount, ResourceBindFlags::None, Buffer::CpuAccess::Read);
        pRenderContext->copyBufferRegion(pStagingBuffer1.get(), 0, mpLightMeshHistogramBuffer.get(), 0, sizeof(uint) * totalLightMeshCount);
        pRenderContext->copyBufferRegion(pStagingBuffer2.get(), 0, mpSortedLightMeshBuffer.get(), 0, sizeof(uint2) * totalLightMeshCount);

        // Flush GPU and wait for results to be available.
        pRenderContext->flush(false);
        mpFence->gpuSignal(pRenderContext->getLowLevelData()->getCommandQueue());
        mpFence->syncCpu();

        // Read back GPU results
        uint* pData1 = reinterpret_cast<uint*>(pStagingBuffer1->map(Buffer::MapType::Read));
        std::vector<uint> deviceResult1(pData1, pData1 + totalLightMeshCount); // invoke range constructor (linear time)
        pStagingBuffer1->unmap();

        uint2* pData2 = reinterpret_cast<uint2*>(pStagingBuffer2->map(Buffer::MapType::Read));
        std::vector<uint2> deviceResult2(pData2, pData2 + totalLightMeshCount); // invoke range constructor (linear time)
        pStagingBuffer2->unmap();
        
        // Printing to log
        uint sum = 0;
        for (uint i = 0; i < deviceResult1.size(); i++)
        {
            sum += deviceResult1[i];
            logInfo(std::format("mpLightMeshHistogramBuffer[{}] = {}", i, deviceResult1[i]));
        }

        logInfo(std::format("sum = {}, pixels = {}", sum, screenSize.x * screenSize.y));

        sum = 0;
        for (uint i = 0; i < deviceResult2.size(); i++)
        {
            sum += deviceResult2[i].y;
            logInfo(std::format("Light Mesh ID {} = {}", deviceResult2[i].x, deviceResult2[i].y));
        }

        logInfo(std::format("sum = {}, pixels = {}", sum, screenSize.x* screenSize.y));

    }

    mFrameCount++;

    mpPixelDebug->endFrame(pRenderContext);
}

void RasterReSTIR::renderUI(Gui::Widgets& widget)
{
    mpPixelDebug->renderUI(widget);
}

bool RasterReSTIR::onKeyEvent(const KeyboardEvent& keyEvent)
{
    if (keyEvent.type == KeyboardEvent::Type::KeyPressed && keyEvent.key == Input::Key::Key1)
    {
        mFrameCount = 0;
        return true;
    }

    return false;
}

void RasterReSTIR::setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene)
{
    mpScene = pScene;
    mpFence = GpuFence::create();
    mpPixelDebug = PixelDebug::create();
    mpLights = mpScene->getLightCollection(pRenderContext);
    mpSampleGenerator = SampleGenerator::create(SAMPLE_GENERATOR_UNIFORM);
    screenSize = uint2(gpFramework->getTargetFbo()->getWidth(), gpFramework->getTargetFbo()->getHeight());

    logInfo("light mesh count = " + std::to_string((uint)mpLights->getMeshLights().size()));
    logInfo("screenSize = " + to_string(screenSize));

    // Create resources
    mpPrevLightMeshSelectionBuffer = Buffer::createTyped<int>(screenSize.x * screenSize.y, kBufferBindFlags);
    mpLightMeshHistogramBuffer = Buffer::createTyped<uint>(totalLightMeshCount, kBufferBindFlags, Buffer::CpuAccess::Read);
    mpSortedLightMeshBuffer = Buffer::createTyped<uint2>(totalLightMeshCount, kBufferBindFlags, Buffer::CpuAccess::Read);

    if (mpScene->useEmissiveLights())
    {
        mpEmissiveSampler = EmissivePowerSampler::create(pRenderContext, mpScene);
    }

    auto emissiveDefines = mpEmissiveSampler->getDefines();

    // Load shaders
    Program::DefineList defines;
    defines.add(mpSampleGenerator->getDefines());
    mpInitializeBuffer = createComputeShader("ComputeTopNLights.cs.slang", defines, "initializeBuffer");
    mpComputeLightMeshHistogram = createComputeShader("ComputeTopNLights.cs.slang", defines, "computeHistogramTwoAdd");
    defines.clear();

    mBitonicSort.preSort = createComputeShader("Sorting.cs.slang", defines, "preSortCS");
    mBitonicSort.innerSort = createComputeShader("Sorting.cs.slang", defines, "innerSortCS");
    mBitonicSort.outerSort = createComputeShader("Sorting.cs.slang", defines, "outerSortCS");

    mpBuildKeyValuePairs = createComputeShader("BuildKeyValuePairs.cs.slang", defines, "buildKeyValuePairs");
}

RasterReSTIR::RasterReSTIR(const Dictionary& dict)
    : RenderPass(kInfo)
{
    parseDictionary(dict);
}

void RasterReSTIR::parseDictionary(const Dictionary& dict)
{
    for (const auto& [key, value] : dict)
    {
        //if (key == kOptions) mOptions = value;
        //else logWarning("Unknown field '{}' in RTXDIPass dictionary.", key);
    }
}

ComputePass::SharedPtr RasterReSTIR::createComputeShader(const std::string& file, const Program::DefineList& defines, const std::string& entryPoint)
{
    // Where is the shader?  What shader model to use?
    Program::Desc risDesc(kShaderDirectory + file);
    risDesc.setShaderModel("6_5");

    // Create a Falcor shader wrapper, with specified entry point and scene-specific #defines (for common Falcor HLSL routines)
    auto sceneDefines = mpScene->getSceneDefines();
    sceneDefines.add(defines);
    ComputePass::SharedPtr pShader = ComputePass::create(risDesc.csEntry(entryPoint), sceneDefines);

    // Type conformances are needed for specific Slang language constructs used in common Falcor shaders, no need to worry
    // about this unless you're using advanced Slang functionality in a very specific way.
    pShader->getProgram()->setTypeConformances(mpScene->getTypeConformances());

    pShader->getRootVar()["gScene"] = mpScene->getParameterBlock();

    // Zero out structures to make sure they're regenerated with correct settings (conformances, scene defines, etc)
    pShader->setVars(nullptr);

    return pShader;
}
