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
#include "PathTracerReSTIR.h"
#include "RenderGraph/RenderPassHelpers.h"

 // Don't remove this. it's required for hot-reload to function properly
extern "C" __declspec(dllexport) const char* getProjDir()
{
    return PROJECT_DIR;
}

extern "C" __declspec(dllexport) void getPasses(Falcor::RenderPassLibrary & lib)
{
    lib.registerClass("PathTracerReSTIR", "ReSTIR path tracer", PathTracerReSTIR::create);
}

namespace
{
    const char kShaderFile[] = "RenderPasses/PathTracerReSTIR/PathTracerReSTIR.rt.slang";

    // Ray tracing settings that affect the traversal stack size.
    // These should be set as small as possible.
    const uint32_t kMaxPayloadSizeBytes = 80u;
    const uint32_t kMaxAttributeSizeBytes = 8u;
    const uint32_t kMaxRecursionDepth = 2u;

    const ChannelList kOutputChannels =
    {
        { "color",          "gOutputColor",               "Output color (sum of direct and indirect)"                },
    };

    const char kMaxBounces[] = "maxBounces";
    const char kComputeDirect[] = "computeDirect";
    const char kUseReSTIR[] = "useReSTIR";
}

PathTracerReSTIR::SharedPtr PathTracerReSTIR::create(RenderContext* pRenderContext, const Dictionary& dict)
{
    return SharedPtr(new PathTracerReSTIR(dict));
}

PathTracerReSTIR::PathTracerReSTIR(const Dictionary& dict)
{
    parseDictionary(dict);

    // Create a sample generator.
    mpSampleGenerator = SampleGenerator::create(SAMPLE_GENERATOR_UNIFORM);
    assert(mpSampleGenerator);
    mpPixelDebug2 = PixelDebug::create();
    assert(mpPixelDebug2);
}

// The pass is responsible for initializing its members based on key/value pairs found in the Dictionary
void PathTracerReSTIR::parseDictionary(const Dictionary& dict)
{
    for (const auto& [key, value] : dict)
    {
        if (key == kMaxBounces) mMaxBounces = value;
        else if (key == kComputeDirect) mComputeDirect = value;
        else if (key == kUseReSTIR) mUseReSTIR = value;
        else logWarning("Unknown field '" + key + "' in PathTracerReSTIR dictionary");
    }
}

// The render-graph exporter will call the virtual getScriptingDictionary(). Use this function to serialize your pass
Dictionary PathTracerReSTIR::getScriptingDictionary()
{
    Dictionary d;
    d[kMaxBounces] = mMaxBounces;
    d[kComputeDirect] = mComputeDirect;
    d[kUseReSTIR] = mUseReSTIR;
    return d;
}

RenderPassReflection PathTracerReSTIR::reflect(const CompileData& compileData)
{
    RenderPassReflection reflector;

    // Define our input/output channels.
    reflector.addInput("input", "dummy input");
    addRenderPassOutputs(reflector, kOutputChannels);

    return reflector;
}

void PathTracerReSTIR::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    // Update refresh flag if options that affect the output have changed.
    auto& dict = renderData.getDictionary();
    if (mOptionsChanged)
    {
        auto flags = dict.getValue(kRenderPassRefreshFlags, RenderPassRefreshFlags::None);
        dict[Falcor::kRenderPassRefreshFlags] = flags | Falcor::RenderPassRefreshFlags::RenderOptionsChanged;
        mOptionsChanged = false;
    }

    // If we have no scene, just clear the outputs and return.
    if (!mpScene)
    {
        for (auto it : kOutputChannels)
        {
            Texture* pDst = renderData[it.name]->asTexture().get();
            if (pDst) pRenderContext->clearTexture(pDst);
        }
        return;
    }

    if (is_set(mpScene->getUpdates(), Scene::UpdateFlags::GeometryChanged))
    {
        throw std::runtime_error("This render pass does not support scene geometry changes. Aborting.");
    }

    // Request the light collection if emissive lights are enabled.
    if (mpScene->getRenderSettings().useEmissiveLights)
    {
        mpScene->getLightCollection(pRenderContext); // initialize the light collection
    }

    // start debugger
    mpPixelDebug2->beginFrame(pRenderContext, renderData.getDefaultTextureDims());

    // Specialize program.
    // These defines should not modify the program vars. Do not trigger program vars re-creation.
    mTracer.pProgram->addDefine("MAX_BOUNCES", std::to_string(mMaxBounces));
    mTracer.pProgram->addDefine("COMPUTE_DIRECT", mComputeDirect ? "1" : "0");
    mTracer.pProgram->addDefine("USE_RESTIR", mUseReSTIR ? "1" : "0");
    mTracer.pProgram->addDefine("USE_ANALYTIC_LIGHTS", mpScene->useAnalyticLights() ? "1" : "0");
    mTracer.pProgram->addDefine("USE_EMISSIVE_LIGHTS", mpScene->useEmissiveLights() ? "1" : "0");
    mTracer.pProgram->addDefine("USE_ENV_LIGHT", mpScene->useEnvLight() ? "1" : "0");
    mTracer.pProgram->addDefine("USE_ENV_BACKGROUND", mpScene->useEnvBackground() ? "1" : "0");

    // For optional I/O resources, set 'is_valid_<name>' defines to inform the program of which ones it can access.
    // TODO: This should be moved to a more general mechanism using Slang.
    mTracer.pProgram->addDefines(getValidResourceDefines(kOutputChannels, renderData));


    // Prepare program vars. This may trigger shader compilation.
    // The program should have all necessary defines set at this point.
    if (!mTracer.pVars) prepareVars(renderData);
    assert(mTracer.pVars);

    // Set constants.
    auto var = mTracer.pVars->getRootVar();
    var["CB"]["gFrameCount"] = mFrameCount;
    var["CB"]["gPRNGDimension"] = dict.keyExists(kRenderPassPRNGDimension) ? dict[kRenderPassPRNGDimension] : 0u;
    var["CB"]["gTemporalReuse"] = mTemporalReuse;
    var["CB"]["gSpatialReuse"] = mSpatialReuse;
    var["CB"]["gUnbiased"] = mUnbiased;

    // set internal buffer data
    Buffer::SharedPtr reservoirsBuffer = dict["reservoirs"];
    Buffer::SharedPtr GBuffer = dict["GBuffer"];
    var["gReservoirs"] = reservoirsBuffer;
    var["gGBuffer"] = GBuffer;

    // set temporal data buffers
    var["gLastFrameReservoirs"] = mpLastFrameReservoirs;
    var["gLastFrameGBuffer"] = mpLastFrameGBuffer;

    // Bind I/O buffers. These needs to be done per-frame as the buffers may change anytime.
    auto bind = [&](const ChannelDesc& desc) // captures all variables used in the lambda by reference -> "renderData"
    {
        if (!desc.texname.empty())
        {
            var[desc.texname] = renderData[desc.name]->asTexture();
        }
    };
    for (auto channel : kOutputChannels) bind(channel);

    // Get dimensions of ray dispatch.
    const uint2 targetDim = renderData.getDefaultTextureDims();
    assert(targetDim.x > 0 && targetDim.y > 0);

    //PixelDebug::SharedPtr debugger = dict["debugger"];
    //mpPixelDebug = debugger;
    mpPixelDebug2->prepareProgram(mTracer.pProgram, mTracer.pVars->getRootVar());

    // Spawn the rays.
    mpScene->raytrace(pRenderContext, mTracer.pProgram.get(), mTracer.pVars, uint3(targetDim, 1));

    // end debugger
    mpPixelDebug2->endFrame(pRenderContext);

    mFrameCount++;
}

void PathTracerReSTIR::renderUI(Gui::Widgets& widget)
{
    bool dirty = false;

    std::string text1 = std::string("mFrameCount: ") + std::to_string(mFrameCount);
    widget.text(text1);

    dirty |= widget.var("Max bounces", mMaxBounces, 0u, 1u << 16); // return true if the value changed, otherwise false
    widget.tooltip("Maximum path length for indirect illumination.\n0 = direct only\n1 = one indirect bounce etc.", true);

    dirty |= widget.checkbox("Evaluate direct illumination", mComputeDirect);
    widget.tooltip("Compute direct illumination.\nIf disabled only indirect is computed (when max bounces > 0).", true);

    dirty |= widget.checkbox("Use ReSTIR", mUseReSTIR);
    dirty |= widget.checkbox("Temporal Reuse", mTemporalReuse);
    dirty |= widget.checkbox("Spatial Reuse", mSpatialReuse);
    dirty |= widget.checkbox("Use unbiased", mUnbiased);

    mpPixelDebug2->renderUI(widget);

    // If rendering options that modify the output have changed, set flag to indicate that. 
    // In execute() we will pass the flag to other passes for reset of temporal data etc.
    if (dirty)
    {
        mOptionsChanged = true;
    }
}

void PathTracerReSTIR::setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene)
{
    // Clear data for previous scene.
    // After changing scene, the raytracing program should to be recreated.
    mTracer.pProgram = nullptr;
    mTracer.pBindingTable = nullptr;
    mTracer.pVars = nullptr;
    mFrameCount = 0;

    // Set new scene.
    mpScene = pScene;

    if (mpScene)
    {
        if (mpScene->hasGeometryType(Scene::GeometryType::Procedural))
        {
            logWarning("This render pass only supports triangles. Other types of geometry will be ignored.");
        }

        // Create emissive light sampler
        mpEmissiveSampler = EmissiveUniformSampler::create(pRenderContext, pScene);

        // Create ray tracing program.
        RtProgram::Desc desc;
        desc.addShaderLibrary(kShaderFile);
        desc.setMaxPayloadSize(kMaxPayloadSizeBytes);
        desc.setMaxAttributeSize(kMaxAttributeSizeBytes);
        desc.setMaxTraceRecursionDepth(kMaxRecursionDepth);
        desc.addDefines(mpScene->getSceneDefines());

        mTracer.pBindingTable = RtBindingTable::create(2, 2, mpScene->getGeometryCount());
        auto& sbt = mTracer.pBindingTable;
        sbt->setRayGen(desc.addRayGen("rayGen"));
        sbt->setMiss(0, desc.addMiss("scatterMiss"));
        sbt->setMiss(1, desc.addMiss("shadowMiss"));
        sbt->setHitGroupByType(0, mpScene, Scene::GeometryType::TriangleMesh, desc.addHitGroup("scatterClosestHit", "scatterAnyHit"));
        sbt->setHitGroupByType(1, mpScene, Scene::GeometryType::TriangleMesh, desc.addHitGroup("", "shadowAnyHit"));

        mTracer.pProgram = RtProgram::create(desc);
    }
}

bool PathTracerReSTIR::onMouseEvent(const MouseEvent& mouseEvent)
{
    return mpPixelDebug2->onMouseEvent(mouseEvent);
}

void PathTracerReSTIR::prepareVars(const RenderData& renderData)
{
    assert(mTracer.pProgram);

    // Configure program.
    mTracer.pProgram->addDefines(mpSampleGenerator->getDefines());
    mTracer.pProgram->addDefines(mpEmissiveSampler->getDefines());

    // Create program variables for the current program.
    // This may trigger shader compilation. If it fails, throw an exception to abort rendering.
    mTracer.pVars = RtProgramVars::create(mTracer.pProgram, mTracer.pBindingTable);
    auto var = mTracer.pVars->getRootVar();

    // Set sample generator
    bool success = mpSampleGenerator->setShaderData(var);
    if (!success) throw std::exception("Failed to bind sample generator");

    success = mpEmissiveSampler->setShaderData(var);
    if (!success) throw std::exception("Failed to bind emissive light generator");

    // create temporal data buffers
    const uint2 targetDim = renderData.getDefaultTextureDims();
    const uint elementCount = targetDim.x * targetDim.y;
    mpLastFrameReservoirs = Buffer::createStructured(var["gLastFrameReservoirs"], elementCount);
    mpLastFrameReservoirs->setName("lastFrameReservoirs");
    mpLastFrameGBuffer = Buffer::createStructured(var["gLastFrameGBuffer"], elementCount);
    mpLastFrameGBuffer->setName("lastFrameGBuffer");
}
