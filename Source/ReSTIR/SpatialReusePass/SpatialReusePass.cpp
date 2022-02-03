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
#include "SpatialReusePass.h"
#include "RenderGraph/RenderPassHelpers.h"

namespace
{
    const char kDesc[] = "Insert pass description here";    
}

// Don't remove this. it's required for hot-reload to function properly
extern "C" __declspec(dllexport) const char* getProjDir()
{
    return PROJECT_DIR;
}

extern "C" __declspec(dllexport) void getPasses(Falcor::RenderPassLibrary& lib)
{
    lib.registerClass("SpatialReusePass", kDesc, SpatialReusePass::create);
}

namespace
{
    const char kShaderFile[] = "RenderPasses/SpatialReusePass/SaptialReusePass.rt.slang";

    // Ray tracing settings that affect the traversal stack size.
    // These should be set as small as possible.
    const uint32_t kMaxPayloadSizeBytes = 80u;
    const uint32_t kMaxAttributeSizeBytes = 8u;
    const uint32_t kMaxRecursionDepth = 2u;

    const char kViewDirInput[] = "viewW";

    const ChannelList kInputChannels =
    {
        { "posW",           "gWorldPosition",             "World-space position (xyz) and foreground flag (w)"       },
        { "normalW",        "gWorldShadingNormal",        "World-space shading normal (xyz)"                         },
        { "tangentW",       "gWorldShadingTangent",       "World-space shading tangent (xyz) and sign (w)", true /* optional */ },
        { "faceNormalW",    "gWorldFaceNormal",           "Face normal in world space (xyz)",                        },
        { kViewDirInput,    "gWorldView",                 "World-space view direction (xyz)", true /* optional */    },
        { "mtlDiffOpacity", "gMaterialDiffuseOpacity",    "Material diffuse color (xyz) and opacity (w)"             },
        { "mtlSpecRough",   "gMaterialSpecularRoughness", "Material specular color (xyz) and roughness (w)"          },
        { "mtlEmissive",    "gMaterialEmissive",          "Material emissive color (xyz)"                            },
        { "mtlParams",      "gMaterialExtraParams",       "Material parameters (IoR, flags etc)"                     },
        { "mvec",           "gMotionVectors",             "Motion vectors in screen space range [0, 1]", false, ResourceFormat::RG32Float},
    };
}

SpatialReusePass::SharedPtr SpatialReusePass::create(RenderContext* pRenderContext, const Dictionary& dict)
{
    SharedPtr pPass = SharedPtr(new SpatialReusePass);
    
    return pPass;
}

SpatialReusePass::SpatialReusePass()
{
    mpSampleGenerator = SampleGenerator::create(SAMPLE_GENERATOR_UNIFORM);
    assert(mpSampleGenerator);
    mpPixelDebug = PixelDebug::create();
    assert(mpPixelDebug);
}

Dictionary SpatialReusePass::getScriptingDictionary()
{
    return Dictionary();
}

RenderPassReflection SpatialReusePass::reflect(const CompileData& compileData)
{
    // Define the required resources here
    RenderPassReflection reflector;
    auto dims = compileData.defaultTexDims;

    addRenderPassInputs(reflector, kInputChannels);
    reflector.addOutput("output", "dummy output");

    return reflector;
}

void SpatialReusePass::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    // use internal dict to pass GBuffer and Reservoir data
    auto& dict = renderData.getDictionary();

    if (!mpScene) return;

    // Request light collection for emissive light
    if (mpScene->getRenderSettings().useEmissiveLights)
    {
        mpScene->getLightCollection(pRenderContext);
    }

    // Configure depth-of-field.
    const bool useDOF = mpScene->getCamera()->getApertureRadius() > 0.f;
    if (useDOF && renderData[kViewDirInput] == nullptr)
    {
        logWarning("Depth-of-field requires the '" + std::string(kViewDirInput) + "' input. Expect incorrect shading.");
    }

    // start debugger
    mpPixelDebug->beginFrame(pRenderContext, renderData.getDefaultTextureDims());

    // Add defines
    mTracer.pProgram->addDefines(getValidResourceDefines(kInputChannels, renderData));

    // perform only once
    if (!mTracer.pVars) prepareVars(renderData); 
    assert(mTracer.pVars);

    // set constant
    auto var = mTracer.pVars->getRootVar();
    var["CB"]["gFrameCount"] = mFrameCount;
    var["CB"]["gPRNGDimension"] = dict.keyExists(kRenderPassPRNGDimension) ? dict[kRenderPassPRNGDimension] : 0u;

    // set structured buffer
    var["gReservoirs"] = mpReservoirBuffer;
    var["gGBuffer"] = mpGBuffer;

    // bind I/O buffers
    auto bind = [&](const ChannelDesc& desc)
    {
        if (!desc.texname.empty())
        {
            var[desc.texname] = renderData[desc.name]->asTexture();
        }
    };
    for (auto channel : kInputChannels) bind(channel);

    const uint2 targetDim = renderData.getDefaultTextureDims();
    assert(targetDim.x > 0 && targetDim.y > 0);

    mpPixelDebug->prepareProgram(mTracer.pProgram, mTracer.pVars->getRootVar());

    // spawn the ray
    mpScene->raytrace(pRenderContext, mTracer.pProgram.get(), mTracer.pVars, uint3(targetDim, 1));

    mpPixelDebug->endFrame(pRenderContext);

    // update internal data
    dict["reservoirs"] = mpReservoirBuffer;
    dict["GBuffer"] = mpGBuffer;
    //dict["debugger"] = mpPixelDebug;

    mFrameCount++;
}

void SpatialReusePass::renderUI(Gui::Widgets& widget)
{
    mpPixelDebug->renderUI(widget);
}

void SpatialReusePass::setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene)
{
    mTracer.pProgram = nullptr;
    mTracer.pBindingTable = nullptr;
    mTracer.pVars = nullptr;

    mpScene = pScene;

    if (mpScene)
    {
        // Create emissive light samlper
        mpEmissiveSampler = EmissiveUniformSampler::create(pRenderContext, pScene);

        // Create ray trace program
        RtProgram::Desc desc;
        desc.addShaderLibrary(kShaderFile);
        desc.setMaxPayloadSize(kMaxPayloadSizeBytes);
        desc.setMaxAttributeSize(kMaxAttributeSizeBytes);
        desc.setMaxTraceRecursionDepth(kMaxRecursionDepth);
        desc.addDefines(mpScene->getSceneDefines());

        mTracer.pBindingTable = RtBindingTable::create(1, 1, mpScene->getGeometryCount());
        auto& bt = mTracer.pBindingTable;
        bt->setRayGen(desc.addRayGen("rayGen"));
        bt->setMiss(0, desc.addMiss("shadowMiss"));
        bt->setHitGroupByType(0, mpScene, Scene::GeometryType::TriangleMesh, desc.addHitGroup("", "shadowAnyHit"));

        mTracer.pProgram = RtProgram::create(desc);
    }
}

void SpatialReusePass::prepareVars(const RenderData& renderData)
{
    // create ray trace variables
    assert(mTracer.pProgram);

    mTracer.pProgram->addDefines(mpSampleGenerator->getDefines());
    mTracer.pProgram->addDefines(mpEmissiveSampler->getDefines());

    mTracer.pVars = RtProgramVars::create(mTracer.pProgram, mTracer.pBindingTable);

    auto var = mTracer.pVars->getRootVar();

    // create structured buffer
    const uint2 targetDim = renderData.getDefaultTextureDims();
    mpReservoirBuffer = Buffer::createStructured(var["gReservoirs"], uint32_t(targetDim.x * targetDim.y));
    mpReservoirBuffer->setName("gReservoirs");
    mpGBuffer = Buffer::createStructured(var["gGBuffer"], uint32_t(targetDim.x * targetDim.y));
    mpGBuffer->setName("gGBuffer");

    // set sample generator
    bool success = mpSampleGenerator->setShaderData(var);
    if (!success) throw std::exception("Failed to bind sample generator");

    success = mpEmissiveSampler->setShaderData(var["CB"]["gEmissiveSampler"]);
    if (!success) throw std::exception("Failed to bind emissive light generator");

}
