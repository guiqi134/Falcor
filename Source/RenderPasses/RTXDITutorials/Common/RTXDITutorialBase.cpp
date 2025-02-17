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
#include "RTXDITutorialBase.h"
#include "RTXDITutorialConstants.h"

 /** This RenderPass has common resources and methods needed by all our tutorials.

     Please do not start reading this code!

     Read the tutorial passes, and refer to specific methods below as needed by the
     tutorial steps.  This will help clarify that you don't need to start an
     integration with a 500+ line complete base class (like this), but can start
     with a (relatively) smaller set of methods.
 */

using namespace Falcor;

namespace
{
    // Some shortcuts for common resource access patterns, used when initializing Falcor buffers and textures.

    // In our tutorials, we use Falcor buffers as read-write objects
    const Resource::BindFlags kBufferBindFlags =
        Resource::BindFlags::ShaderResource |        // Allows reading a buffer via a Buffer<> object in HLSL
        Resource::BindFlags::UnorderedAccess;        // Allows reading/writing a buffer via a RWBuffer<> object in HLSL

    // In our tutorials, we use Falcor's automatic mipmap generation, which requires the following access types
    const Resource::BindFlags kAutoMipBindFlags =
        Resource::BindFlags::ShaderResource |        // Allows using the mipmapped texture as a Texture2D in HLSL
        Resource::BindFlags::UnorderedAccess |       // Allows writing the base mip layer as a RWTexture2D in HLSL
        Resource::BindFlags::RenderTarget;           // Needed for the automatic mipmap generation
};


/** Our render pass constructor checks if the Python script which specifies a render graph has provided
    and specific settings we should respect.  The keys below are optional parameters the tutorial passes
    respond to from these Python scripts.
*/
RTXDITutorialBase::RTXDITutorialBase(const Dictionary& dict, const RenderPass::Info& desc)
    : RenderPass(desc)
{
    for (const auto& [key, value] : dict)
    {
        if (key == "useLowerShininess")        mLightingParams.useHigherShininess = !bool(value);
        else if (key == "triEmissiveScale")    mLightingParams.relativeTriangleWeight = float(value);
        else if (key == "envEmissiveScale")    mLightingParams.relativeEnvMapWeight = float(value);
        else if (key == "epsilon")             mLightingParams.epsilon = float(value);
        else if (key == "compressLightTiles")  mLightingParams.storePerTileLightGeom = bool(value);
        else logWarning("Unknown field '" + key + "' in RTXDITutorialBase dictionary");
    }

    mpPixelDebug = PixelDebug::create();
}


/** Falcor render passes are just one part of a renderer.  Our RTXDI tutorials assume they get connected up
    to a V-buffer (see https://jcgt.org/published/0002/02/04/).  Falcor provides a standard V-buffer pass
    in both ray traced and raster variance (these are located in the GBuffer render pass, which gets compiled
    into GBuffer.dll).  Our tutorials use this standard pass.

    When hooking passes together, we need to tell Mogwai (which executes sequences of render graphs) what
    resources we expect from prior passes and what output resources we provide for later passes to consume.
    This specification happens in the reflect() callback, which is used to validate that all expected resources
    have been provided for correct execution.
*/
RenderPassReflection RTXDITutorialBase::reflect(const CompileData& compileData)
{
    RenderPassReflection reflector;

    // What buffers does this render pass expect as inputs from prior render passes?
    reflector.addInput("vbuffer", "");    // We expect an input V-Buffer, allowing look ups of surface data at primary hits
    reflector.addInput("mvec", "");       // The V-buffer pass should also give us a texture with motion vectors

    // This pass outputs a shaded color for display or for other passes to run postprocessing
    reflector.addOutput("color", "").format(ResourceFormat::RGBA16Float);
    reflector.addOutput("debug", "").format(ResourceFormat::RGBA16Float);
    reflector.addOutput("hasSampleChangedInReusing", "").format(ResourceFormat::RGBA16Float);
    reflector.addOutput("reservoirM", "").format(ResourceFormat::RGBA16Float);

    return reflector;
}


/** Falcor "recompiles" the render graph whenever the window size changes.  Recompilation happens other times,
    but we're using this callback solely to capture window resize events, after which we need to reinitialize 
    the RTXDI context.
*/
void RTXDITutorialBase::compile(RenderContext* pContext, const CompileData& compileData)
{
    // If our screen resolution has changed, we need to remake our RTXDI context
    if (mPassData.screenSize != compileData.defaultTexDims)
    {
        mpRtxdiContext = nullptr;
        mPassData.screenSize = compileData.defaultTexDims;
    }
}


/** Updates flags inside this base class to (selectively/lazily) rebuild structures due to dynamic scene changes
*/
void RTXDITutorialBase::checkForSceneUpdates()
{
    // TODO: Make more granular / flexible; need to update Falcor UpdateFlags to allow more granular queries.
    //   For instance Scene::UpdateFlags::MaterialsChanged checks if any scene material has changed in any way,
    //   but for RTXDI, we really only care if a material's *emissivity* has changed.

    // Determine what, if anything happened since last frame.  
    Scene::UpdateFlags updates = mPassData.scene->getUpdates();
    if (bool(updates & Scene::UpdateFlags::LightCollectionChanged))  mPassData.updateEmissiveTriangleGeom = true;
    if (bool(updates & Scene::UpdateFlags::MaterialsChanged))        mPassData.updateEmissiveTriangleFlux = true;
    if (bool(updates & Scene::UpdateFlags::EnvMapChanged))           mPassData.updateEnvironmentMapPdf = true;
}


/** When Falcor/Mogwai load a scene, this callback gets called.  We stash the new scene and set flags to
    rebuild the RTXDI context and all the internal light structures.
*/
void RTXDITutorialBase::setScene(RenderContext* pContext, const Scene::SharedPtr& pScene)
{
    // Callback for when Mogwai detects we have a new scene.  Stash it.
    mPassData.scene = pScene;

    // Force RTXDI to recreate its context, just to make sure we size resources correctly.
    mpRtxdiContext = nullptr;

    // If we successfully loaded a scene, tell Falcor to build a collection of emissives for the scene.
    if (mPassData.scene)
    {
        mPassData.lights = mPassData.scene->getLightCollection(pContext);
        mPassData.updateEnvironmentMapPdf = true;
        mPassData.updateEmissiveTriangleFlux = true;
        mPassData.updateEmissiveTriangleGeom = true;
    }
}

/** When RTXDI reuses spatial neighbors, it is _not_ a dense reuse (e.g., of all pixels in a 5x5 regions).

    Reuse is instead stochastic, it reuses some random subset of pixels within a specified radius of the
    current pixel.

    But for various reasons, you don't want _pure_ random reuse with white noise samples.  RTXDI provides
    a function that fills a buffer with reasonably distributed samples (in roughly a poisson / blue-noise
    density).  You could, instead, fill this buffer with your own distribution.  This buffer is passed
    to the RTXDI bridge, which draws samples from this during spatial reuse.

    When starting an integration, start with this distribution.  It's likely you'll never change it.
*/
bool RTXDITutorialBase::updateNeighborBuffer()
{
    // Don't reallocate this; it's really a one-size-fits-all setting and buffer.
    if (!mResources.neighborOffsetBuffer)
    {
        // Create a CPU array of offsets; ask RTXDI to fill it with the neighbor offsets it needs
        std::vector<uint8_t> offsets(2 * size_t(mRtxdiContextParams.NeighborOffsetCount));
        mpRtxdiContext->FillNeighborOffsetBuffer(offsets.data());

        // Create a GPU buffer, initializing it with the CPU array we just created
        mResources.neighborOffsetBuffer = Buffer::createTyped(ResourceFormat::RG8Snorm, mRtxdiContextParams.NeighborOffsetCount,
            kBufferBindFlags, Buffer::CpuAccess::None, static_cast<void*>(offsets.data()));
    }
    return (mResources.neighborOffsetBuffer != nullptr);
}

/** This executes a shader that converts the input V-buffer (see the comment on reflect() method)
    into a packed G-buffer format that is efficient to use during RTXDI execution.  This is important,
    since access to your engine's material properties at surface hit points may not be particularly
    efficient (or, more importantly, memory coherent).  During reuse in RTXDI, we re-evaluate shading
    at hits _many_ times per pixel, so we need loading material properties to be memory coherent.

    This is common code called by all our tutorials.  The contents of the executed shader are not
    particularly important, as long as you can replicate a (similar) output from your engine.
*/
void RTXDITutorialBase::prepareSurfaceData(RenderContext* pContext, const RenderData& data)
{
    // This kernel repacks the v-buffer data and stores it in mResources.gBufferData.  This occurs in the
    // RTXDI_PrepareSurfaceData.cs.slang kernel.  The packed format is 128 bits (one float4), with
    // packed surface normal (32-bits), packed diffuse reflectance (24-bits), relative weight of the diffuse
    // lobe (8-bits), packed specular reflectance (24-bits), surface roughness (8-bits), and distance from
    // camera to the hitpoint (32-bits).  The exact terms needed may depend on the material model you use
    // during resampling in the RTXDI bridge.  Our format is a decent place to start.

    FALCOR_PROFILE("Format Surface Hit Data");
    auto prepSurfaceVars = mShader.prepareSurfaceData->getRootVar();
    prepSurfaceVars["CB"]["gScreenRes"] = mPassData.screenSize;
    prepSurfaceVars["CB"]["gEnvironmentLightPresent"] = bool(mPassData.scene->getEnvMap() != nullptr);
    prepSurfaceVars["CB"]["gRelativeTrianglePower"] = float(mLightingParams.relativeTriangleWeight);
    prepSurfaceVars["CB"]["gRelativeEnvironmentPower"] = float(mLightingParams.relativeEnvMapWeight);
    prepSurfaceVars["CB"]["gUsePrimaryHitTextureLoD"] = mLightingParams.usePrimaryLoD;
    prepSurfaceVars["CB"]["gGBufferOffset"] = uint32_t(mLightingParams.currentGBufferIndex * mPassData.screenSize.x * mPassData.screenSize.y);
    prepSurfaceVars["gVbuffer"] = data["vbuffer"]->asTexture();        // Our input V-buffer
    prepSurfaceVars["gOutputGeom"] = mResources.gBufferData;           // Our output packed surface data (G-buffer)
    prepSurfaceVars["gOutputEmissives"] = mResources.emissiveColors;   // Full-screen image with emission from directly-visible geometry (and env map)
    mShader.prepareSurfaceData->execute(pContext, mPassData.screenSize.x, mPassData.screenSize.y);
}

void RTXDITutorialBase::computePDFTextures(RenderContext* pContext, const RenderData& data)
{
    FALCOR_PROFILE("Create PDF Texure");

    /* Create the PDF texture for primitive lights.  Right now, this is only triangles,
       but it could include other primitives (spheres, points, rects, cylinders, etc.)
    */
    {
        // This shader executes one thread per primitive light.  This means the kernel launch 
        // doesn't have a natural width & height.  We split it into a 2D kernel launch, as 1D launches 
        // have a maximum size (i.e., max # of lights).  Our 2D "width" is a arbitrary size of 8192, 
        // with the height dependent on the number of lights in the scene.  Optimal sizing has not
        // been explored.
        uint launchWidth = 8192u;
        uint launchHeight = (mPassData.lights->getActiveLightCount() / launchWidth) + 1;

        // This shader also (optionally) repacks Falcor lights into a more cache coherent format, 
        // in addition to creating the pdf texture for use in initial candidate sampling.
        auto pdfVars = mShader.createLightResources->getRootVar();
        pdfVars["CB"]["gLaunchWidth"] = launchWidth;
        pdfVars["CB"]["gNumPrimitives"] = mPassData.lights->getActiveLightCount();
        pdfVars["CB"]["gUpdatePrimitiveGeometry"] = mPassData.updateEmissiveTriangleGeom;  // Did our lights move?
        pdfVars["CB"]["gUpdatePrimitiveFlux"] = mPassData.updateEmissiveTriangleFlux;      // Did our lights change color?
        pdfVars["gPackedLightGeom"] = mResources.lightGeometry;             // Output re-packed light geometry
        pdfVars["gPrimitivePDFTexture"] = mResources.localLightPdfTexture;  // Output PDF texture for sampling
        mShader.createLightResources->execute(pContext, launchWidth, launchHeight);
        mPassData.updateEmissiveTriangleGeom = false;

        // The above kernel creates the base level of the light pdf texture; now, create a mipmap chain 
        if (mPassData.updateEmissiveTriangleFlux)
        {
            mResources.localLightPdfTexture->generateMips(pContext);
            mPassData.updateEmissiveTriangleFlux = false;
        }
    }

    // If we have an environment map, create a pdf texture to sample that, too.
    if (mPassData.scene->getEnvMap() && mPassData.updateEnvironmentMapPdf)
    {
        // Yeah, I know.  Stash the environment map texture size for easier use.
        uint2 envMapSize = uint2(mPassData.scene->getEnvMap()->getEnvMap()->getWidth(),
            mPassData.scene->getEnvMap()->getEnvMap()->getHeight());

        auto pdfVars = mShader.createEnvironmentPDFTexture->getRootVar();
        pdfVars["gEnvMapPDFTexture"] = mResources.environmentPdfTexture;   // Output PDF texture for sampling
        mShader.createEnvironmentPDFTexture->execute(pContext, envMapSize.x, envMapSize.y);

        // Create a mipmap chain from this light pdf texure
        mResources.environmentPdfTexture->generateMips(pContext);
        mPassData.updateEnvironmentMapPdf = false;
    }
}

void RTXDITutorialBase::presampleLights(RenderContext* pContext, const RenderData& data)
{
    /* Do presampling of lights into tiles.  The two shaders executed here are *exceptionally* simple,
       as they just call the approrpriate RTXDI API (i.e., RTXDI_PresampleLocalLights() and
       RTXDI_PresampleEnvironmentMap())
    */

    FALCOR_PROFILE("Presample Lights");

    // For our primitive / triangle lights 
    {
        auto presampleVars = mShader.presamplePrimitiveLights->getRootVar();
        setupRTXDIBridgeVars(presampleVars, data);
        mShader.presamplePrimitiveLights->execute(pContext,
            mRtxdiContextParams.TileSize, mRtxdiContextParams.TileCount);
    }

    // For our environment lights 
    if (mRtxdiFrameParams.environmentLightPresent)
    {
        auto presampleVars = mShader.presampleEnvironmentLights->getRootVar();
        setupRTXDIBridgeVars(presampleVars, data);
        mShader.presampleEnvironmentLights->execute(pContext,
            mRtxdiContextParams.EnvironmentTileSize, mRtxdiContextParams.EnvironmentTileCount);
    }
}

void RTXDITutorialBase::setCurrentFrameRTXDIParameters(const RenderData& renderData)
{
    /* At the start of each frame, we need to update RTXDI parameters, so it knows the situation
       in the current frame.  This checks for changes, and calls the RTXDI C++ API to fill our
       shader parameter buffer that we give to each of the shaders using RTXDI.
    */

    // Do we use light tile presampling?  (Generally, you should.)
    mRtxdiFrameParams.enableLocalLightImportanceSampling = true;

    // Specify how many emissive triangles we have (for RTXDI, these can be other types of primitives
    // but our Falcor scenes currently only have triangle lights)
    mRtxdiFrameParams.firstLocalLight = 0;
    mRtxdiFrameParams.numLocalLights = mPassData.lights->getActiveLightCount();

    // Are we using an environment map?  If so, store it in the RTXDI light list right
    // after the last triangle light (i.e., envMapIndex == mPassData.lights->getTotalLightCount())
    mRtxdiFrameParams.environmentLightPresent = (mPassData.scene->getEnvMap() != nullptr);
    mRtxdiFrameParams.environmentLightIndex = mPassData.lights->getTotalLightCount();  

    // We currently do not handle "infinite" lights (aka traditional GL/DX directional lights)
    mRtxdiFrameParams.numInfiniteLights = 0;

    // In case we're using ReGIR (currently not done here), update the grid center to be at the camera.
    auto cameraPos = mPassData.scene->getCamera()->getPosition();
    mRtxdiFrameParams.regirCenter = rtxdi::float3{ cameraPos.x, cameraPos.y, cameraPos.z };

    // Update the parameters RTXDI needs when we call its functions in our shaders.
    mpRtxdiContext->FillRuntimeParameters(mRtxdiShaderParams, mRtxdiFrameParams);
}

void RTXDITutorialBase::setupRTXDIBridgeVars(ShaderVar& vars, const RenderData& renderData)
{
    /* Our Falcor - specific RTXDI application bridge has a number of shader parameters & resources
       that we need to set for the bridge to function.  This routine consistently sets them.  Any
       shaders using the bridge needs to call setupRTXDIBridgeVars() prior to launching the shader.
    */

    // Update our shared RTXDI bridge parameter block for the current frame
    vars["BridgeCB"]["gEpsilon"] = float(mLightingParams.epsilon);
    vars["BridgeCB"]["gFrameIndex"] = uint32_t(mRtxdiFrameParams.frameIndex);
    vars["BridgeCB"]["gFrameSize"] = uint2(mpRtxdiContext->GetParameters().RenderWidth, mpRtxdiContext->GetParameters().RenderHeight);
    vars["BridgeCB"]["gEnvMapRes"] = !mResources.environmentPdfTexture
        ? uint2(0u) : uint2(mResources.environmentPdfTexture->getWidth(), mResources.environmentPdfTexture->getHeight());
    vars["BridgeCB"]["gUseLowerShininess"] = !mLightingParams.useHigherShininess;  // Allows matching early prototypes with different matl models
    vars["BridgeCB"]["gRtxdiParams"].setBlob(&mRtxdiShaderParams, sizeof(mRtxdiShaderParams));
    vars["BridgeCB"]["gRelativeTrianglePower"] = float(mLightingParams.relativeTriangleWeight);
    vars["BridgeCB"]["gRelativeEnvironmentPower"] = float(mLightingParams.relativeEnvMapWeight);
    vars["BridgeCB"]["gUseEmissiveTextures"] = mLightingParams.useTextureForShade;
    vars["BridgeCB"]["gUseRTXDIInitialSampling"] = (mLightingParams.selectedDisplayPipeline >= 2u);  // True if not using a "baseline" pipeline
    vars["BridgeCB"]["gStoreCompactedLightInfo"] = mLightingParams.storePerTileLightGeom;
    vars["BridgeCB"]["gCurrentGBufferOffset"] = uint32_t(mLightingParams.currentGBufferIndex * mPassData.screenSize.x * mPassData.screenSize.y);
    vars["BridgeCB"]["gPriorGBufferOffset"] = uint32_t(mLightingParams.priorGBufferIndex * mPassData.screenSize.x * mPassData.screenSize.y);

    // Setup textures and other buffers needed by the RTXDI bridge
    vars["gLightGeometry"] = mResources.lightGeometry;
    vars["gGBuf"] = mResources.gBufferData;
    vars["gRisBuffer"] = mResources.lightTileBuffer;
    vars["gPackedPresampledLights"] = mResources.compressedLightBuffer;
    vars["gLightReservoirs"] = mResources.reservoirBuffer;
    vars["gLocalLightPdfTexture"] = mResources.localLightPdfTexture;
    vars["gEnvMapPdfTexture"] = mResources.environmentPdfTexture;
    vars["gNeighborBuffer"] = mResources.neighborOffsetBuffer;
    vars["gMotionVectorTexture"] = renderData["mvec"]->asTexture();

    // Some debug textures
    vars["gHasSampleChangedInReusing"] = renderData["hasSampleChangedInReusing"]->asTexture();
}

bool RTXDITutorialBase::allocateRtxdiResrouces(RenderContext* pContext, const RenderData& data)
{
    // We can't allocate resources without a scene
    if (!mPassData.scene) return false;

    // Ask for some other refreshes elsewhere to make sure we're all consistent
    mPassData.clearReservoirs = true;
    mPassData.updateEmissiveTriangleFlux = true;
    mPassData.updateEmissiveTriangleGeom = true;
    mPassData.updateEnvironmentMapPdf = true;

    // Make sure the RTXDI context has the current screen resolution
    mRtxdiContextParams.RenderWidth = mPassData.screenSize.x;
    mRtxdiContextParams.RenderHeight = mPassData.screenSize.y;

    // Set the number and size of our presampled tiles 
    mRtxdiContextParams.TileSize = mLightingParams.presampledTileSize;
    mRtxdiContextParams.TileCount = mLightingParams.presampledTileCount;
    mRtxdiContextParams.EnvironmentTileSize = mLightingParams.presampledTileSize;
    mRtxdiContextParams.EnvironmentTileCount = mLightingParams.presampledTileCount;

    // Create a new RTXDI context.
    mpRtxdiContext = std::make_unique<rtxdi::Context>(mRtxdiContextParams);
    if (!mpRtxdiContext) return false;

    // Allocate a neighbor offset buffer
    //   -> This really should be a one-time cost; no need to rebuild it as the 8k size is plenty
    //      big, yet is cheap and cache-coherent to use.  Should set and forget this.
    updateNeighborBuffer();

    // Allocate our "RIS buffers" which is what RTXDI calls the buffers containing presampled light tiles
    // (see the HPG 2021 paper, "Rearchitecting Spatiotemporal Resampling for Production")
    uint32_t tileBufferCount = std::max(mpRtxdiContext->GetRisBufferElementCount(), 1u);
    mResources.lightTileBuffer = Buffer::createTyped(ResourceFormat::RG32Uint, tileBufferCount);

    // Each of our presampled lights in the tiles above can have their geometric info stashed in a packed
    // format, which improves coherency (over calling RAB_LoadLightInfo() from the global list of scene lights)
    // Each light currently takes 2x RGBA32F values in this packed format.
    mResources.compressedLightBuffer = Buffer::createTyped(ResourceFormat::RGBA32Float, 2 * tileBufferCount);

    // Allocate light index mapping buffer.  (TODO) Currently unused.  The right thing to do with this 
    // buffer is create a correspondance between this frame's light and last frame's lights.  However,
    // since Falcor currently doesn't (really) allow much dynamic lighting, beyond animating lights, we
    // skip this.  Our RTXDI bridge uses the identity mapping when asked for frame-to-frame corresponances.
    uint32_t lightBufferElements = 2 * mPassData.lights->getTotalLightCount();
    mResources.lightIndexMapBuffer = Buffer::createTyped(ResourceFormat::R32Uint, lightBufferElements);

    // Allocate light reservoir buffer.  There are multiple reservoirs (specified by kMaxReservoirs) concatenated together,
    // which allows RTXDI to swap between them with just an additional offset.  The number needed depends on the complexity
    // of the lighting pipeline.  2 is *probably* sufficient, but 3 gives a bit more flexibility.
    uint32_t reservoirBufferElementCount = mpRtxdiContext->GetReservoirBufferElementCount() * kMaxReservoirs;
    mResources.reservoirBuffer = Buffer::createStructured(uint32_t(sizeof(RTXDI_PackedReservoir)), reservoirBufferElementCount);

    // Allocate texture for environment map importance sampling.
    if (mPassData.scene->getEnvMap())
    {
        uint envWidth = mPassData.scene->getEnvMap()->getEnvMap()->getWidth();
        uint envHeight = mPassData.scene->getEnvMap()->getEnvMap()->getHeight();
        mResources.environmentPdfTexture = Texture::create2D(envWidth, envHeight, ResourceFormat::R16Float, 1, uint(-1), nullptr, kAutoMipBindFlags);
    }

    // Allocate local light PDF map, which RTXDI uses for importance sampling emissives
    uint32_t texWidth, texHeight, texMipLevels;
    rtxdi::ComputePdfTextureSize(mPassData.lights->getTotalLightCount(), texWidth, texHeight, texMipLevels);
    mResources.localLightPdfTexture = Texture::create2D(texWidth, texHeight, ResourceFormat::R16Float, 1, uint(-1), nullptr, kAutoMipBindFlags);

    // Allocate buffers to store primary hit data.  (Think: G-Buffers)  See `RTXDI_PrepareSurfaceData.cs.slang` for
    // discussion of *why* you might do this rather than using your existing G-Buffer format, whatever it is.  These
    // are, strictly speaking, not necessary if you use your existing G-buffer directly as input to the RTXDI bridge
    mResources.gBufferData = Buffer::createTyped<float4>(2 * mPassData.screenSize.x * mPassData.screenSize.y, kBufferBindFlags);
    mResources.emissiveColors = Texture::create2D(mPassData.screenSize.x, mPassData.screenSize.y, ResourceFormat::RGBA16Float, 1, uint(-1), nullptr, kBufferBindFlags);

    // Falcor stores light info in a fairly complex structure that involves some memory indirection.  In order to
    // improve memory coherency during resampling, where touching light data in a poor way can be catastrophic, 
    // we pack data into a coherent structure.
    mResources.lightGeometry = Buffer::createTyped<float4>(2 * mPassData.lights->getTotalLightCount());

    return true;
}

ComputePass::SharedPtr RTXDITutorialBase::createComputeShader(const std::string& file, const std::string& entryPoint)
{
    // Where is the shader?  What shader model to use?
    Program::Desc risDesc(kShaderDirectory + file);
    risDesc.setShaderModel("6_5");

    // Create a Falcor shader wrapper, with specified entry point and scene-specific #defines (for common Falcor HLSL routines)
    ComputePass::SharedPtr pShader = ComputePass::create(risDesc.csEntry(entryPoint), mPassData.scene->getSceneDefines());

    // Type conformances are needed for specific Slang language constructs used in common Falcor shaders, no need to worry
    // about this unless you're using advanced Slang functionality in a very specific way.
    pShader->getProgram()->setTypeConformances(mPassData.scene->getTypeConformances());

    // Zero out structures to make sure they're regenerated with correct settings (conformances, scene defines, etc)
    pShader->setVars(nullptr);

    // Falcor doesn't, by default, pass down scene geometry, materials, etc. for compute shaders.  But we'll need that
    // in all (or most) of our shaders in these tutorials, so just automatically send this data down.
    pShader->getRootVar()["gScene"] = mPassData.scene->getParameterBlock(); // Song: do we really need this? 
    return pShader;
}

void RTXDITutorialBase::loadCommonShaders(void)
{
    // The current setup configuration requires having a scene *before* loading shaders.
    if (!mPassData.scene) return;

    // Load utility shaders needed to setup (some) pipeline variants and data structures for RTXDI.
    mShader.createLightResources = createComputeShader(kRTXDI_CreateLightResources);
    mShader.createEnvironmentPDFTexture = createComputeShader(kRTXDI_CreateEnvMapPDFTexture);
    mShader.presamplePrimitiveLights = createComputeShader(kRTXDI_PresamplePrimitiveLights);
    mShader.presampleEnvironmentLights = createComputeShader(kRTXDI_PresampleEnvironmentLights);
    mShader.prepareSurfaceData = createComputeShader(kRTXDI_PrepareSurfaceData);
}

void RTXDITutorialBase::loadShaders()
{
    // The current setup configuration requires having a scene *before* loading shaders.
    if (!mPassData.scene) return;

    // Load shaders common to all tutorials
    loadCommonShaders();

    // Load a shader that does a simple Monte Carlo baseline
    mShader.monteCarloBaseline = createComputeShader(kRTXDI_MonteCarloBaseline);

    // Load our shaders for our simple RIS-only RTXDI example
    mShader.risOnlyInitialCandidates = createComputeShader(kRTXDI_RISOnly_InitialSamples);
    mShader.risOnlyShade = createComputeShader(kRTXDI_RISOnly_Shade);

    // Load a shader that only does a single spatial reuse pass (can be run iteratively)
    mShader.spatialReuse = createComputeShader(kRTXDI_SpatialReuseOnly);

    // Load a shader that does temporal reuse
    mShader.temporalReuse = createComputeShader(kRTXDI_TemporalReuseOnly);

    // Load a shader that combines temporal reuse with a (single) spatial/spatiotemporal pass
    mShader.spatiotemporalReuse = createComputeShader(kRTXDI_SpatiotemporalReuse);

    // Load common shaders (across multiple example RTXDI pipelines) for generating initial candidates and doing final shading
    mShader.initialCandidates = createComputeShader(kRTXDI_InitialSamples);
    mShader.initialCandidateVisibility = createComputeShader(kRTXDI_InitialVisibility);
    mShader.shade = createComputeShader(kRTXDI_Shade);

}


