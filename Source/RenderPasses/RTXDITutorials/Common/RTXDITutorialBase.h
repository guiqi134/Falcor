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

/** This is the base Falcor RenderPass used by all of the RTXDI Tutorials.
 
    Note:  You *will* look at this class as you try to understand the tutorials.
    But you should *not* start by reading this base class.

    This class contains common resource allocators, resources, shader I/O, etc.,
    that allows us to keep a common codebase for these across all tutorials.

    However, please look at the individual tutorial steps to understand which
    of these common bits are needed at each stage of integration.
*/

#include "Falcor.h"
#include "rtxdi/RTXDI.h"
#include <random>

using namespace Falcor;

class RTXDITutorialBase : public RenderPass
{
public:
    using SharedPtr = std::shared_ptr<RTXDITutorialBase>;


    ////////////////////////////////////////////////////////////////////////////////////////////
    // Callbacks for generic RenderPass objects that we must override 
    ////////////////////////////////////////////////////////////////////////////////////////////

    virtual RenderPassReflection reflect(const CompileData& compileData) override;
    virtual void compile(RenderContext* pContext, const CompileData& compileData) override;
    virtual void execute(RenderContext* pContext, const RenderData& renderData) override;
    virtual void renderUI(Gui::Widgets& widget) override;
    virtual void setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene) override;
    virtual bool onMouseEvent(const MouseEvent& mouseEvent) override { return mpPixelDebug->onMouseEvent(mouseEvent); }
    virtual bool onKeyEvent(const KeyboardEvent& keyEvent) override;

protected:
    RTXDITutorialBase(const Dictionary& dict, const RenderPass::Info& desc);

    /** Config setting : Maximum number of unique screen-sized reservoir bufers needed by any
        of our derived RTXDI tutorials.  This just controls memory allocation (not really perf)
    */
    static const uint32_t kMaxReservoirs = 3;   // 3 is probably sufficient with careful ping-ponging

    // Structures built by, for, or needed explicitly by RTXDI
    rtxdi::ContextParameters           mRtxdiContextParams;      // Parameters that largely stay constant during program execution
    rtxdi::FrameParameters             mRtxdiFrameParams;        // Parameters likely to (or easy to) change from frame-to-frame
    RTXDI_ResamplingRuntimeParameters  mRtxdiShaderParams = {};  // Structure passed per-frame, as-is, to our HLSL shaders for use on the GPU
    std::unique_ptr<rtxdi::Context>    mpRtxdiContext = nullptr; // Our RTXDI context

    /** A structure containing all the resources RTXDI needs allocated
    */
    struct {
        // Resources RTXDI specifically asks to be allocated
        Buffer::SharedPtr   lightTileBuffer = nullptr;        // Stores precomputed light tiles (see presampleLights()), lightIndex and invSourcePdf
        Buffer::SharedPtr   lightIndexMapBuffer = nullptr;    // Stores mapping between current & last frame lights (currently unused)
        Buffer::SharedPtr   neighborOffsetBuffer = nullptr;   // Stores a Poisson(-ish) distribution of offsets for finding randomized neighbors
        Buffer::SharedPtr   reservoirBuffer = nullptr;        // Stores intermediate reservoirs between kernels (and between frames)
        Texture::SharedPtr  environmentPdfTexture = nullptr;  // Used and created to sample our environment map proportional to intensity (times solid angle)
        Texture::SharedPtr  localLightPdfTexture = nullptr;   // Used and created to sample our triangle emissive proportional to intenstiy
        Buffer::SharedPtr   compressedLightBuffer = nullptr;  // RTXDI can optionally rearrange light data to improve coherence; it stores it here

        // Resources we allocate to store geometry data in whatever format we need it to be to load it in the RTXDI bridge
        Buffer::SharedPtr   gBufferData = nullptr;            // Our packed G-buffer data for both this frame & last frame
        Texture::SharedPtr  emissiveColors = nullptr;         // RTXDI compute reflected color.  This stores emission from directly-seen lights

        // Resources we allocate to store light data for the RTXDI bridge
        Buffer::SharedPtr   lightGeometry;                    // Repacked from Falcor data to improve cache coherency during resampling
    } mResources;


    /** A structure containing controls and data for this RenderPass, but not specific to RTXDI
    */
    struct {
        Scene::SharedPtr scene = nullptr;                     // A reference to our Falcor scene structure
        uint2            screenSize;                          // Our current window size
        bool             updateEmissiveTriangleFlux = true;   // Set if triangle emissive flux changed (so we need to update relevant resources)
        bool             updateEmissiveTriangleGeom = true;   // Set if our emissive triangles may have moved (so we need to update resourecs)
        bool             updateEnvironmentMapPdf = false;     // Do we need to update our environment map sampling texture?
        bool             clearReservoirs = false;             // Has the UI changed something such that it would be a good idea to clear reservoirs?
        LightCollection::SharedPtr       lights;              // A shortcut to the class that collects the lights in our scene.
    } mPassData;

    /** HLSL shaders used in our RTXDI integration
    */
    struct {
        // Common shaders across all RTXDI pipelines
        ComputePass::SharedPtr initialCandidates;
        ComputePass::SharedPtr initialCandidateVisibility;
        ComputePass::SharedPtr shade;

        // Shaders that kick off various types of ReSTIR reuse using RTXDI
        ComputePass::SharedPtr spatialReuse;
        ComputePass::SharedPtr temporalReuse;
        ComputePass::SharedPtr spatiotemporalReuse;

        // Shaders to setup RTXDI structures and pipe data from Falcor into RTXDI
        ComputePass::SharedPtr createLightResources;           // Pipe data from Falcor into RTXDI; build sampling texture for triangles
        ComputePass::SharedPtr createEnvironmentPDFTexture;    // Build sampling texture for environment map
        ComputePass::SharedPtr presamplePrimitiveLights;       // Presample triangles into per-frame light tiles
        ComputePass::SharedPtr presampleEnvironmentLights;     // Presample the environment into per-frame light tiles
        ComputePass::SharedPtr prepareSurfaceData;             // Load and/or pack primary surface hits for efficient loading by the RTXDI bridge

        // A baseline pipeline for comparison (doing simple Monte Carlo integration)
        ComputePass::SharedPtr monteCarloBaseline;

        // A baseline pipeline doing simple Talbot RIS (split into 2, roughly equivalent to "initialCandidates" and "shade")
        ComputePass::SharedPtr risOnlyInitialCandidates;
        ComputePass::SharedPtr risOnlyShade;
    } mShader;


    /** Parameters specifically to pass to RTXDI shaders and pipelines
    */
    struct {
        // We provide a number of sample RTXDI pipelines and baslines.  Which one is currently displayed?
        uint32_t selectedDisplayPipeline = 4u;

        // Configuration if using baselines (Monte Carlo integration or Talbot RIS); RTXDI passes defaults to the best of them (i.e., 2)
        // Note: The original ReSTIR paper uses an alias table (see Ch 21 in Ray Tracing Gems 2), which is not provided by RTXDI or used
        // in this sample.  Alias tables are faster than the light mipmap used by RTXDI, but are not easily updatable; if combined with 
        // precomputed light tiles, the runtime performance difference between alias tables and the RTXDI light mipmap are insignificant.  
        uint32_t initialLightPdfMode = 2u;       // Which basic sampling technique do we use to sample lights? 

        // Below are parameters that specify the number of initial candidates (for RTXDI) or the total samples (RIS / basic Monte Carlo)
        uint32_t primLightSamples = 1u;         // How many samples do we take on emissive primitive lights (in Falcor currently, just triangles)
        uint32_t envLightSamples = 0u;           // How many samples do we take on the environment map?

        // Below are parameters controlling initial light candidate sampling, prior to any reuse
        bool     traceInitialShadowRay = true;   // Before doing reuse, do we trace a shadow ray to the one selected candidate per pixel?
        uint32_t presampledTileCount = 128u;     // How many precomputed light tiles do we use? (Currently, we use different sets for tris & env maps)
        uint32_t presampledTileSize = 1024u;     // How large is each precomputed tile of lights?
        bool     storePerTileLightGeom = false;  // Rearrange light geometry to improve coherence (positive impact only in large scenes)

        // Below are parameters for spatial and temporal reuse
        uint32_t maxHistoryLength = 20u;         // What do we clamp M to?  (Roughly: how long is our temporal window?)
        uint32_t biasCorrectionMode = 1u;        // Which biased / unbiased mode are we using.  Default 1 = best perf/quality tradeoff for real-time
        uint32_t spatialSamples = 1u;            // Number of random spatial samples taken by each pixel
        uint32_t spatialIterations = 1u;         // Only respected by spatial-only pass.  Spatiotemporal reuse always uses only 1 iteration
        float    spatialRadius = 30.0f;          // Pixel radius within which spatial reuse is allowed
        float    depthThreshold = 0.1f;          // Relative depth difference at which pixels are deemed too far apart to be relevant (0.1 = 10%)
        float    normalThreshold = 0.5f;         // Cosine of the angle between normals, below which pixels are deemed too far apart to be relevant
        bool     useBoilFilter = false;          // Use a boil filter to supress boiling with temporal reuse?
        float    boilFilterStrength = 0.0f;      // 0.0f = off, 1.0f = full strength
        uint32_t lastFrameOutput = 1u;           // Id / offset of reservoir containing last frame's output (for temporal reuse)

        // RTXDI options not currently exposed in the sample, as it isn't configured for them to produce acceptable results
        bool     useVisibilityShortcut = false;  // Reuse visibility across frames to reduce cost; requres careful setup to avoid bias / numerical blowups
        bool     permuteTemporalSamples = false; // Can decorrelate samples from temporal reuse to help denoiser; increases apparent sample noise, however

        // Parameter controlling behavior of final shading.  Lights can have an emissive texture containing arbitrarily high frequncies.
        // To improve convergence and significantly reduce texture lookup costs, this code always uses a preintegrated emissivity over each 
        // triangle during resampling.  This preintegrated value can *also* be used for final shading, which reduces noise (quite a lot if the
        // artists go crazy on the textures), at the tradeoff of losing high frequency details in the lighting.  This option is not exposed
        // in the GUI, since few of our public Falcor scenes have sufficient numbers of textured emissives for it to matter.  We recommend
        // using "false" -- if that overblurs, consider tessellating lights to better match lighting variations.  If dynamic textured lights
        // make textured lookups vital for quality, we recommend enabling this setting on only lights where it is vital.
        bool     useTextureForShade = false;     // Use preintegrated triangle emissivity for final shading (or lookup color in the texture)?

        // Rendering parameters entirely unrelated to RTXDI and ReSTIR.  All controllable from the Python rendergraph script
        float    epsilon = 1.0e-3f;                 // Ray tracing epsilon. 
        float    relativeEnvMapWeight = 1.0f;       // Multiplier to arbitrarily increase/decrease intensity of environment map
        float    relativeTriangleWeight = 1.0f;     // Multiplier to arbitrarily increase/decrease intensity of emissive triangles
        bool     useHigherShininess = false;        // Falcor changed material models during ReSTIR research, false = old specular settings.
        bool     usePrimaryLoD = true;              // Use texture LoD for G-buffer at primary hits (rather than LoD 0)
        uint32_t currentGBufferIndex = 0u;          // Which of the two G-buffers is the current one?
        uint32_t priorGBufferIndex = 1u;            // Which of the two G-buffers is the prior one?
    } mLightingParams;

    PixelDebug::SharedPtr mpPixelDebug;

    // SSRT parameters 
    enum class VisibilityMode { ShadowRay, SSRT };
    VisibilityMode mVisibilityMode = VisibilityMode::SSRT;
    Sampler::SharedPtr mpTrilinearSampler;
    bool mFrozenFrame = false;
    bool mResetAccumulation = false;
    bool mShadowRayForShading = false;

    struct
    {
        GraphicsProgram::SharedPtr pProgram;
        GraphicsVars::SharedPtr pVars;
        GraphicsState::SharedPtr pState;
        Fbo::SharedPtr pFbo;
    } mZBufferPass;

    struct
    {
        float2 csRayOriginLenEpsilon = float2(0.01f, 0.02f); // Offset values for camera space ray origin and length
        float csZThickness = 0.02f;          // Camera space thickness to ascribe to each pixel in the depth buffer
        float stride = 1.0f;                // Step in horizontal or vertical pixels between samples. This is a float because integer math is slow on GPUs, but should be set to an integer >= 1
        float jitterFraction = 0.0f;        // Number between 0 and 1 for how far to bump the ray in stride units to conceal banding artifacts
        float maxSteps = 400.0f;            // Maximum number of iterations. Higher gives better images but may be slow
        uint layers = 2u; // Number of layers for z-buffer
        bool clipToFrustum = false;
    } mSSRTParams;

    // TODO: this may can be integrated into G-Buffer instead of a seperated raster pass
    struct
    {
        Texture::SharedPtr currZBufferTexture;
        Texture::SharedPtr prevZBufferTexture;
    } mSSRTResources;

    ////////////////////////////////////////////////////////////////////////////////////////////
    // Various internal pass utilities, setup/allocation routines, and common routines for
    // consistently setting common shader parameters.
    ////////////////////////////////////////////////////////////////////////////////////////////

    /** Update RTXDI parameters (on CPU and GPU) for the current frame
    */
    void setCurrentFrameRTXDIParameters(const RenderData& renderData);

    /** Initializes all RTXDI bridge variables (in one method to ensure consistency across various shaders)
    */
    void setupRTXDIBridgeVars(ShaderVar& vars, const RenderData& renderData);

    /** Create PDF textures used to importance sample lights (and env maps) proportional to intensity
    */
    void computePDFTextures(RenderContext* pContext, const RenderData& data);

    /** Create presampled light tiles for more coherent memory access during resampling
    */
    void presampleLights(RenderContext* pContext, const RenderData& data);

    /** When the RTXDI context is (re-)created, allocate appropriate resources (i.e., in mResources)
    */
    bool allocateRtxdiResrouces(RenderContext* pRenderContext, const RenderData& renderData);

    /** Build a texture used to stratify sampling of neighbors (for spatial reuse)
    */
    bool updateNeighborBuffer();

    /** Load and compile all the shaders needed in this render pass
    */
    void loadShaders();

    /** Update flags in 'mPassData' that note if we need to update light data structures due to scene changes
    */
    void checkForSceneUpdates();

    /** Our RTXDI bridge requires per-pixel G-buffer data stored in a certain format for optimal performance.
        This method launches a GPU kernel to load primary hits and materials stored in a standard Falcor formats
        and convert it to a packed G-buffer format more optimized for spatiotemporal reuse.
    */
    void prepareSurfaceData(RenderContext* pContext, const RenderData& data);

    /** A function to simplify loading shaders throughout our tutorials.
    */
    ComputePass::SharedPtr createComputeShader(const std::string& file, const std::string& entryPoint = "main");

    /** Load and compile the common shaders needed for all tutorials; these are the first shaders whose
        purposes must be understood for a very crude RTXDI integration.
    */
    void loadCommonShaders(void);

    // SSRT functions
    void setupSSRTVars(ShaderVar& vars, const RenderData& renderData, bool usePreviousZBuffer);
    void runZBufferRaster(RenderContext* pRenderContext, const RenderData& renderData);
};
