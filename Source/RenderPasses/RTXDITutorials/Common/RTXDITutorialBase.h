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
#include "../rtxdi/RTXDI.h"
#include <random>
#include <numeric>
#include "../HostDeviceSharedDefinitions.h"

// For baseline method use
#include "Rendering/Lights/EmissivePowerSampler.h"
#include "Rendering/Lights/EmissiveUniformSampler.h"
#include "Utils/Sampling/SampleGenerator.h"

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
    virtual void execute(RenderContext* pRenderContext, const RenderData& renderData) override;
    virtual void renderUI(Gui::Widgets& widget) override;
    virtual void setScene(RenderContext* pRenderContext, const Scene::SharedPtr& pScene) override;
    virtual bool onMouseEvent(const MouseEvent& mouseEvent) override { return mpPixelDebug->onMouseEvent(mouseEvent); }
    virtual bool onKeyEvent(const KeyboardEvent& keyEvent) override;

    // Which part in render pass has changed and needs update
    enum class UpdateFlags
    {
        None = 0x0,
        VisibilityChanged = 0x1,
        ResourcesChanged = 0x2,
        ShaderDefinesChanged = 0x4,
        All = -1
    };

protected:
    RTXDITutorialBase(const Dictionary& dict, const RenderPass::Info& desc);

    PixelDebug::SharedPtr mpPixelDebug;

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

        // Below are for analytic lights
        bool updateLightPosition = true;
        bool updateLightIntensity = true;
        bool updateLightOtherProperties = true;
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
        uint32_t primLightSamples = 32u;         // How many samples do we take on emissive primitive lights (in Falcor currently, just triangles)
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

    ////////////////////////////////////////////////////////////////////////////////////////////
    // ReSTIR shadow map parameters & resources
    ////////////////////////////////////////////////////////////////////////////////////////////

    /** All types, parameters and resources for this render pass
    */ 
    enum class SortingRules { AllPixels, OccludedPixels, ShadowEdgePixels};
    enum class ISMSceneType { Triangle, ThreePoints, CenterPoint };
    enum class ISMPushSamplingMode { Point, Interpolation };

    Visibility mVisibility = Visibility::ShadowMap_ISM;
    UpdateFlags mUpdates = UpdateFlags::None;
    GpuFence::SharedPtr mpFence;
    Sampler::SharedPtr mpPointSampler;
    Sampler::SharedPtr mpLinearSampler;
    uint mSceneName = 0; // TODO: use pybind11 to register scene enum on python side 

    // Below are parameters for rendering, scene or updating control
    bool mEnableStatistics = false;
    bool mShadowParamsChanged = false;
    bool mTurnOffShadowRay = false;
    bool mShadowRayForRestPixels = false;
    bool mShadingVisibility = true;
    bool mDrawWireframe = false;
    bool mAdaptiveLightNearPlane = false;
    bool mFullSizeShadowMaps = true;
    bool mAdaptiveISM = false;
    bool mOnlyUseIsmForTesting = false;

    // Below are the parameters for debugging
    bool mFrozenFrame = false;
    bool mDisplayLightSampling = false;
    bool mDebugLight = false;
    uint mDebugLightMeshID = 0u;
    uint mVisualizeMipLevel = 0;
    uint2 mVisLightFaceID = uint2(0);

    struct 
    {
        bool getShadowMapSizeData = false;
        bool getSortingData = false;
        bool getIsmData = false;
        bool getConverageData = false;
    } mGpuDataToGet;

    // Below are the light infos  
    uint mTotalLightMeshCount = 0u;
    uint mTotalPointLightsCount = 0u;
    uint mTotalLightsCount = 0u;
    std::vector<PointLight*> mPointLights;
    float2 mLightNearFarPlane = float2(0.001f, 100.0f);

    // Below are the parameters in our method
    uint mLightTopN = 4u;
    uint mTemporalReusingLength = 5u;
    uint mCurrFrameLightStartIdx = 0u;
    uint mShadowMapSize = 1 << 10;
    float mSmDepthBias = 1e-6f;
    float mConstEpsilonForAdaptiveDepthBias = 0;
    uint mUpdatingFrequency = 1u;
    uint mSortingRules = (uint)SortingRules::AllPixels;
    uint mShadowDepthBias = (uint)ShadowDepthBias::SlopeScale;

    // ISM parameters
    uint mTotalIsmCount = 0u;
    float mIsmDepthBias = 0.003f; 
    uint mIsmPerLight = 2u;
    uint mIsmVisTextureSize = 2048u; // max light: 2k*2k -> 128, 4k*4k -> 512 
    uint mIsmSize = 128u;
    uint mIsmMipLevels = 3;
    float mSceneDepthThresholdScale = 0.01f;
    float mDepthThreshold = 0.0f;
    float mBaseTriSize = 0.01f;
    uint mIsmLightSamplingMode = 0u;
    uint mIsmPushMode = (uint)ISMPushSamplingMode::Point;
    uint mIsmSceneType = (uint)ISMSceneType::ThreePoints;

    // Global resources
    Buffer::SharedPtr mpLightShadowDataBuffer; // mesh lights + point lights
    Buffer::SharedPtr mpPrevLightSelectionBuffer;
    Buffer::SharedPtr mpLightHistogramBuffer;
    Buffer::SharedPtr mpSortedLightsBuffer;
    Buffer::SharedPtr mpTotalValidPixels;
    Buffer::SharedPtr mpLightPdfBuffer;
    Buffer::SharedPtr mpLightCdfBuffer;

    // Temporal resuing shadow map resources
    Buffer::SharedPtr mpReusingLightIndexBuffer;
    //Texture::SharedPtr mpReusingShadowMapsTexture; // TODO: use vector to store multiple texture arrays to hold all lights SM (in Emerald Scene)

    // ISM resources
    Texture::SharedPtr mpIsmTextureArray; // Max lights = 1024
    Texture::SharedPtr mpIsmTexture;
    Buffer::SharedPtr mpIsmLightIndexBuffer;
    Buffer::SharedPtr mpPointsBuffer;
    Buffer::SharedPtr mpCounterBuffer;

    // Experiments with different shadow map resolutions
    //std::vector<Texture::SharedPtr> mReusingSmTextureArrays; // 32, 64, 128, 256, 512, 1024 (max = 172 lights)
    //std::map<uint, std::vector<Texture::SharedPtr>> mReusingSmTextureArrays;

    std::vector<Texture::SharedPtr> mSortedLightsShadowMaps;

    /** Passes for computing and updating the top N lighs 
    */
    struct
    {
        ComputePass::SharedPtr computeLightHistogram;
        ComputePass::SharedPtr createKeyValuePairs;

        struct
        {
            ComputePass::SharedPtr preSort;
            ComputePass::SharedPtr innerSort;
            ComputePass::SharedPtr outerSort;
        } bitonicSort;

    } mComputeTopLightsPass;

    // Pass for updating light shadow data buffer when light data changes in animation
    ComputePass::SharedPtr mpUpdateLightShadowDataCenter;

    ComputePass::SharedPtr mpUpdateLightMeshData;

    /** Normal shadow map pass
    */ 
    struct
    {
        GraphicsProgram::SharedPtr pProgram;
        GraphicsState::SharedPtr pState;
        GraphicsVars::SharedPtr pVars;
        Fbo::SharedPtr pFbo;
    } mShadowMapPass;

    /** ISM passes
    */
    ComputePass::SharedPtr mpBuildPDF;
    ComputePass::SharedPtr mpBuildCDF;

    struct
    {
        GraphicsProgram::SharedPtr pProgram;
        GraphicsState::SharedPtr pState;
        //GraphicsStateObject::SharedPtr pGSO;
        GraphicsVars::SharedPtr pVars;
        Fbo::SharedPtr pFbo;
    } mIsmRenderPass;

    // TODO: using CS to render ISMs
    struct
    {
        ComputePass::SharedPtr generatePoints;
        ComputePass::SharedPtr renderISMs;
    } mCsIsmRenderPass;

    ComputePass::SharedPtr mpIsmPullPass;
    ComputePass::SharedPtr mpIsmPushPass;

    RasterizerState::SharedPtr mpWireframeRS;
    DepthStencilState::SharedPtr mpNoDepthDS;
    DepthStencilState::SharedPtr mpDepthTestDS;

    /** Baseline shadow map part
    */
    EmissiveLightSampler::SharedPtr mpEmissiveSampler;
    Texture::SharedPtr mpHeroLightShadowMapsTexture;
    ComputePass::SharedPtr mpBaselineShading;

    /** Helpers to debug or visualize
    */
    struct
    {
        std::vector<float> shadowOptionsCoverage1;
        std::vector<float> shadowOptionsCoverage2;

        std::vector<uint> extraPointsCount;
        uint totalExtraPoints;

        std::string sortedLightIndexArray;
        std::string reusingLightIndexArray;
        std::string ismLightIndexArray;

        std::string shadowMapSizeCount;
        uint64_t totalShadowTexSize;
    } mValuesToPrint;

    // Recording data resources
    Buffer::SharedPtr mpShadowOptionsBuffer; // Each pixel's shadow option (two places). Stores how each pixel's shadow is evalutated
    Buffer::SharedPtr mpExtraPointsCountBuffer;
    Buffer::SharedPtr mpShadowMapSizeBuffer;

    struct
    {
        ComputePass::SharedPtr pVisualizeSingle;
        ComputePass::SharedPtr pVisualizeAll;
        ComputePass::SharedPtr pVisualizeAll2;
    } mVisualizePass;

    struct
    {
        GraphicsProgram::SharedPtr pProgram;
        GraphicsState::SharedPtr pState;
        GraphicsVars::SharedPtr pVars;
        Fbo::SharedPtr pFbo;
    } mWireframePass;

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

    ////////////////////////////////////////////////////////////////////////////////////////////
    // ReSTIR shadow map functions
    ////////////////////////////////////////////////////////////////////////////////////////////

    /** Global functions 
    */
    void allocateShadowTextureArrays();
    void prepareLightShadowMapData();
    void computeTopLightsPass(RenderContext* pRenderContext);
    void prepareStochasticShadowMaps(RenderContext* pRenderContext, const RenderData& data);
    ComputePass::SharedPtr createComputeShader(const std::string& file, const Program::DefineList& defines, const std::string& entryPoint = "main", bool hasScene = false);
    void visualizeShadowMapTex(ComputePass::SharedPtr pPass, Texture::SharedPtr pSrcTexture, Texture::SharedPtr pDstTexture, RenderContext* pRenderContext, uint mipLevel,
        uint2 lightFaceIdx, const std::string& mode);

    /** Some ISM functions
    */
    void loadIsmShaders();
    void prepareIsms(RenderContext* pRenderContext, const RenderData& data);

    /** Baseline shadow map function
    */ 
    void runBaselineShadowMap(RenderContext* pRenderContext, const RenderData& data);

    /** Debug & Print helper functions
    */ 
    void printDeviceResources(RenderContext* pRenderContext);
    void calcDiffShadowCoverage(const std::vector<uint32_t>& shadowOptionsList, uint offset);

    /** This function only supports getting the typed buffer data or texture with the multiple of word(4 bytes) size data
    */ 
    template <typename T>
    std::vector<T> getDeviceResourceData(RenderContext* pRenderContext, const Resource::SharedPtr& pResource)
    {
        auto resourceType = pResource->getType();
        auto size = pResource->getSize();
        std::vector<T> deviceResult;
        if (resourceType == Resource::Type::Buffer)
        {
            auto pBuffer = pResource->asBuffer();
            auto elementCount = pBuffer->getElementCount();

            if (pBuffer->getCpuAccess() != Buffer::CpuAccess::Read)
            {
                logError("Cannot read the buffer");
            }

            // Must use a staging buffer which has no bind flags to map data
            auto pStagingBuffer = Buffer::createTyped<T>(elementCount, ResourceBindFlags::None, Buffer::CpuAccess::Read);
            pRenderContext->copyBufferRegion(pStagingBuffer.get(), 0, pBuffer.get(), 0, size);

            // Flush GPU and wait for results to be available.
            pRenderContext->flush(false);
            mpFence->gpuSignal(pRenderContext->getLowLevelData()->getCommandQueue());
            mpFence->syncCpu();

            // Read back GPU results
            T* pData = reinterpret_cast<T*>(pStagingBuffer->map(Buffer::MapType::Read));
            deviceResult.assign(pData, pData + elementCount); // invoke range constructor (linear time)
            pStagingBuffer->unmap();
        }
        else
        {
            auto pTexture = pResource->asTexture();
            auto elementCount = pTexture->getWidth() * pTexture->getHeight();
            deviceResult.reserve(elementCount);

            // Texture has a built-in function to get data from GPU
            auto dataInBytes = pRenderContext->readTextureSubresource(pTexture.get(), 0);

            // Loop over the texture
            for (uint i = 0; i < elementCount; i++)
            {
                // Convert byte data to the target type
                T targetTypeData;
                uint nWord = sizeof(T) / 4;
                for (uint j = 0; j < nWord; j++)
                {
                    int startIndex = i * sizeof(T) + j * nWord;
                    uint8_t bytesToFloat[4] = {
                        dataInBytes[startIndex], dataInBytes[startIndex + 1],
                        dataInBytes[startIndex + 2], dataInBytes[startIndex + 3],
                    };
                    memcpy(&targetTypeData + j, &bytesToFloat, 4);
                }
                deviceResult[i] = targetTypeData;
            }
        }

        return deviceResult;
    }
    
};

// Operator overload for enum class type
typedef RTXDITutorialBase::UpdateFlags e_;
inline e_ operator& (e_ a, e_ b) { return static_cast<e_>(static_cast<int>(a) & static_cast<int>(b)); }
inline e_& operator&= (e_& a, e_ b) { a = a & b; return a; };
inline e_ operator| (e_ a, e_ b) { return static_cast<e_>(static_cast<int>(a) | static_cast<int>(b)); }
inline e_& operator|= (e_& a, e_ b) { a = a | b; return a; }
