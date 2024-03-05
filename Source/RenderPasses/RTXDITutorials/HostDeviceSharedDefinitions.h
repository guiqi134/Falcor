#pragma once

#include "Utils/HostDeviceShared.slangh"
BEGIN_NAMESPACE_FALCOR

#ifdef HOST_CODE
#include "Falcor.h"
#endif

static const uint kMaxSupportedLights1024 = 256; // TODO: better can be divided by top N light in each frame
static const uint kShadowMapsPerPointLight = 6u;
static const uint kShadowMapsPerSpotLight = 1u;
static const uint kMinPointSamplesPerTri = 3u;

struct LightFaceData // size = 124B
{
    uint shadowMapType = 0; // 0: None, 1: PSM, 2: ISM, 3: High resolution ISM (!!!)
    uint shadowMapSize = 0;
    uint frequency = 0; // number of pixel selected

    // Used for reducing the flickering by restricting the face ranking change
    float mean = 0.0f;
    float varianceUnbiased = 0.0f;

    // View matrices. (Paraboloid projection will only use -Y (index 3) and +Y (index 2) as front and back)
    // the opposite direction of the third row of viewMat(negative z) is the view direction
    float4x4 viewMat = {};

    // Variable for indexing into two sorted texture array. (For per-light base, these variables should also be set for each face)
    uint whichPsmTexArray = 0;
    int psmTexArrayIdx = -1; // this is also the index in PSM reusing list
    uint whichIsmTexArray = 0;
    int ismTexArrayIdx = -1;
    uint whichHighResIsmTexArray = 0;
    int highResIsmTexArrayIdx = -1;

    // Temporal related information.
    uint prevRanking = 0; // update frequency depends on temporal resuing length
    uint currRanking = 0; // updating each frame

    // For shadow fade in 
    bool typeChanged = false; // ISM -> PSM
    uint fadeInFrameCount = 1;
};

struct LightShadowMapData // size = ?
{
    float3 centerPosW = float3(0);
    float2 nearFarPlane = float2(0);
    float lightFrustumSize = 0.0f;
    uint falcorLightID = 0;

    // Point or spot light
    uint numShadowMaps = 0;

    // Projection matrix
    float4x4 persProjMat = {};

    LightFaceData lightFaceData[6];
};

struct PackedIsmPointSample // size: 16B 
{
    float3 pos;
    uint packedLightInstanceID;
};

struct PCF_Parameters
{
#ifdef HOST_CODE
    void setShaderData(const ShaderVar& var)
    {
        var["usePCF"] = usePCF;
        var["usePoisson"] = usePoisson;
        var["useRotation"] = useRotation;
        var["kernelRadius"] = kernelRadius;
        var["numPcfPoissonSamples"] = numPcfPoissonSamples;
    }
#endif

    bool usePCF = true;
    bool usePoisson = false;
    bool useRotation = false;
    float kernelRadius = 1.0f;
    //float kernelRadius = 0.0f;
    uint numPcfPoissonSamples = 32u;
};

// How each pixel's visibility is evaluted?
enum class Visibility : uint
{
    AllShadowRay = 0,
    ShadowMap_FullyLit = 1,
    ShadowMap_ISM = 2,
    AllISM = 3,
};

enum class ShadowMapType : uint
{
    None = 0,
    PSM = 1,
    LowIsm = 2,
    HighIsm = 3,
};

enum class ShadowDepthBias : uint
{
    Constant = 0,
    SlopeScale = 1,
    Dou2014 = 2
};

enum class SortingRules : uint
{
    Light = 0,
    Light_OccludedPixels = 1,
    LightFaces = 2,
    LightFace_OccludedPixels = 3
};

enum class ShadowOptions : uint
{
    InValid = 0,
    FullyLit = 1,
    PSM = 2,
    ISM = 3,
    ShadowRay = 4
};

enum class TemporalReusingFix : uint
{
    None = 0,
    SplitRendering = 1,
    MotionVector = 2
};

enum class FlickerReduction : uint
{
    None = 0,
    RegionRestricted = 1,
    VarianceCheck = 2
};

enum class ISMPushSamplingMode : uint
{
    Point = 0,
    Interpolation = 1
};

// Comparison between different importance for the ranking
enum class RankingImportance : uint
{
    ReSTIR = 0,
    onlyUniformRIS = 1,
    onlyEmissivePowerRIS = 2,
};

// Comparison for different three places to collect sampling data
enum class PlacesForRankingData : uint
{
    AfterRIS = 0,
    AfterVisibilityCheck = 1,
    AfterReusing = 2,

    EmissivePowerFixed = 3
};



END_NAMESPACE_FALCOR
