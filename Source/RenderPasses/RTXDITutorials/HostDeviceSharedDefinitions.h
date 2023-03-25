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

struct LightFaceData
{
    uint shadowMapType; // 0: None, 1: PSM, 2: ISM, 3: High resolution ISM (!!!)
    uint shadowMapSize;
    uint frequency; // number of pixel selected

    // Used for reducing the flickering by restricting the face ranking change
    //float weightSum;
    //float weightSum2;
    float mean;
    // float variance;
    float varianceUnbiased;

    // View matrices. (Paraboloid projection will only use -Y (index 3) and +Y (index 2) as front and back)
    float4x4 viewMat;
    float4x4 normalViewMat;
    //float4x4 prevViewMat; // previous frame view matrix. This is used for light motion

    // TODO: this actually is the opposite direction of the second row of viewMat(negative z)
    float3 viewDirection; // Defined in light world space.
    float3 viewDirectionBase; // six axis-aligned base direction from the start. Defined in light world space.

    // Variable for indexing into two sorted texture array. (For per-light base, these variables should also be set for each face)
    uint whichPsmTexArray;
    int psmTexArrayIdx; // this is also the index in PSM reusing list
    int staticPsmTexArrayIdx; // since PSM array for static objects are inconsistent with indexing in the reusing list
    uint whichIsmTexArray;
    int ismTexArrayIdx;
    uint whichHighResIsmTexArray;
    int highResIsmTexArrayIdx;

    // Temporal related information.
    uint accumulatedFrames;
    uint prevRanking; // update frequency depends on temporal resuing length
    uint currRanking; // updating each frame
    bool newPsmLightFace; // used for static PSM rendering
    uint psmAge; // used for motion vector, age since last PSM update

};

struct LightShadowMapData
{
    bool isLightValid;
    //bool isDynamic;
    float3 centerPosW;
    float2 nearFarPlane;
    float lightFrustumSize;
    uint falcorLightID;

    // Point or spot light
    uint numShadowMaps;

    // Projection matrix
    float4x4 persProjMat;
    float4x4 normalPersProjMat;

    // Face rotation matrix w.r.t. axis-aligned directions. In light world space.
    float4x4 rotationFromBase;

    // Sum over all pixel's direction select this light. Reset every frame
    float3 currAvgDirection;
    float3 accumAvgDirection;

    LightFaceData lightFaceData[6];
};


struct PackedIsmPointSample // size: 10B
{
    float16_t3 pos;
    uint packedLightInstanceID;
};

// How each pixel's visibility is evaluted?
enum class Visibility : uint
{
    AllShadowRay = 0,
    ShadowMap_FullyLit = 1,
    ShadowMap_ISM = 2,
    AllISM = 3,

    BaselineSM = 4,

    Experiment = 5 // All Shadow Maps
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
    AllPixels = 0,
    OccludedPixels = 1,
    LightFaces = 2
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




END_NAMESPACE_FALCOR
