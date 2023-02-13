#pragma once

#include "Utils/HostDeviceShared.slangh"
BEGIN_NAMESPACE_FALCOR

#ifdef HOST_CODE
#include "Falcor.h"
#endif

static const uint kMaxSupportedLights1024 = 256; // TODO: better can be divided by top N light in each frame

struct LightFaceData
{
    uint shadowMapType; // 0: None, 1: PSM, 2: ISM
    uint shadowMapSize;

    // View matrices. (Paraboloid projection will only use -Y (index 3) and +Y (index 2) as front and back)
    float4x4 viewMat;
    //float4x4 prevViewMat; // previous frame view matrix. This is used for light motion

    // Variable for indexing into two sorted texture array. (For per-light base, these variables should also be set for each face)
    uint whichPsmTexArray;
    int psmTexArrayIdx; // for per-light base, it is the start index (true index should also add face idx in the evaluating part)
    uint whichIsmTexArray;
    int ismTexArrayIdx;

    // Temporal related information. 
    uint accumulatedFrames;
    uint prevRanking; // update frequency depends on temporal resuing length
    uint currRanking; // updating each frame
    //bool newPsmLightFace;

    // Data to print on CPU side
    uint2 rankingFreqInFrame;
};

struct LightShadowMapData
{
    bool isLightValid;
    float3 centerPosW;
    float2 nearFarPlane;
    float lightFrustumSize;

    // Projection matrix
    float4x4 persProjMat;

    LightFaceData lightFaceData[6];
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



END_NAMESPACE_FALCOR
