#pragma once
#include "Utils/HostDeviceShared.slangh"
BEGIN_NAMESPACE_FALCOR

#ifdef HOST_CODE
#include "Falcor.h"
#endif

static const uint kMaxSupportedLights1024 = 256;

struct LightShadowMapData
{
    float3 centerPosW;
    float2 nearFarPlane;
    int shadowMapCount;
    uint shadowMapType;
    uint shadowMapSize;
    float lightFrustumSize;

    // TODO: these two indexes can be combined into one
    uint ranking; // -> shadow map array start index
    int ismArrayStartIndex;

    // Perspective projection will use all of them. Paraboloid projection will only use -Y (index 3) and +Y (index 2) as front and back.
    float4x4 viewMats[6];

    // Composed matrix for perspective projection
    float4x4 viewProjMats[6];
    float4x4 persProjMat;

    // Temporal related information
    int reusingArrayIndex;
    uint age;
    //bool isNewShadowMapLight;
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


END_NAMESPACE_FALCOR
