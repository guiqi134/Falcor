#pragma once
#include "Utils/HostDeviceShared.slangh"
BEGIN_NAMESPACE_FALCOR

#ifdef HOST_CODE
#include "Falcor.h"
#endif

struct LightShadowMapData
{
    float3 centerPosW;
    float2 nearFarPlane;
    uint texArrayIndex;
    int startIndex;
    int shadowMapCount;
    uint shadowMapType;
    uint shadowMapSize;
    bool isNewShadowMapLight;

    float4x4 viewProjMat[6]; // for prespective projection
    float4x4 viewMats[2]; // for paraboloid projection
};

// How each pixel's visibility is evaluted? 
enum class Visibility : uint
{
    AllShadowRay = 0,
    ShadowMap_FullyLit = 1,
    ShadowMap_ISM = 2,
    AllISM = 3,

    BaselineSM = 4,

    Experiment = 5
};

END_NAMESPACE_FALCOR
