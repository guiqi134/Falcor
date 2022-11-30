#pragma once
#include "Utils/HostDeviceShared.slangh"
BEGIN_NAMESPACE_FALCOR

#ifdef HOST_CODE
#include "Falcor.h"
#endif

struct LightMeshData
{
    float3 centerPosW;
    float4x4 viewProjMat[6]; // for point light
    int startIndex;
    int shadowMapCount;
};

enum class ShadowEvaluateOption : uint
{
    ShadowMap = 0,
    FullyLit = 1,
    ShadowRay = 2,
};

END_NAMESPACE_FALCOR
