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

END_NAMESPACE_FALCOR
