#pragma once
#include "Utils/HostDeviceShared.slangh"
BEGIN_NAMESPACE_FALCOR

#ifdef HOST_CODE
#include "Falcor.h"
#endif

struct LightShadowData
{
    float3 centerPosW; // if it is an analytic light, we can directly get the positon from gScene

    // TODO: These two can be compressed, no need to use float4x4 
    float4x4 viewMats[2];

    float4x4 persProjMat; // project down 

    //int startIndex;
};

END_NAMESPACE_FALCOR
