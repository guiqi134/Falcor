#pragma once
#include "Utils/HostDeviceShared.slangh"
BEGIN_NAMESPACE_FALCOR

enum class ShadowType
{
    NewPCSS, PCSS, VSM, CSM, EVSM, MSM
};

#ifdef HOST_CODE
#include "Falcor.h"
#endif

#if defined(HOST_CODE) || defined(_DEVICE_NEED_INCLUDE)


struct LightParams
{
#ifdef HOST_CODE
    void setShaderData(const ShaderVar& var)
    {
        var["lightSpaceMat"] = lightSpaceMat;
        var["lightView"] = lightView;
        var["lightProj"] = lightProj;
        var["lightPos"] = lightPos;
    }
#endif

    float4x4 lightSpaceMat;
    float4x4 lightView;
    float4x4 lightProj;
    float3 lightPos;
};

#endif

END_NAMESPACE_FALCOR
