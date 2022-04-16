#pragma once
#include "Utils/HostDeviceShared.slangh"
BEGIN_NAMESPACE_FALCOR

enum class ShadowType
{
    ShadowRay, NewPCSSReSTIR, NewPCSS, PCSS, VSM, EVSM, MSM,
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
        var["lightNearPlane"] = lightNearPlane;
        var["lightFarPlane"] = lightFarPlane;
        var["lightFrustumSize"] = lightFrustumSize;
    }
#endif

    float4x4 lightSpaceMat;
    float4x4 lightView;
    float4x4 lightProj;
    float3 lightPos;
    float lightNearPlane;
    float lightFarPlane;
    float lightFrustumSize;
};

// This per-pixel data should be passed between shaders
struct BlockerSearchOutputs
{
    float4 projCoords; // projCoords + avgBlockerDepth
    float4 shadingPosLightView; // shadingPosLightView + hasBlocker
    // float2 poissonOffset; // This is only for origin PCSS, can be removed later
};

#endif

END_NAMESPACE_FALCOR
