#include "Utils/Math/MathConstants.slangh"
import Utils.Sampling.SampleGenerator;
import Utils.Math.MathHelpers;
import Utils.Color.ColorHelpers;
import Scene.RaytracingInline;
import Scene.HitInfo;
import Scene.Scene;
import Scene.Shading;
import Experimental.Scene.Material.StandardMaterial;
import Experimental.Scene.Material.MaterialHelpers;
import Experimental.Scene.Lights.LightHelpers;
import Utils.Debug.PixelDebug;

import ShadingDataLoader;
import PCSS;

cbuffer CB
{
    uint2 gViewportDims;
    uint gFrameIndex;
    ShadingDataLoader gShadingDataLoader;
    PCSS gPCSS;
}

static struct Data
{
    StandardMaterial standardMaterial;
} gData;

RWTexture2D<float4> gOutputColor;

[numthreads(16, 16, 1)]
void main(uint2 threadIndex : SV_DispatchThreadID, uint3 groupIndex : SV_GROUPID)
{
    if (any(threadIndex >= gViewportDims))
    {
        return;
    }
    printSetPixel(threadIndex);

    float3 res = 0.0f;

    SampleGenerator sg = SampleGenerator.create(threadIndex, gFrameIndex);
    ShadingData sd;
    bool valid = gShadingDataLoader.loadShadingData(threadIndex, gViewportDims, gScene.camera, sd);

    // There is a hit in visibilty buffer
    if (valid)
    {
        float3 posWOrigin = sd.computeNewRayOrigin();
        uint lightCount = gScene.getLightCount();
        BlockerSearchOutputs bso;

#if _SHADOW_OPTION == 1
        float3 L = normalize(gPCSS.gLightPos - posWOrigin);
        float NdotL = dot(sd.N, L);
        gPCSS.blockerSearch(bso, NdotL, sd.posW, sampleNext2D(sg) - 0.5f); // translated poisson
        // gPCSS.blockerSearch(bso, NdotL, sd.posW, randomRotation(sampleNext1D(sg) * M_2PI)); // rotated poisson
#endif

        // Sample area light
        for (uint i = 0; i < _LIGHT_SAMPLES; i++)
        {
            uint lightIndex = min(uint(sampleNext1D(sg) * lightCount), lightCount - 1);
            AnalyticLightSample ls;
            if (sampleLight(posWOrigin, gScene.getLight(lightIndex), sg, ls) && dot(sd.N, ls.dir) >= kMinCosTheta)
            {
                float3 toLight = ls.posW - posWOrigin;
                float distance = sqrt(max(FLT_MAX, dot(toLight, toLight)));

#if _SHADOW_OPTION == 0
                RayDesc ray;
                ray.Origin = posWOrigin;
                ray.Direction = ls.dir;
                ray.TMin = 0.001f;
                ray.TMax = distance - 0.001f;

                if (traceSceneVisibilityRay<1>(ray, RAY_FLAG_NONE, 0xff))
                {
                    res += gData.standardMaterial.eval(sd, ls.dir) * ls.Li;
                }
#else
                float visibility = bso.hasBlocker ? gPCSS.shadowMapPCSSModified(ls.posW, bso) : 1.0f;
                res += gData.standardMaterial.eval(sd, ls.dir) * ls.Li * visibility;
#endif
            }
        }

        res = res / _LIGHT_SAMPLES + sd.emissive;
    }
    else
    {
        RayDesc ray = gScene.camera.computeRayPinhole(threadIndex, gViewportDims).toRayDesc();
        res += gScene.envMap.eval(ray.Direction);
    }

    gOutputColor[threadIndex] = float4(res, 1.0f);
}
