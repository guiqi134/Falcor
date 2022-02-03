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

import ReSTIR;
import Helpers;
import ShadingDataLoader;
import PCSS;

cbuffer CB
{
    uint2 gViewportDims;
    uint gFrameIndex;
    uint gInputBufferIndex;
    ResamplingRuntimeParameters gParams;
    ShadingDataLoader gShadingDataLoader;
    PCSS gPCSS;
}

static struct Data
{
    StandardMaterial standardMaterial;
} gData;

RWStructuredBuffer<Reservoir> gReservoirs;
RWTexture2D<float4> gShadingOutput;

[numthreads(16, 16, 1)]
void main(uint2 threadIndex : SV_DispatchThreadID, uint3 groupIndex : SV_GROUPID)
{
    if (any(threadIndex >= gViewportDims))
    {
        return;
    }

    printSetPixel(threadIndex);

    float3 res = 0.0;

    SampleGenerator sg = SampleGenerator.create(threadIndex, gFrameIndex);
    ShadingData sd;
    float2 poissonOffset = sampleNext2D(sg);
    bool valid = gShadingDataLoader.loadShadingData(threadIndex, gViewportDims, gScene.camera, sd);
    if (valid)
    {
        float3 posWOrigin = sd.computeNewRayOrigin();

#if !_BRUTE_FORCE
        // Reservoir Shading
        {
            Reservoir r = loadReservoir(gParams, gReservoirs, threadIndex, gInputBufferIndex);
            if (isValidReservoir(r))
            {
                uint lightIndex = getReservoirLightIndex(r);
                float3 samplePos = r.posW;

                float3 toLight = samplePos - posWOrigin;
                const float distSqr = max(FLT_MIN, dot(toLight, toLight));
                float distance = sqrt(distSqr);
                LightData light = gScene.getLight(lightIndex);
                float3 Le = light.intensity;
                
                float finalVisbility = 0.0f;
#if _SHADING_MODE == 0
                RayDesc ray;
                ray.Origin = posWOrigin;
                ray.Direction = toLight / distance;
                ray.TMin = 0.001f;
                ray.TMax = distance - 0.001f;

                if (traceSceneVisibilityRay<1>(ray, RAY_FLAG_NONE, 0xff))
                {
                    res += Le * gData.standardMaterial.eval(sd, ray.Direction) * getReservoirInvPdf(r); // f * r.W
                    finalVisbility = 1.0f;
                } 
// #elif _SHADING_MODE == 1
//                 finalVisbility = gPCSS.shadowMapPCSSModified(sd.posW, sd.N, toLight / distance, samplePos, poissonOffset);
//                 res += Le * gData.standardMaterial.eval(sd, toLight / distance) * finalVisbility * getReservoirInvPdf(r);
// #else
//                 finalVisbility = gPCSS.shadowMapPCSS(sd.posW, sd.N, toLight / distance, poissonOffset);
//                 res += Le * gData.standardMaterial.eval(sd, toLight / distance) * finalVisbility * getReservoirInvPdf(r);
#endif

#if _STORE_FINAL_VISIBILITY
                uint pointer = reservoirPositionToPointer(gParams, threadIndex, gInputBufferIndex);
                gReservoirs[pointer].weightSum = finalVisbility;
#endif
            }
        }
#else
        // Groundtruth Shading
        {
            const uint lightCount = gScene.getLightCount();
            uint lightIndex = min(uint(sampleNext1D(sg) * lightCount), lightCount - 1);
            AnalyticLightSample lightSample;
            if (sampleLight(posWOrigin, gScene.getLight(lightIndex), sg, lightSample) && dot(sd.N, lightSample.dir) >= kMinCosTheta)
            {
                RayDesc ray;
                ray.Origin = posWOrigin;
                ray.Direction = lightSample.dir;
                ray.TMin = 0.001f;
                ray.TMax = FLT_MAX;

                if (traceSceneVisibilityRay<1>(ray, RAY_FLAG_NONE, 0xff))
                {
                    res += gData.standardMaterial.eval(sd, lightSample.dir) * lightSample.Li;
                }
            }
        }
#endif
        res += sd.emissive;
    }
    else
    {
        RayDesc ray = gScene.camera.computeRayPinhole(threadIndex, gViewportDims).toRayDesc();
        res += gScene.envMap.eval(ray.Direction);
    }

    gShadingOutput[threadIndex] = float4(res, 1.0);
}
