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

import PCSS;
import ReSTIR;
import Helpers;
import ShadingDataLoader;
import VSM;
import MSM;

cbuffer CB
{
    uint2 gViewportDims;
    uint gFrameIndex;
    uint gInputBufferIndex;
    ResamplingRuntimeParameters gParams;
    ShadingDataLoader gShadingDataLoader;
    LightParams gLightParams;
    uint gShadowType;

    SamplerComparisonState gSamplerCmp;
    SamplerState gLinearSampler;
    SamplerState gPointSampler;
}

RWStructuredBuffer<Reservoir> gReservoirs;

Texture2D gShadowMap;
Texture2D gVSM;
Texture2D gSAT;
RWTexture2D<float4> gShadingOutput;

static struct Data
{
    StandardMaterial standardMaterial;
} gData;


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
    SampleGenerator sgFixed = SampleGenerator.create(threadIndex, 0);

    ShadingData sd;
    bool valid = gShadingDataLoader.loadShadingData(threadIndex, gViewportDims, gScene.camera, sd);

    if (valid)
    {
        float3 posWOrigin = sd.computeNewRayOrigin();
        Reservoir r = loadReservoir(gParams, gReservoirs, threadIndex, gInputBufferIndex);
        BlockerSearchOutputs bso = { r.projCoords, r.shadingPosLightView };
        float bias = 0.0f;

#if _NEED_BLOCKER_SEARCH
        print(222);
        float3 L = normalize(gLightParams.lightPos - posWOrigin);
        float NdotL = dot(sd.N, L);
        bias = max(2.0f * _DEPTH_BIAS * (1.0 - NdotL), _DEPTH_BIAS);
        blockerSearch(bso, bias, sd.posW, sampleNext2D(sgFixed) - 0.5f, gLinearSampler, gShadowMap, gLightParams);
#endif
        print(bso.projCoords);
        //print(bso.shadingPosLightView);

#if !_BRUTE_FORCE
        // Reservoir Shading
        if (gShadowType == (uint)ShadowType::ShadowRay || gShadowType == (uint)ShadowType::NewPCSSReSTIR)
        {
            if (isValidReservoir(r))
            {
                uint lightIndex = getReservoirLightIndex(r);
                float3 samplePos = r.posW;

                float3 toLight = samplePos - posWOrigin;
                const float distSqr = max(FLT_MIN, dot(toLight, toLight));
                float distance = sqrt(distSqr);
                LightData light = gScene.getLight(lightIndex);
                float3 Le = light.intensity;
                
                float finalVisbility = 1.0f;
                if (gShadowType == (uint)ShadowType::ShadowRay)
                {
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
                }
                else if (gShadowType == (uint)ShadowType::NewPCSSReSTIR)
                {
                    finalVisbility = bso.shadingPosLightView.w ? shadowMapNewPCSS(samplePos, bso, gSamplerCmp, gShadowMap, gLightParams) : 1.0f;
                    res += Le * gData.standardMaterial.eval(sd, toLight / distance) * finalVisbility * getReservoirInvPdf(r);
                }

//#if _STORE_FINAL_VISIBILITY
//                uint pointer = reservoirPositionToPointer(gParams, threadIndex, gInputBufferIndex);
//                gReservoirs[pointer].weightSum = finalVisbility;
//#endif
            }
        }
        else
        {
            float visibility = 1.0f;

            // Compute unshadowed contribution or New PCSS
            const uint lightCount = gScene.getLightCount();
            uint lightIndex = min(uint(sampleNext1D(sg) * lightCount), lightCount - 1);
            float3 unshadowedContrib = 0.0f;
            float3 shadowedContrib = 0.0f;
            for (uint i = 0; i < _LIGHT_SAMPLES; i++)
            {
                AnalyticLightSample lightSample;
                if (sampleLight(posWOrigin, gScene.getLight(lightIndex), sg, lightSample) && dot(sd.N, lightSample.dir) >= kMinCosTheta)
                {
                    float3 radiance = gData.standardMaterial.eval(sd, lightSample.dir) * lightSample.Li;
                    unshadowedContrib += radiance;
                    if (gShadowType == (uint)ShadowType::NewPCSS)
                    {
                        visibility = bso.shadingPosLightView.w ? shadowMapNewPCSS(lightSample.posW, bso, gSamplerCmp, gShadowMap, gLightParams) : 1.0f;
                        shadowedContrib += radiance * visibility;
                    }
                }
            }
            unshadowedContrib /= _LIGHT_SAMPLES;

            // Compute final contribution
            switch (gShadowType)
            {
            case ShadowType::NewPCSS:
                res = shadowedContrib / _LIGHT_SAMPLES;
                break;
            case ShadowType::PCSS:
                float angle = sampleNext1D(sgFixed) * M_2PI;
                float2 rotation = float2(cos(angle), sin(angle));
                // TODO: change to fixed samples
                visibility = bso.shadingPosLightView.w ? shadowMapPCSS(bso, 0.3f * (sampleNext2D(sg) - 0.5f), gSamplerCmp, gShadowMap, rotation) : 1.0f;
                res += unshadowedContrib * visibility;
                break;
            case ShadowType::VSM:
            case ShadowType::EVSM:
            case ShadowType::MSM:
                float3 projCoords = bso.projCoords.xyz;
                print(bias);
                float depthNoBias = projCoords.z + bias;
                float avgBlockerDepth = bso.projCoords.w;
                bool hasBlocker = bso.shadingPosLightView.w;
                float zReceiver = abs(bso.shadingPosLightView.z);

                res = unshadowedContrib;
                if (hasBlocker)
                {
                    // Calculate filter size
                    float penumbraSize = _LIGHT_WORLD_SIZE * (zReceiver - avgBlockerDepth) / avgBlockerDepth; // depth in light view space
                    float2 filterSize = float2(penumbraSize * _LIGHT_NEAR_PLANE / zReceiver);
                    filterSize /= _LIGHT_FRUSTUM_SIZE;

                    float2 shadowMapSize;
                    gSAT.GetDimensions(shadowMapSize.x, shadowMapSize.y);
                    print(shadowMapSize);
                    float2 texelSize = 1 / shadowMapSize;
                    filterSize = clamp(filterSize, kMinFilterWidth, kMaxFilterWidth);

                    // Get filter two bounding points
                    float2 coordsUL = projCoords.xy - 0.5f * filterSize;
                    float4 filterBound = coordsUL.xyxy + float4(0.0f, 0.0f, filterSize.xy);
                    float4 moments = sampleSAT(filterBound, shadowMapSize, depthNoBias, gSAT, gLinearSampler);
                    print(moments);

                    if (gShadowType == (uint)ShadowType::VSM)
                        visibility = vsmShadowContribSAT(moments, depthNoBias);
                    else if (gShadowType == (uint)ShadowType::EVSM)
                        visibility = evsmShadowContribSAT(moments, depthNoBias);
                    else
                    {
                        visibility = computeHamburger4MSM(moments, depthNoBias, 3e-5) / 0.98f;
                        //visibility = computeHausdorff4MSM(moments, depthNoBias, 3e-5);
                        visibility = LBR(visibility);
                    }

                    
                }
                res = unshadowedContrib * visibility;
                break;
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

    // Check for NaN and Inf
    if (any(isnan(res) || isinf(res))) res = 0.f;

    gShadingOutput[threadIndex] = float4(res, 1.0);
}
