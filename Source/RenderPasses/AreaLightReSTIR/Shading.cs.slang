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
import Experimental.Scene.Lights.EmissiveLightSampler;
import Utils.Debug.PixelDebug;

__exported import PCSS;
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
    bool gPrecomputeLightSamples;
    bool gUseNewBlockerSearch;
    bool gUseAdaptiveDepthBias;
    float2 gLightProjData;

    SamplerComparisonState gSamplerCmp;
    SamplerState gLinearSampler;
    SamplerState gPointSampler;
    EmissiveLightSampler gEmissiveLightSampler;
}

Texture2D gShadowMap;
Texture2D gVSM;
Texture2D gSAT;
Texture1D<float2> gLightSampleTexture;
Texture1D<float2> gBlockerSearchSamples;
Texture1D<float2> gPCFSamples;

RWStructuredBuffer<Reservoir> gReservoirs;
RWTexture2D<float4> gShadingOutput;

static const uint sampleLightMask = 2048;

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

    uint2 shadowMapSize;
    gShadowMap.GetDimensions(shadowMapSize.x, shadowMapSize.y);

    if (valid)
    {
        float3 posWOrigin = sd.computeNewRayOrigin();
        Reservoir r = loadReservoir(gParams, gReservoirs, threadIndex, gInputBufferIndex);
        BlockerSearchOutputs bso = { r.projCoords, r.shadingPosLightView };
        float bias = 0.0f;

#if _NEED_BLOCKER_SEARCH
        if (gUseAdaptiveDepthBias)
        {
            bias = adaptiveDepthBias(sd.N, sd.posW, shadowMapSize.x, gShadowMap, gLinearSampler, gLightParams);
        }
        else
        {
            float3 L = normalize(gLightParams.lightPos - posWOrigin);
            float NdotL = dot(sd.N, L);
            bias = _DEPTH_BIAS * saturate(1.0 - NdotL) + _DEPTH_BIAS;
            print(bias);
        }

        if (!gUseNewBlockerSearch || gShadowType != (uint)ShadowType::NewPCSS)
        {
            print(111);
            float angle = sampleNext1D(sgFixed) * M_2PI;
            float2 rotation = float2(cos(angle), sin(angle)); // This can turn into a LUT
            blockerSearch(bso, bias, sd.posW, rotation, gLinearSampler, gShadowMap, gBlockerSearchSamples, gLightParams);
        }
        else
        {
            print(222);
            blockerSearchNewPCSS(bso, bias, sd.posW, sgFixed, gLinearSampler, gShadowMap, gLightParams);
        }
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
                float3 samplePosW = r.posW;

                float3 toLight = samplePosW - posWOrigin;
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
                    float3 lightDirView = mul(samplePosW - posWOrigin, float3x3(gLightParams.lightView));
                    finalVisbility = bso.shadingPosLightView.w ? shadowMapNewPCSS(lightDirView, bso, gSamplerCmp, gShadowMap, gLightParams, gLightProjData) : 1.0f;
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
            //const uint lightCount = gScene.getLightCount();
            //uint lightIndex = min(uint(sampleNext1D(sg) * lightCount), lightCount - 1);
            float3 unshadowedContrib = 0.0f;
            float3 shadowedContrib = 0.0f;
            uint unshadowedCount = 2;
            for (uint i = 0; i < unshadowedCount; i++)
            {
                //AnalyticLightSample lightSample;
                //if (sampleLight(posWOrigin, gScene.getLight(lightIndex), sg, lightSample) && dot(sd.N, lightSample.dir) >= kMinCosTheta)
                //{
                //    float3 radiance = gData.standardMaterial.eval(sd, lightSample.dir) * lightSample.Li;
                //    unshadowedContrib += radiance;
                //    if (gShadowType == (uint)ShadowType::NewPCSS)
                //    {
                //        visibility = bso.shadingPosLightView.w ? shadowMapNewPCSS(lightSample.posW, bso, gSamplerCmp, gShadowMap, gLightParams) : 1.0f;
                //        shadowedContrib += radiance * visibility;
                //    }
                //}

                TriangleLightSample tls;
                if (gEmissiveLightSampler.sampleLight(posWOrigin, sd.N, true, sg, tls) && dot(sd.N, tls.dir) >= kMinCosTheta)
                {
                    float3 radiance = gData.standardMaterial.eval(sd, tls.dir) * tls.Le / tls.pdf;
                    unshadowedContrib += radiance;
                }
            }
            unshadowedContrib /= unshadowedCount;

            // Compute final contribution
            switch (gShadowType)
            {
            case ShadowType::NewPCSS:
                res = unshadowedContrib;

                if (bso.shadingPosLightView.w)
                {
                    float sum = 0.0f;
                    uint startIndex = sampleNext1D(sg) * sampleLightMask;
                    for (uint i = 0; i < _LIGHT_SAMPLES; i++)
                    {
                        // Analytic light sampling
                        //AnalyticLightSample lightSample;
                        //if (sampleLight(posWOrigin, gScene.getLight(lightIndex), sg, lightSample) && dot(sd.N, lightSample.dir) >= kMinCosTheta)
                        //    sum += bso.shadingPosLightView.w ? shadowMapNewPCSS(lightSample.posW, bso, gSamplerCmp, gShadowMap, gLightParams) : 1.0f;

                        // Emissive light sampling
                        float3 lightDirView;
                        if (gPrecomputeLightSamples)
                        {
                            uint sampleIndex = (startIndex + i) % sampleLightMask;
                            float3 samplePosLightView = float3(gLightSampleTexture[sampleIndex], 0.0f);
                            lightDirView = samplePosLightView - bso.shadingPosLightView.xyz;
                        }
                        else
                        {
                            float3 randoms = sampleNext3D(sg);
                            uint triangleCount = gScene.lightCollection.triangleCount;
                            uint triangleIndex = min((uint)(randoms.x * triangleCount), triangleCount - 1);
                            const EmissiveTriangle tri = gScene.lightCollection.getTriangle(triangleIndex);
                            float3 samplePosW = tri.getPosition(sample_triangle(randoms.yz));
                            lightDirView = mul(samplePosW - posWOrigin, float3x3(gLightParams.lightView));
                        }

                        sum += shadowMapNewPCSS(lightDirView, bso, gSamplerCmp, gShadowMap, gLightParams, gLightProjData);
                    }
                    res = unshadowedContrib * sum / _LIGHT_SAMPLES;
                }

                //float sum = 0.0f;
                //for (uint i = 0; i < _LIGHT_SAMPLES; i++)
                //{
                //    float3 randoms = sampleNext3D(sg);
                //    uint triangleCount = gScene.lightCollection.triangleCount;
                //    uint triangleIndex = min((uint)(randoms.x * triangleCount), triangleCount - 1);
                //    const EmissiveTriangle tri = gScene.lightCollection.getTriangle(triangleIndex);
                //    float3 samplePosW = tri.getPosition(sample_triangle(randoms.yz));
                //    float3 lightDirView = mul(samplePosW - posWOrigin, float3x3(gLightParams.lightView));
                //    sum += bso.shadingPosLightView.w ? shadowMapNewPCSS(lightDirView, bso, gSamplerCmp, gShadowMap, gLightParams, gLightProjData) : 1.0;
                //}
                //res = unshadowedContrib * sum / _LIGHT_SAMPLES;

                break;
            case ShadowType::PCSS:
                float angle = sampleNext1D(sgFixed) * M_2PI;
                float2 rotation = float2(cos(angle), sin(angle));
                visibility = bso.shadingPosLightView.w ? shadowMapPCSS(bso, 0.3f * (sampleNext2D(sg) - 0.5f) /* translated poission */,
                    gSamplerCmp, gLinearSampler, gShadowMap, gPCFSamples, rotation) : 1.0f;
                res += unshadowedContrib * visibility;
                break;
            case ShadowType::VSM:
            case ShadowType::EVSM:
            case ShadowType::MSM:
                float3 projCoords = bso.projCoords.xyz;
                float depthNoBias = projCoords.z + bias;
                float avgBlockerDepth = bso.projCoords.w;
                bool hasBlocker = bso.shadingPosLightView.w;
                float zReceiver = abs(bso.shadingPosLightView.z);

                res = unshadowedContrib;
                float2 texelSize = 1 / shadowMapSize;

                // In order to correctly sample SAT, we need to handle edge case when coordinate is outside the texture
                if (hasBlocker && all(projCoords.xy > float2(0.0f)) && all(projCoords.xy < float2(1.0f)))
                {
                    // Calculate filter size
                    float2 penumbraSize = _LIGHT_WORLD_SIZE * (zReceiver - avgBlockerDepth) / avgBlockerDepth; // depth in light view space
                    float2 filterSize = penumbraSize * _LIGHT_NEAR_PLANE / zReceiver;
                    filterSize /= _LIGHT_FRUSTUM_SIZE;

                    filterSize = min(filterSize, kMaxFilterWidth);
                    print(filterSize);

                    // Get moments from SAT
                    float2 coordsUL = projCoords.xy - 0.5f * filterSize;
                    float4 filterBound = coordsUL.xyxy + float4(0.0f, 0.0f, filterSize.xy);
                    float4 moments = sampleSAT(filterBound, shadowMapSize, depthNoBias, gSAT, gLinearSampler);
                    print(moments);

                    // If shading point depth is very close to shadow map value, degenerate to PCF
                    float shadowMapDepth = gShadowMap.SampleLevel(gLinearSampler, projCoords.xy, 0).x;
                    float difference = projCoords.z - shadowMapDepth;
                    print(difference);
                    bool checkFilterSize = all(filterSize <= _FILTER_SIZE_THRESHOLD);
                    if (difference <= _DEPTH_DIFFERENCE && checkFilterSize)
                    {
                        res = float3(1.0f, 0.0f, 0.0f);
                        float sum = 0.0f;
                        const uint PCFSampleCount = 64;

                        for (int i = 0; i < PCFSampleCount; i++)
                        {
                            // Poisson Rectangle
                            float2 selectedSample = POISSON_RECT_SAMPLES_64[i] - 0.5f;
                            float2 offset = selectedSample * filterSize;
                            sum += gShadowMap.SampleCmpLevelZero(gSamplerCmp, projCoords.xy + offset, projCoords.z);
                        }
                        visibility = saturate(sum / PCFSampleCount);
                    }
                    else
                    {
                        res = float3(0.0f, 1.0f, 0.0f);

                        if (gShadowType == (uint)ShadowType::VSM)
                            visibility = vsmShadowContribSAT(moments, depthNoBias);
                        else if (gShadowType == (uint)ShadowType::EVSM)
                            visibility = evsmShadowContribSAT(moments, depthNoBias);
                        else
                        {
                            visibility = computeHamburger4MSM(moments, depthNoBias, 5e-4) / 0.98f;
                            //visibility = computeHausdorff4MSM(moments, depthNoBias, 3e-5) / 0.98f;
                            visibility = LBR(visibility);
                        }
                    }

                    //print(visibility);
                }
                res = unshadowedContrib * visibility;
                break;
            }
        }
#else
        // Groundtruth Shading
        {
            //const uint lightCount = gScene.getLightCount();
            //uint lightIndex = min(uint(sampleNext1D(sg) * lightCount), lightCount - 1);
            //AnalyticLightSample lightSample;
            //if (sampleLight(posWOrigin, gScene.getLight(lightIndex), sg, lightSample) && dot(sd.N, lightSample.dir) >= kMinCosTheta)
            //{
            //    RayDesc ray;
            //    ray.Origin = posWOrigin;
            //    ray.Direction = lightSample.dir;
            //    ray.TMin = 0.001f;
            //    ray.TMax = FLT_MAX;

            //    if (traceSceneVisibilityRay<1>(ray, RAY_FLAG_NONE, 0xff))
            //    {
            //        res += gData.standardMaterial.eval(sd, lightSample.dir) * lightSample.Li;
            //    }
            //}

            for (uint i = 0; i < _SHADOW_RAYS; i++)
            {
                TriangleLightSample tls;
                if (gEmissiveLightSampler.sampleLight(posWOrigin, sd.N, true, sg, tls) && dot(sd.N, tls.dir) >= kMinCosTheta)
                {
                    RayDesc ray;
                    ray.Origin = posWOrigin;
                    ray.Direction = tls.dir;
                    ray.TMin = 0.001f;
                    ray.TMax = tls.distance - 0.001f;
                    if (traceSceneVisibilityRay<1>(ray, RAY_FLAG_NONE, 0xff))
                    {
                        res += tls.Le * gData.standardMaterial.eval(sd, tls.dir) / tls.pdf;
                    }
                }
            }
            res /= _SHADOW_RAYS;
        }
#endif
    }
    else
    {
        RayDesc ray = gScene.camera.computeRayPinhole(threadIndex, gViewportDims).toRayDesc();
        res += gScene.envMap.eval(ray.Direction);
    }

    // Check for NaN and Inf
    if (any(isnan(res) || isinf(res))) res = 0.f;

    res += sd.emissive;

    gShadingOutput[threadIndex] = float4(res, 1.0);
}
