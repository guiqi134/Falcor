#include "Utils/Math/MathConstants.slangh"

import Utils.Sampling.SampleGenerator;
import Utils.Math.MathHelpers;
import Utils.Color.ColorHelpers;
import Scene.RaytracingInline;
import Scene.HitInfo;
import Scene.Scene;
import Scene.Shading;
import Scene.Material.MaterialFactory;
import Rendering.Materials.StandardMaterial;
import Rendering.Lights.LightHelpers;
import Rendering.Lights.EmissiveLightSampler;
import Rendering.Lights.EmissivePowerSampler;
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
    bool gTemporalReuseBlockerDepth;
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
Texture2D<float2> gPrevBlockerDepths;
RWTexture2D<float2> gCurrBlockerDepths;
RWTexture2D<float4> gShadingOutput;

static const uint sampleLightMask = 2048;
static const int maxHistoryLength = 32;

groupshared float2 samplePool[sampleLightMask];

static struct Data
{
    StandardMaterial standardMaterial;
} gData;

[numthreads(16, 16, 1)]
void main(uint2 threadIndex : SV_DispatchThreadID, uint3 groupIndex : SV_GROUPID, uint indexInGroup : SV_GroupIndex)
{
    printSetPixel(threadIndex);

    float3 res = 0.0;

    SampleGenerator sg = SampleGenerator(threadIndex, gFrameIndex);
    SampleGenerator sgFixed = SampleGenerator(threadIndex, 0);

    ShadingData sd;
    let lod = ExplicitLodTextureSampler(.0f);
    bool valid = gShadingDataLoader.loadShadingData(threadIndex, gViewportDims, gScene.camera, sd, lod);
    let bsdf = gScene.materials.getBSDF(sd, lod);

    uint2 shadowMapSize;
    gShadowMap.GetDimensions(shadowMapSize.x, shadowMapSize.y);

    if (valid)
    {
        float3 posWOrigin = sd.computeNewRayOrigin();
        Reservoir r = loadReservoir(gParams, gReservoirs, threadIndex, gInputBufferIndex);
        BlockerSearchOutputs bso = { r.projCoords, r.shadingPosLightView };
        float bias = 0.0f;

        // Compute depth bias
        if (gUseAdaptiveDepthBias)
        {
            bias = adaptiveDepthBias(sd.N, sd.posW, shadowMapSize.x, gShadowMap, gLinearSampler, gLightParams);
        }
        else
        {
            float3 L = normalize(gLightParams.lightPos - posWOrigin);
            float NdotL = dot(sd.N, L);
            bias = _DEPTH_BIAS * saturate(1.0 - NdotL) + _DEPTH_BIAS;
        }

#if !_NO_BLOCKER_SEARCH
        // Origin blocker search
        if (!gUseNewBlockerSearch || gShadowType != (uint)ShadowType::NewPCSS)
        {
            float angle = sampleNext1D(sg) * M_2PI;
            float2 rotation = float2(cos(angle), sin(angle)); // This can turn into a LUT
            blockerSearch(bso, bias, sd.posW, rotation, gPointSampler, gShadowMap, gBlockerSearchSamples, gLightParams);
        }
        // Ours blocker search
        else
        {
            float4 lightSpacePosH = mul(float4(sd.posW, 1.0f), gLightParams.lightSpaceMat); // clipping space
            float3 lightSpacePos = lightSpacePosH.xyz / lightSpacePosH.w; // NDC
            lightSpacePos.y *= -1; // DirectX flips the Y axis in viewport scale matrix
            float3 projCoords = float3(lightSpacePos.xy * 0.5f + 0.5f, lightSpacePos.z - bias); // NDC
            float3 shadingPosLightView = mul(float4(sd.posW, 1.0f), gLightParams.lightView).xyz;
            float zReceiver = abs(shadingPosLightView.z);

            float projectionPlane = gLightParams.lightNearPlane + sampleNext1D(sg) * (zReceiver - gLightParams.lightNearPlane);
            float2 searchSize = float2(_LIGHT_WORLD_SIZE * (zReceiver - projectionPlane) / zReceiver);
            searchSize /= gLightParams.lightFrustumSize; 

            float blockerDepth = 0.0f;
            uint numBlockers = 0;
            if (gTemporalReuseBlockerDepth)
            {
                // Calculate previous frame pixel position using motion vector 
                float2 motion = gShadingDataLoader.motionVector[threadIndex];
                motion.xy += (sampleNext2D(sg) - 0.5); // remove camera jitter?
                float2 reprojectedSamplePosition = float2(threadIndex) + motion.xy;
                int2 prevPos = int2(round(reprojectedSamplePosition));
                float2 prevBlockerDepth = gPrevBlockerDepths[prevPos];

                // Map samplePos to shadow map.
                // Light front facing direction is -z, so depth is negative
                float3 randoms = sampleNext3D(sgFixed);
                uint triangleCount = gScene.lightCollection.triangleCount;
                uint triangleIndex = min((uint)(randoms.x * triangleCount), triangleCount - 1);
                const EmissiveTriangle tri = gScene.lightCollection.getTriangle(triangleIndex);
                float3 samplePosW = tri.getPosition(sample_triangle(randoms.yz));
                float3 lightDirView = mul(samplePosW - posWOrigin, float3x3(gLightParams.lightView));
                float projectionPlane = gLightParams.lightNearPlane + sampleNext1D(sg) * (zReceiver - gLightParams.lightNearPlane);
                float3 queryPosView = shadingPosLightView + lightDirView * (1.f - projectionPlane / zReceiver);
                float2 samplePosProj = 0.5 * float2(-gLightProjData.x, gLightProjData.y) * queryPosView.xy / queryPosView.z + 0.5;

                //float2 selectedSample = blockSearchDiskSamples[i] * 0.5f;
                float2 selectedSample = sampleNext1D(sg) - 0.5f;
                float2 offset = selectedSample * searchSize;

                blockerDepth = prevBlockerDepth.x;
                float sampleCount = prevBlockerDepth.y;
                sampleCount = min(maxHistoryLength, sampleCount);
                float blockerDepthSum = blockerDepth * sampleCount;
                float shadowMapDepth = gShadowMap.SampleLevel(gPointSampler, samplePosProj, 0).r;

                if (shadowMapDepth < projCoords.z)
                {
                    sampleCount++;
                    float linearDepth = abs(LinearizeDepth(shadowMapDepth, gLightParams.lightNearPlane, gLightParams.lightFarPlane));
                    blockerDepth = (blockerDepthSum + linearDepth) / sampleCount;
                }

                gCurrBlockerDepths[threadIndex] = float2(blockerDepth, sampleCount);
            }
            else
            {
                float sum = 0.0f;
                uint numBlockers = 0;
                for (uint i = 0; i < _BLOCKER_SEARCH_SAMPLES; i++)
                {
                    // Light sampling
                    float3 randoms = sampleNext3D(sg);
                    uint triangleCount = gScene.lightCollection.triangleCount;
                    uint triangleIndex = min((uint)(randoms.x * triangleCount), triangleCount - 1);
                    const EmissiveTriangle tri = gScene.lightCollection.getTriangle(triangleIndex);
                    float3 samplePosW = tri.getPosition(sample_triangle(randoms.yz));
                    float3 lightDirView = mul(samplePosW - posWOrigin, float3x3(gLightParams.lightView));

                    // Map samplePos to shadow map.
                    float projectionPlane = gLightParams.lightNearPlane + 0.0f * (zReceiver - gLightParams.lightNearPlane);
                    //projectionPlane = gLightParams.lightNearPlane;
                    float3 queryPosView = shadingPosLightView + lightDirView * (1.f - projectionPlane / zReceiver);
                    float2 samplePosProj = 0.5 * float2(-gLightProjData.x, gLightProjData.y) * queryPosView.xy / queryPosView.z + 0.5;


                    float shadowMapDepth = gShadowMap.SampleLevel(gPointSampler, samplePosProj.xy, 0).r;
                    float3 samplePosView = mul(float4(samplePosW, 1.0f), gLightParams.lightView).xyz;
                    if (shadowMapDepth < projCoords.z)
                    {
                        float linearDepth = abs(LinearizeDepth(shadowMapDepth, gLightParams.lightNearPlane, gLightParams.lightFarPlane));
                        sum += linearDepth;
                        numBlockers++;
                    }
                }

                blockerDepth = numBlockers == 0 ? blockerDepth : sum / numBlockers;
            }
            bso.projCoords = float4(projCoords, blockerDepth);
            bso.shadingPosLightView = float4(shadingPosLightView, (blockerDepth > 0 ? true : false));
        }
#endif

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
#if RTX_GPU
                    Ray ray = Ray(posWOrigin, toLight / distance, 0.001f, distance - 0.001f);
                    if (traceSceneVisibilityRay<1>(ray, RAY_FLAG_NONE, 0xff))
                    {
                        res += Le * bsdf.eval(sd, ray.dir, sg) * getReservoirInvPdf(r); // f * r.W
                        finalVisbility = 1.0f;
                    }
#endif
                }
                else if (gShadowType == (uint)ShadowType::NewPCSSReSTIR)
                {
                    float3 lightDirView = mul(samplePosW - posWOrigin, float3x3(gLightParams.lightView));
                    finalVisbility = bso.shadingPosLightView.w ? shadowMapNewPCSS(lightDirView, bso, gSamplerCmp, gShadowMap, gLightParams, gLightProjData) : 1.0f;
                    res += Le * bsdf.eval(sd, toLight / distance, sg) * finalVisbility * getReservoirInvPdf(r);
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
            uint unshadowedCount = 16;
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
                    float3 radiance = bsdf.eval(sd, tls.dir, sg) * tls.Le / tls.pdf;
                    unshadowedContrib += radiance;
                }
            }
            unshadowedContrib /= unshadowedCount;

            // Compute final contribution
            switch (gShadowType)
            {
            case ShadowType::NewPCSS:
                res = unshadowedContrib;

                [unroll]
                for (int i = 0; i < 8; i++)
                    samplePool[8 * indexInGroup + i] = gLightSampleTexture[8 * indexInGroup + i];

                GroupMemoryBarrierWithGroupSync();

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
                            float3 samplePosLightView = float3(samplePool[sampleIndex], 0.0f);
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

                break;
            case ShadowType::PCSS:
                float angle = sampleNext1D(sgFixed) * M_2PI;
                float2 rotation = float2(cos(angle), sin(angle));
                visibility = bso.shadingPosLightView.w ? shadowMapPCSS(bso, 0.3f * (sampleNext2D(sg) - 0.5f) /* translated poission */,
                    gSamplerCmp, gLinearSampler, gShadowMap, gPCFSamples, rotation, gLightParams) : 1.0f;
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
                    float2 filterSize = penumbraSize * gLightParams.lightNearPlane / zReceiver;
                    filterSize /= gLightParams.lightFrustumSize;

                    filterSize = min(filterSize, kMaxFilterWidth);

                    // Get moments from SAT
                    float2 coordsUL = projCoords.xy - 0.5f * filterSize;
                    float4 filterBound = coordsUL.xyxy + float4(0.0f, 0.0f, filterSize.xy);
                    float4 moments = sampleSAT(filterBound, shadowMapSize, depthNoBias, gSAT, gLinearSampler);

                    // If shading point depth is very close to shadow map value, degenerate to PCF
                    float shadowMapDepth = gShadowMap.SampleLevel(gLinearSampler, projCoords.xy, 0).x;
                    float difference = projCoords.z - shadowMapDepth;
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
#if RTX_GPU
            for (uint i = 0; i < _SHADOW_RAYS; i++)
            {
                TriangleLightSample tls;
                if (gEmissiveLightSampler.sampleLight(posWOrigin, sd.N, true, sg, tls) && dot(sd.N, tls.dir) >= kMinCosTheta)
                {
                    Ray ray = Ray(posWOrigin, toLight / distance, 0.001f, distance - 0.001f);
                    if (traceSceneVisibilityRay<1>(ray, RAY_FLAG_NONE, 0xff))
                    {
                        res += tls.Le * bsdf.eval(sd, tls.dir, sg) / tls.pdf;
                    }
                }
            }
            res /= _SHADOW_RAYS;
#endif
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

    res += bsdf.getProperties(sd).emission;

    gShadingOutput[threadIndex] = float4(res, 1.0);
}
