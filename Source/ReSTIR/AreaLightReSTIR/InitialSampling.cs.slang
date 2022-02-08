#include "Utils/Math/MathConstants.slangh"
import Utils.Sampling.SampleGenerator;
import Utils.Math.MathHelpers;
import Utils.Color.ColorHelpers;
import Scene.RaytracingInline;
import Scene.Scene;
import Scene.Shading;
import Experimental.Scene.Material.StandardMaterial;
import Experimental.Scene.Material.MaterialHelpers;
import Experimental.Scene.Lights.LightHelpers;
import Utils.Debug.PixelDebug;

import ReSTIR;
import ShadingDataLoader;
import Helpers;
import PCSS;

cbuffer CB
{
    uint2 gViewportDims;
    uint gFrameIndex;
    uint gOutputBufferIndex;
    ResamplingRuntimeParameters gParams;
    ShadingDataLoader gShadingDataLoader;
    PCSS gPCSS;
}

RWStructuredBuffer<Reservoir> gReservoirs;

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

    SampleGenerator sg = SampleGenerator.create(threadIndex, gFrameIndex + 1 * 13);
    Reservoir r = emptyReservoir();

    // Prepare shading data from vBuffer
    ShadingData sd;
    bool valid = gShadingDataLoader.loadShadingData(threadIndex, gViewportDims, gScene.camera, sd);

    if (valid)
    {
        float3 posWOrigin = sd.computeNewRayOrigin();

        // Compute average blocker depth (how can we use slope-scale bias here?)
        // TODO: compute only once in this shader and pass it to other shaders
        #if _TARGET_PDF == 1
            float avgBlockerDepth, zReceiver;
            float2 minMaxBlockerDepth;
            float3 projCoords, lightSpaceView;
            
            float3 L = normalize(gPCSS.gLightPos - posWOrigin);
            float NdotL = dot(sd.N, L);
            bool hasBlocker = gPCSS.blockerSearch(avgBlockerDepth, minMaxBlockerDepth, projCoords, zReceiver, lightSpaceView, 
                NdotL, sd.posW, sampleNext2D(sg));

            print("avgBlockerDepth = ", avgBlockerDepth);
            print("minMaxBlockerDepth = ", minMaxBlockerDepth);
        #endif

        // Sampling analytic area light
        Reservoir areaLightReservoir = emptyReservoir();
        for (uint i = 0; i < _INITIAL_AREA_LIGHT_SAMPLES; i++)
        {
            const uint lightCount = gScene.getLightCount();
            if (lightCount == 0) break;

            AnalyticLightSample lightSample;
            uint lightIndex = min(uint(sampleNext1D(sg) * lightCount), lightCount - 1);
            if (sampleLight(posWOrigin, gScene.getLight(lightIndex), sg, lightSample) && dot(sd.N, lightSample.dir) >= kMinCosTheta)
            {
                float lightSelectionPdf = 1.0f / lightCount;
                float invPdf = 1.0f / (lightSample.pdf * lightSelectionPdf);
                float targetPdf = luminance(lightSample.Li * lightSample.pdf * gData.standardMaterial.eval(sd, lightSample.dir));

                // Add visibility to target pdf using PCSS -> more overhead
                float3 toLight = lightSample.posW - posWOrigin; // unnormalized
                float distance = sqrt(max(FLT_MIN, dot(toLight, toLight)));
                float visibility = 1.0f;

                // _TARGET_PDF == 0 -> unshadowed 
                #if _TARGET_PDF == 1 // Shadow Map
                    if (hasBlocker)
                    {
                        float sum = 0.0f;
                        for (int j = 0; j < _NEW_PCSS_SAMPLES; j++)
                        {
                            avgBlockerDepth = _NEW_PCSS_SAMPLES == 1 ? avgBlockerDepth 
                                : lerp(minMaxBlockerDepth.x, minMaxBlockerDepth.y, 1 - sampleNext1D(sg));
                            sum += gPCSS.shadowMapPCSSModified(lightSample.posW, avgBlockerDepth, projCoords, zReceiver,
                                lightSpaceView);
                        }
                        visibility = max(_MIN_VISIBILITY, sum / _NEW_PCSS_SAMPLES);
                    }

                    // visibility = hasBlocker ? gPCSS.shadowMapPCSSModified(lightSample.posW, avgBlockerDepth, projCoords, zReceiver, 
                    //     lightSpaceView) : 1.0f;

                    print("visibility = ", visibility);

                #elif _TARGET_PDF == 2 // Shadow Ray
                    RayDesc ray;
                    ray.Origin = posWOrigin;
                    ray.Direction = toLight / distance;
                    ray.TMin = 0.001f;
                    ray.TMax = distance - 0.001f;

                    visibility = traceSceneVisibilityRay<1>(ray, RAY_FLAG_NONE, 0xff);
                #endif

                targetPdf *= visibility;
                streamSample(areaLightReservoir, lightIndex, lightSample.posW, sampleNext1D(sg), targetPdf, invPdf);
            }
            else
            {
                areaLightReservoir.M += 1;
            }
        }
        finalizeResampling(areaLightReservoir, 1.0, areaLightReservoir.M);

        // Pass area Light reservoir to current pixel reservoir
        // areaLightReservoir.M = 1; // Q: why do this? In order to pass weightSum to Reservoir r
        // combineReservoirs(r, areaLightReservoir, sampleNext1D(sg), areaLightReservoir.targetPdf);
        // finalizeResampling(r, 1.0, 1.0);
        // r.M = 1;
        r = areaLightReservoir;

        // If no reuse, no need to run visibility reuse
        #if _HAS_REUSING
            if (isValidReservoir(r))
            {
                uint lightIndex = getReservoirLightIndex(r);
                float3 samplePos = r.posW;
                float3 toLight = samplePos - posWOrigin; // unnormalized
                float distSqr = max(FLT_MIN, dot(toLight, toLight));
                float distance = sqrt(distSqr);

                // Trace shadow ray for selected light sample
                #if _TARGET_PDF == 0
                    RayDesc ray;
                    ray.Origin = posWOrigin;
                    ray.Direction = toLight / distance;
                    ray.TMin = 0.001f;
                    ray.TMax = distance - 0.001f;

                    // bool traceSceneVisibilityRay<let UseAlphaTest : int>(const RayDesc ray, uint rayFlags, uint instanceInclusionMask)
                    if (!traceSceneVisibilityRay<1>(ray, RAY_FLAG_NONE, 0xff)) 
                    {
                        r.lightData = 0;
                        r.weightSum = 0; // r.W
                    }
                #endif
            }
        #endif
    }

    print("r.W = ", r.weightSum);

    storeReservoir(gParams, r, gReservoirs, threadIndex, gOutputBufferIndex);
}
