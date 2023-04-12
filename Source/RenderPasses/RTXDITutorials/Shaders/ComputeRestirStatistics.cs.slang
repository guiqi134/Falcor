#include "RtxdiBridge.slangh"

import Utils.Debug.PixelDebug;

[numthreads(16, 16, 1)]
void main(uint3 threadID: SV_DispatchThreadID)
{
    const uint2 launchIdx = threadID.xy;
    uint bufferIndex = gScreenSize.x * launchIdx.y + launchIdx.x;
    int lightID = gPrevLightSelectionBuffer[bufferIndex];
    LightShadowMapData lightShadowData = gLightShadowDataBuffer[lightID];

    // Get surface from our G-buffer
    RAB_Surface surface = RAB_GetGBufferSurface((int2)launchIdx, false);
    bool isValidSurface = RAB_IsSurfaceValid(surface);

    if (lightID == -1 || !isValidSurface) return;

    // Project pixel's shading point to post-rotated light face
    float3 posW = RAB_GetSurfaceWorldPos(surface);
    float3 lightPosW = lightShadowData.centerPosW;
    float3 lightToSurfaceW = normalize(posW - lightPosW);

    // Compuate global face ID
    if (lightID < gFirstSpotLightID)
    {
        uint faceIdx = getFaceIndex(normalize(lightToSurfaceW));
        gPrevLightSelectionBuffer[bufferIndex] = kShadowMapsPerPointLight * lightID + faceIdx;
    }
    else
    {
        float4 posLightClipping = mul(mul(float4(posW, 1.0f), lightShadowData.lightFaceData[0].viewMat), lightShadowData.persProjMat);
        float3 posLightNDC = posLightClipping.xyz / posLightClipping.w;
        float3 ismProjCoord = float3(posLightNDC.xy * float2(0.5f, -0.5f) + 0.5f, posLightNDC.z);

        // Check if the surface point is inside the spot light view
        if (all(ismProjCoord.xy >= 0.0f) && all(ismProjCoord.xy <= 1.0f))
        {
            gPrevLightSelectionBuffer[bufferIndex] = kShadowMapsPerPointLight * gFirstSpotLightID + (lightID - gFirstSpotLightID);
        }
    }
}
