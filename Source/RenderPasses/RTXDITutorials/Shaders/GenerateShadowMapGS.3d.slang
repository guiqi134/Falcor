// #include "../HostDeviceSharedDefinitions.h"

import Scene.Raster;
import Utils.Debug.PixelDebug;
import ISMHelpers;

#ifndef MAX_VERTEX_COUNT
#define MAX_VERTEX_COUNT 9
#endif

cbuffer shadowMapCB
{
    uint gCurrPassReusingStartIdx;
    uint gFirstSpotLightID;
    uint gCurrNumShadowMaps;
}

Buffer<uint> gReusingLightIndexBuffer;
StructuredBuffer<LightShadowMapData> gLightShadowDataBuffer; // 0, 1, ..., Max
Buffer<uint2> gSortedLightsBuffer; // for baseline use

struct GSOut
{
    VSOut vsOut;
    uint rtIndex : SV_RenderTargetArrayIndex;
    uint triangleIndex : SV_PrimitiveID;
}

struct PSOut
{
    float color : SV_Target0;
    float depth : SV_Depth;
}

// TODO: remove all redundent computations
VSOut vsMain(VSIn vIn)
{
    VSOut vsOut;
    vsOut = defaultVS(vIn);

    return vsOut;
}

// TODO: do the culling for each primitive and decide it will be visible on which face, then only loop over the visible face (ISM)
[instance(1)]
[maxvertexcount(MAX_VERTEX_COUNT)] // Warning: this may cause bug if MAX_VERTEX_COUNT is larger than 100
void gsMain(triangle VSOut gsIn[3], uint gsInstanceID : SV_GSInstanceID, uint primitiveID : SV_PrimitiveID, inout TriangleStream<GSOut> outStream)
{
    printSetPixel(uint2(gsIn[0].instanceID.index, primitiveID));

    GeometryInstanceData instance = gScene.getGeometryInstance(gsIn[0].instanceID);
    print("isDynamic = ", instance.isDynamic());

    GSOut gsOut;

    // Ignore emissive triangles. TODO: build the VAO for non-emissive triangles
    bool isAnalyticLight = true;
    if (isAnalyticLight)
    {
        uint materialID = gScene.getMaterialID(gsIn[0].instanceID);
        if (gScene.materials.isBasicMaterial(materialID))
        {
            float16_t3 emissive = gScene.materials.getBasicMaterialData(materialID).emissive;

            // Some objects have emissive textures (e.g. the train in Subway), so we cannot just ignore them
            if (!gScene.materials.isEmissiveTextured(materialID) && any(emissive != float16_t3(0)))
                return;
        }
    }

    print("gCurrNumShadowMaps = ", gCurrNumShadowMaps);
    print("gCurrPassReusingStartIdx = ", gCurrPassReusingStartIdx);

    for (uint psmIdx = 0; psmIdx < gCurrNumShadowMaps; psmIdx++)
    {
        // If ranking update is disabled, we need to get correct global face ID from sorted buffer
        uint globalFaceID = gReusingLightIndexBuffer[gCurrPassReusingStartIdx + psmIdx];

        // Get light and face index
        uint2 localLightIDAndFaceIdx = getLocalLightIdAndFaceIdx(globalFaceID, gFirstSpotLightID);
        uint localLightID = localLightIDAndFaceIdx.x;
        uint faceIdx = localLightIDAndFaceIdx.y;

        gsOut.rtIndex = psmIdx;
        gsOut.triangleIndex = gCurrNumShadowMaps * primitiveID + psmIdx;
        float4x4 lightViewProj = mul(gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx].viewMat, gLightShadowDataBuffer[localLightID].persProjMat);

        for (uint v = 0; v < 3; v++)
        {
            gsOut.vsOut = gsIn[v];
            gsOut.vsOut.posH = mul(float4(gsIn[v].posW, 1.0f), lightViewProj);
            outStream.Append(gsOut);
        }
        outStream.RestartStrip();
    }
}

PSOut psMain(GSOut psIn)
{
    // printSetPixel(uint2(psIn.vsOut.posH.xy));

    PSOut psOut;

    let lod = ExplicitLodTextureSampler(0.f);
    if (alphaTest(psIn.vsOut, psIn.triangleIndex, lod)) discard;

    float depth = saturate(psIn.vsOut.posH.z);
    psOut.color = depth;
    psOut.depth = depth;

    print("depth = ", depth);

    return psOut;
}
