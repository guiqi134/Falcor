// #include "../HostDeviceSharedDefinitions.h"
#include "Utils/Math/MathConstants.slangh"

import Scene.Raster;
import Utils.Debug.PixelDebug;
import ISMHelpers;

cbuffer shadowMapCB
{
    uint gFirstSpotLightID;
    uint gRank;
}

Buffer<uint> gReusingLightIndexBuffer;
StructuredBuffer<LightShadowMapData> gLightShadowDataBuffer; // 0, 1, ..., Max

struct GSOut
{
    VSOut vsOut;
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

    // Get light and face index
    uint globalFaceID = gReusingLightIndexBuffer[gRank];
    uint2 localLightIDAndFaceIdx = getLocalLightIdAndFaceIdx(globalFaceID, gFirstSpotLightID);
    uint localLightID = localLightIDAndFaceIdx.x;
    uint faceIdx = localLightIDAndFaceIdx.y;
    float4x4 lightViewProj = mul(gLightShadowDataBuffer[localLightID].lightFaceData[faceIdx].viewMat, gLightShadowDataBuffer[localLightID].persProjMat);
    vsOut.posH = mul(float4(vsOut.posW, 1.0f), lightViewProj);

    // Ignore emissive triangles. TODO: build the VAO for non-emissive triangles
    bool isAnalyticLight = true;
    if (isAnalyticLight)
    {
        GeometryInstanceData instance = gScene.getGeometryInstance(vsOut.instanceID);
        uint materialID = gScene.getMaterialID(vsOut.instanceID);
        if (gScene.materials.isBasicMaterial(materialID))
        {
            float16_t3 emissive = gScene.materials.getBasicMaterialData(materialID).emissive;

            // Some objects have emissive textures (e.g. the train in Subway), so we cannot just ignore them
            if (!gScene.materials.isEmissiveTextured(materialID) && any(emissive != float16_t3(0)))
                vsOut.posH.z = FLT_MAX;
        }
    }

    return vsOut;
}

// [maxvertexcount(3)]
// void gsMain(triangle VSOut gsIn[3], uint gsInstanceID : SV_GSInstanceID, uint primitiveID : SV_PrimitiveID, inout TriangleStream<GSOut> outStream)
// {
//     // printSetPixel(uint2(gsIn[0].instanceID.index, primitiveID));

//     // // Ignore emissive triangles. TODO: build the VAO for non-emissive triangles
//     // bool isAnalyticLight = true;
//     // if (isAnalyticLight)
//     // {
//     //     GeometryInstanceData instance = gScene.getGeometryInstance(gsIn[0].instanceID);
//     //     uint materialID = gScene.getMaterialID(gsIn[0].instanceID);
//     //     if (gScene.materials.isBasicMaterial(materialID))
//     //     {
//     //         float16_t3 emissive = gScene.materials.getBasicMaterialData(materialID).emissive;

//     //         // Some objects have emissive textures (e.g. the train in Subway), so we cannot just ignore them
//     //         if (!gScene.materials.isEmissiveTextured(materialID) && any(emissive != float16_t3(0)))
//     //             return;
//     //     }
//     // }

//     GSOut gsOut;
//     for (uint v = 0; v < 3; v++)
//     {
//         gsOut.vsOut = gsIn[v];
//         outStream.Append(gsOut);
//     }
//     outStream.RestartStrip();
// }

PSOut psMain(VSOut psIn, uint triangleIndex: SV_PrimitiveID)
{
    // printSetPixel(uint2(psIn.vsOut.posH.xy));

    PSOut psOut;

    let lod = ExplicitLodTextureSampler(0.f);
    if (alphaTest(psIn, triangleIndex, lod)) discard;

    float depth = saturate(psIn.posH.z);
    psOut.color = depth;
    psOut.depth = depth;

    print("depth = ", depth);

    return psOut;
}

// PSOut psMain(GSOut psIn)
// {
//     PSOut psOut;

//     // let lod = ExplicitLodTextureSampler(0.f);
//     // if (alphaTest(psIn.vsOut, triangleIndex, lod)) discard;

//     float depth = saturate(psIn.vsOut.posH.z);
//     psOut.color = depth;
//     psOut.depth = depth;

//     print("depth = ", depth);

//     return psOut;
// }
