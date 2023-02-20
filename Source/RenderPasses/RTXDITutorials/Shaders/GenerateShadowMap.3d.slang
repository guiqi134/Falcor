import Scene.Raster;
import Utils.Debug.PixelDebug;

#include "../HostDeviceSharedDefinitions.h"

#ifndef MAX_VERTEX_COUNT
#define MAX_VERTEX_COUNT 18
#endif

cbuffer shadowMapCB
{
    uint gCurrPassReusingStartIdx;
    uint gVisMode;
    float gDepthBias;
    uint gShadowMapsPerLight;
    uint gSortingRules;
    uint gTemporalReusingFix;
    bool gHasDynamicObjects;
    uint gShadowMapSize;
}

Buffer<uint> gReusingLightIndexBuffer;
StructuredBuffer<LightShadowMapData> gLightShadowDataBuffer; // 0, 1, ..., Max
Buffer<uint2> gSortedLightsBuffer; // for baseline use

struct GSOut
{
    VSOut vsOut;
    uint rtIndex : SV_RenderTargetArrayIndex;
    uint triangleIndex : SV_PrimitiveID;

    linear float4 prevPosLightH : PrevPosLightH;
    float4 posLightH : PosLightH;
}

struct PSOut
{
    float color : SV_Target0;
    float4 mvec : SV_Target1;
    float depth : SV_Depth;
}

// TODO: remove all redundent computations
VSOut vsMain(VSIn vIn)
{
    VSOut vsOut;
    vsOut = defaultVS(vIn);
    printSetPixel(uint2(vsOut.instanceID.index, vIn.vertexID));

    if (gHasDynamicObjects && gTemporalReusingFix == (uint)TemporalReusingFix::MotionVector)
    {
        GeometryInstanceData instance = gScene.getGeometryInstance(vsOut.instanceID);
        uint prevVertexIndex = gScene.meshes[instance.geometryIndex].prevVbOffset + vIn.vertexID;
        float3 prevPos = gScene.prevVertices[prevVertexIndex].position;
        vsOut.prevPosW = mul(float4(prevPos, 1.f), gScene.getPrevWorldMatrix(vsOut.instanceID));
        // vsOut.prevPosW = mul(float4(vIn.pos, 1.f), gScene.getPrevWorldMatrix(vsOut.instanceID));

        // print("pos = ", vIn.pos);
        // print("prevPos = ", prevPos);
        // print("posW = ", vsOut.posW);
        // print("prevPosW = ", vsOut.prevPosW);

        for (uint i = 0; i < 4; i++)
        {
            print("prevWorldMatrix = ", gScene.getPrevWorldMatrix(vsOut.instanceID)[i]);
        }
        for (uint i = 0; i < 4; i++)
        {
            print("worldMatrix = ", gScene.getWorldMatrix(vsOut.instanceID)[i]);
        }
    }

    return vsOut;
}

// TODO: do the culling for each primitive and decide it will be visible on which face, then only loop over the visible face (ISM)
[instance(1)]
[maxvertexcount(MAX_VERTEX_COUNT)] // Warning: this may cause bug if MAX_VERTEX_COUNT is larger than 100
void gsMain(triangle VSOut gsIn[3], uint gsInstanceID : SV_GSInstanceID, uint primitiveID : SV_PrimitiveID, inout TriangleStream<GSOut> outStream)
{
    // printSetPixel(uint2(gsIn[0].instanceID.index, primitiveID));

    GeometryInstanceData instance = gScene.getGeometryInstance(gsIn[0].instanceID);
    print("isDynamic = ", instance.isDynamic());

    GSOut gsOut;

    for (uint psmIdx = 0; psmIdx < gShadowMapsPerLight; psmIdx++)
    {
        // Get light and face index
        uint offset = gSortingRules == (uint)SortingRules::LightFaces ? psmIdx : 0u;
        uint lightOrGlobalFaceIdx = gReusingLightIndexBuffer[gCurrPassReusingStartIdx + offset];
        uint localLightIdx = gSortingRules == (uint)SortingRules::LightFaces ? lightOrGlobalFaceIdx / gShadowMapsPerLight : lightOrGlobalFaceIdx;
        uint faceIdx = gSortingRules == (uint)SortingRules::LightFaces ? lightOrGlobalFaceIdx % gShadowMapsPerLight : psmIdx;

        // print("newPsmLightFace = ", gLightShadowDataBuffer[localLightIdx].lightFaceData[faceIdx].newPsmLightFace);

        // Check if the face is a new PSM light face, so we need to render static objects for it
        if (gTemporalReusingFix != (uint)TemporalReusingFix::None && !gHasDynamicObjects && !gLightShadowDataBuffer[localLightIdx].lightFaceData[faceIdx].newPsmLightFace)
            continue;

        // Discard primitive if it is on the light itself & it is a true emissive triangle
        uint currLightIntanceID = gScene.lightCollection.meshData[localLightIdx].instanceID;
        uint emissiveTriIndex = gScene.lightCollection.getTriangleIndex(gsIn[0].instanceID, primitiveID);
        // if (currLightIntanceID == gsIn[0].instanceID.index && any(gScene.lightCollection.getAverageRadiance(emissiveTriIndex) != float3(0.0f)))
        if (currLightIntanceID == gsIn[0].instanceID.index)
            continue;

        gsOut.rtIndex = psmIdx;
        gsOut.triangleIndex = gShadowMapsPerLight * primitiveID + psmIdx;
        float4x4 lightViewProj = mul(gLightShadowDataBuffer[localLightIdx].lightFaceData[faceIdx].viewMat, gLightShadowDataBuffer[localLightIdx].persProjMat);
        float4x4 prevLightViewProj = mul(gLightShadowDataBuffer[localLightIdx].lightFaceData[faceIdx].prevViewMat, gLightShadowDataBuffer[localLightIdx].persProjMat);

        for (uint v = 0; v < 3; v++)
        {
            gsOut.vsOut = gsIn[v];
            gsOut.vsOut.posH = mul(float4(gsIn[v].posW, 1.0f), lightViewProj);
            gsOut.posLightH = mul(float4(gsIn[v].posW, 1.0f), lightViewProj);
            if (gHasDynamicObjects && gTemporalReusingFix == (uint)TemporalReusingFix::MotionVector)
                gsOut.prevPosLightH = mul(gsIn[v].prevPosW, prevLightViewProj);
            outStream.Append(gsOut);
        }
        outStream.RestartStrip();
    }
}

/** D3DXMatrixPerspectiveFovRH (be care, it is row major, so it is multiplied in right!!!):
    xScale     0          0              0
    0        yScale       0              0
    0          0      zf/(zn-zf)        -1
    0          0      zn*zf/(zn-zf)      0

    Transform from NDC to camera space
*/
float linearizeDepth(float depthNDC, float near, float far)
{
    return (near * far) / (depthNDC * (far - near) - far);
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
    psOut.mvec = float4(0.0f, 0.0f, 0.0f, 1.0f);

    // Compute motion vector for dynamic objects
    if (gHasDynamicObjects && gTemporalReusingFix == (uint)TemporalReusingFix::MotionVector)
    {
        int2 ipos = int2(psIn.vsOut.posH.xy);          // The integer value of pixel position on shadow map
        float2 pixelPos = float2(ipos) + float2(0.5f); // Get the center of this pixel

        print("prevPosW = ", psIn.vsOut.prevPosW);
        print("posW = ", psIn.vsOut.posW);

        // Project motion vector to light space
        float3 prevCrd = psIn.prevPosLightH.xyz / psIn.prevPosLightH.w;
        prevCrd.xy *= float2(0.5, -0.5);
        prevCrd.xy += 0.5f;
        float2 normalizedCrd = pixelPos / gShadowMapSize;
        psOut.mvec = float4(float3(normalizedCrd, depth) - prevCrd, 1.0f); // Should we use view space depth?

        print("prevPosH = ", psIn.prevPosLightH);
        print("posH = ", psIn.posLightH);

        print("mvec = ", psOut.mvec);
    }


    return psOut;
}
