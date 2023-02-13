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
    GSOut gsOut;
    // uint psmIdx = gsInstanceID;
    for (uint psmIdx = 0; psmIdx < gShadowMapsPerLight; psmIdx++)
    {
        // Get light and face index
        uint offset = gSortingRules == (uint)SortingRules::LightFaces ? psmIdx : 0u;
        uint lightOrGlobalFaceIdx = gReusingLightIndexBuffer[gCurrPassReusingStartIdx + offset];
        uint localLightIdx = gSortingRules == (uint)SortingRules::LightFaces ? lightOrGlobalFaceIdx / gShadowMapsPerLight : lightOrGlobalFaceIdx;
        uint faceIdx = gSortingRules == (uint)SortingRules::LightFaces ? lightOrGlobalFaceIdx % gShadowMapsPerLight : psmIdx;

        // Discard primitive if it is on the light itself & it is a true emissive triangle
        uint currLightIntanceID = gScene.lightCollection.meshData[localLightIdx].instanceID;
        uint emissiveTriIndex = gScene.lightCollection.getTriangleIndex(gsIn[0].instanceID, primitiveID);
        // if (currLightIntanceID == gsIn[0].instanceID.index && any(gScene.lightCollection.getAverageRadiance(emissiveTriIndex) != float3(0.0f)))
        if (currLightIntanceID == gsIn[0].instanceID.index)
            continue;

        gsOut.rtIndex = psmIdx;
        gsOut.triangleIndex = gShadowMapsPerLight * primitiveID + psmIdx;
        float4x4 lightViewProj = mul(gLightShadowDataBuffer[localLightIdx].lightFaceData[faceIdx].viewMat, gLightShadowDataBuffer[localLightIdx].persProjMat);

        for (uint v = 0; v < 3; v++)
        {
            gsOut.vsOut = gsIn[v];
            gsOut.vsOut.posH = mul(float4(gsIn[v].posW, 1.0f), lightViewProj);
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

    return psOut;
}
