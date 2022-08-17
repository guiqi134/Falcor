import Scene.Raster;
import Utils.Debug.PixelDebug;

#include "../HostDeviceSharedDefinitions.h"

cbuffer shadowMapCB
{
    uint ranking;
    uint face;
    float gLightNear;
    float gLightFar;
}

Buffer<uint2> gSortedLightMeshBuffer;
StructuredBuffer<LightMeshData> gLightMeshDataBuffer;

// struct ShadowVSOut
// {
//     float2 texCoords : TEXCRD;
//     float4 posCS : PosCameraSpace; // TODO: this can be further reduce to only z component

//     nointerpolation GeometryInstanceID instanceID : INSTANCE_ID;
//     nointerpolation uint materialID : MATERIAL_ID;

//     float4 posH : SV_Position;
// }

struct PSOut
{
    float4 color : SV_Target0;
    float depth : SV_Depth;
}

VSOut vsMain(VSIn vIn)
{
    VSOut vsOut;
    vsOut = defaultVS(vIn);

    // Get current light mesh's data
    uint lightMeshIndex = gSortedLightMeshBuffer[ranking].x;
    float4x4 lightViewProj = gLightMeshDataBuffer[lightMeshIndex].viewProjMat[face];

    float3 posW = vsOut.posW;
    vsOut.posH = mul(float4(posW.xyz, 1.0f), lightViewProj);

    return vsOut;
}

// Note: check for SV_RenderTargetArrayIndex Semantic
void gsMain()
{

}

/** D3DXMatrixPerspectiveFovRH (be care, it is row major!!!):
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

PSOut psMain(VSOut vsOut, uint triangleIndex : SV_PrimitiveID)
{
    printSetPixel(vsOut.posH.xy);

    PSOut psOut;

    let lod = ImplicitLodTextureSampler();
    if (alphaTest(vsOut, triangleIndex, lod)) discard;

    // Note: need to ignore the emissive surface's fragment


    float depth = saturate(vsOut.posH.z);

    float depthCS = linearizeDepth(depth, gLightNear, gLightFar);
    float4 debugColor = float4(float3(abs(depthCS) / 5.0f), 1.0f);

    print("face = ", face);
    print("depth = ", depth);
    print("depthCS = ", depthCS);
    uint lightMeshIndex = gSortedLightMeshBuffer[ranking].x;
    float4x4 lightViewProj = gLightMeshDataBuffer[lightMeshIndex].viewProjMat[face];
    print("lightViewProj = ", lightViewProj[0]);
    print("lightViewProj = ", lightViewProj[1]);
    print("lightViewProj = ", lightViewProj[2]);
    print("lightViewProj = ", lightViewProj[3]);

    psOut.color = debugColor;
    psOut.depth = depth;

    return psOut;
}