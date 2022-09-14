import Scene.Raster;
import Utils.Debug.PixelDebug;

#include "../HostDeviceSharedDefinitions.h"

cbuffer shadowMapCB
{
    uint gRanking;
    uint gCurrFrameLightStartIdx;
    float gLightNear;
    float gLightFar;
}

Buffer<uint> gReusingLightIndexBuffer;
StructuredBuffer<LightMeshData> gLightMeshDataBuffer; // 0, 1, ..., Max

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

// Must use built-in instance to represent each render target 
[instance(1)]
[maxvertexcount(3)]
void gsMain(triangle VSOut gsIn[3], uint instanceID : SV_GSInstanceID, inout TriangleStream<GSOut> outStream)
{
    // Get current light mesh's data
    uint lightMeshIndex = gReusingLightIndexBuffer[gCurrFrameLightStartIdx + gRanking].x;
    uint face = 3;
    
    GSOut gsOut;

    // for (uint face = 0; face < 6; face++)
    {
        gsOut.rtIndex = face;
        gsOut.triangleIndex = face;
        float4x4 lightViewProj = gLightMeshDataBuffer[lightMeshIndex].viewProjMat[face];

        for (uint v = 0; v < 3; v++)
        {
            gsOut.vsOut = gsIn[v];
            gsOut.vsOut.posH = mul(float4(gsIn[v].posW, 1.0f), lightViewProj);
            outStream.Append(gsOut);
        }
        outStream.RestartStrip();
    }

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

PSOut psMain(GSOut psIn)
{
    // printSetPixel(uint2(psIn.vsOut.posH.xy));

    PSOut psOut;

    // let lod = ImplicitLodTextureSampler();
    // if (alphaTest(psIn.vsOut, triangleIndex, lod)) discard;

    print("posH = ", psIn.vsOut.posH);
    print("psIn.vsOut.instanceID = ", psIn.vsOut.instanceID.index);
    print("psIn.triangleIndex = ", psIn.triangleIndex);

    // TODO: ignore the emissive surface's fragment
    let lod = ExplicitLodTextureSampler(0.f);
    float3 viewDir = normalize(gScene.camera.getPosition() - psIn.vsOut.posW);
    ShadingData sd = prepareShadingData(psIn.vsOut, psIn.triangleIndex, viewDir, lod);
    print("isEmissive = ", sd.mtl.isEmissive());
    if (sd.mtl.isEmissive()) discard;

    float depth = saturate(psIn.vsOut.posH.z);
    psOut.color = depth;
    psOut.depth = depth;

    return psOut;
}