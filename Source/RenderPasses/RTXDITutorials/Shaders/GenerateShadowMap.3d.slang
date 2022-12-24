import Scene.Raster;
import Utils.Debug.PixelDebug;

#include "../HostDeviceSharedDefinitions.h"

cbuffer shadowMapCB
{
    uint gRanking;
    uint gCurrFrameLightStartIdx;
    uint gVisMode;
    float gDepthBias;
}

Buffer<uint> gReusingLightIndexBuffer;
StructuredBuffer<LightShadowMapData> gLightShadowDataBuffer; // 0, 1, ..., Max
Buffer<uint2> gSortedLightsBuffer; // for baseline use

struct GSOut
{
    VSOut vsOut;
    float3 lightCenter : LightCenter;
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
[maxvertexcount(18)]
void gsMain(triangle VSOut gsIn[3], uint instanceID : SV_GSInstanceID, inout TriangleStream<GSOut> outStream)
{
    // Get current light mesh's data
    uint localLightIdx;
    if (gVisMode == (uint)Visibility::BaselineSM)
    {
        localLightIdx = gSortedLightsBuffer[gRanking].x;
    }
    else
    {
        localLightIdx = gReusingLightIndexBuffer[gCurrFrameLightStartIdx + gRanking];
    }
    // uint face = 3;

    GSOut gsOut;

    for (uint face = 0; face < 6; face++)
    {
        gsOut.rtIndex = face;
        gsOut.triangleIndex = face;
        gsOut.lightCenter = gLightShadowDataBuffer[localLightIdx].centerPosW;
        float4x4 lightViewProj = gLightShadowDataBuffer[localLightIdx].viewProjMats[face];

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

    // Ignore the emissive surface's fragment
    float3 viewDir = normalize(gScene.camera.getPosition() - psIn.vsOut.posW);
    ShadingData sd = prepareShadingData(psIn.vsOut, psIn.triangleIndex, viewDir, lod);
    // print("isEmissive = ", sd.mtl.isEmissive());
    if (sd.mtl.isEmissive()) discard;

    // Compute the depth bias
    //  -> better do it in the shading part using NdotL
    float NdotL = dot(psIn.vsOut.normalW, psIn.lightCenter - psIn.vsOut.posW);
    // float bias = max(5.0 * gDepthBias * (1.0 - NdotL), gDepthBias);
    float bias = gDepthBias + 5.0 * gDepthBias * max(ddx(psIn.vsOut.posH.z), ddy(psIn.vsOut.posH.z));

    print("bias = ", bias);
    print("ddx(psIn.vsOut.posH.z) = ", ddx(psIn.vsOut.posH.z));
    print("ddy(psIn.vsOut.posH.z) = ", ddy(psIn.vsOut.posH.z));

    float depth = saturate(psIn.vsOut.posH.z);
    psOut.color = depth;
    psOut.depth = depth;

    return psOut;
}
