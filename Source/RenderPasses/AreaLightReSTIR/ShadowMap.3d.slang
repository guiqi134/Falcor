import Scene.Raster; // include Scene.Scene
import Utils.Debug.PixelDebug;
import VSM;
import MSM;

cbuffer PerFrameCB
{
    float4x4 gLightSpaceMat;
    float nearZ;
    float farZ;
};

VSOut vsMain(VSIn vIn)
{
    VSOut vsOut;
    vsOut = defaultVS(vIn);
    
    float3 posW = vsOut.posW;
    vsOut.posH = mul(float4(posW.xyz, 1.0f), gLightSpaceMat);

    return vsOut;
}


/*
D3DXMatrixPerspectiveFovRH (row major):
xScale     0          0              0
0        yScale       0              0
0          0      zf/(zn-zf)        -1
0          0      zn*zf/(zn-zf)      0
*/
float LinearizeDepth(float depth)
{
    return (nearZ * farZ) / (depth * (nearZ - farZ) - farZ);
}

struct PSOut
{
#ifdef _VSM
    float2 color : SV_Target;
#elif defined(_EVSM) || defined(_MSM)
    float4 color : SV_Target;
#endif
    float depth : SV_Depth;
};

PSOut psMain(VSOut vsOut, uint triangleIndex : SV_PrimitiveID)
{
    printSetPixel(vsOut.posH.xy);
    if (alphaTest(vsOut, triangleIndex)) discard;

    PSOut psOut;
    float depth = saturate(vsOut.posH.z); // posH is screen space coordinate (already perform homogeneous divide)

#ifdef _VSM
    float2 moments = float2(depth, depth * depth);
    psOut.color = moments;
#elif defined(_EVSM)
    // Compute EVSM exponents
    float2 expDepth = getEvsmExponents(depth);
    float4 result = float4(expDepth, expDepth * expDepth);
    psOut.color = result.xzyw;
#elif defined(_MSM)
    psOut.color = float4(depth, depth * depth, depth * depth * depth, depth * depth * depth * depth);
#endif

    psOut.depth = vsOut.instanceID.index == 0 ? 1.0f : depth;
    return psOut;
}


