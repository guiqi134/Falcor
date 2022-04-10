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
Projection Matrix (row major):
xScale     0          0              0
0        yScale       0              0
0          0      zf/(zn-zf)        -1
0          0      zn*zf/(zn-zf)      0
*/
float LinearizeDepth(float depth)
{
    return (nearZ * farZ) / (farZ + depth * (nearZ - farZ));
}

#ifdef _VSM
float2 psMain(VSOut vsOut, uint triangleIndex : SV_PrimitiveID) : SV_Target0
{
    printSetPixel(vsOut.posH.xy);

    if (alphaTest(vsOut, triangleIndex)) discard;

    float depth = saturate(vsOut.posH.z); // posH is screen space coordinate
    float2 moments = float2(depth, depth * depth);

    // Compute partial derivatives of depth.
    //float dx = ddx(depth);
    //float dy = ddy(depth);
    // Compute second moment over the pixel extents. (including bias)
    //moments.y = depth * depth + 0.25 * (dx * dx + dy * dy);

    return moments; // TODO: better use linear depth 
}
#elif defined(_EVSM)
float4 psMain(VSOut vsOut, uint triangleIndex : SV_PrimitiveID) : SV_Target0
{
    printSetPixel(vsOut.posH.xy);
    if (alphaTest(vsOut, triangleIndex)) discard;

    // Compute EVSM exponents
    float depth = saturate(vsOut.posH.z);
    float2 expDepth = getEvsmExponents(depth);
    float4 result = float4(expDepth, expDepth * expDepth);

    return result.xzyw;
}
#elif defined(_MSM)
float4 psMain(VSOut vsOut, uint triangleIndex : SV_PrimitiveID) : SV_Target0
{
    printSetPixel(vsOut.posH.xy);
    if (alphaTest(vsOut, triangleIndex)) discard;

    float depth = saturate(vsOut.posH.z);

    // Compute 4 moments
    return float4(depth, depth * depth, depth * depth * depth, depth * depth * depth * depth);
}
#else
void psMain(VSOut vsOut, uint triangleIndex : SV_PrimitiveID) : SV_Target0
{
    //printSetPixel(vsOut.posH.xy);
    if (alphaTest(vsOut, triangleIndex)) discard;
}
#endif
