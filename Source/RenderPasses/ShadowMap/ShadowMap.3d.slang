import Scene.Raster; // include Scene.Scene
import Utils.Debug.PixelDebug;

cbuffer PerFrameCB
{
    float4x4 gLightSpaceMat;
};

VSOut vsMain(VSIn vIn)
{
    VSOut vsOut;
    vsOut = defaultVS(vIn);

    float3 posW = vsOut.posW;
    vsOut.posH = mul(float4(posW.xyz, 1.0f), gLightSpaceMat);

    return vsOut;
}


float LinearizeDepth(float depth)
{
    return (_LIGHT_NEAR_PLANE * _LIGHT_FAR_PLANE)
        / (_LIGHT_FAR_PLANE + depth * (_LIGHT_NEAR_PLANE - _LIGHT_NEAR_PLANE));
}

#ifdef _VSM
float2 psMain(VSOut vsOut, uint triangleIndex : SV_PrimitiveID) : SV_Target0
{
    printSetPixel(vsOut.posH.xy);

    if (alphaTest(vsOut, triangleIndex)) discard;

    float depth = vsOut.posH.z; // posH is screen space coordinate
    float2 moments = float2(depth, depth * depth);

    // Compute partial derivatives of depth.
    //float dx = ddx(depth);
    //float dy = ddy(depth);
    // Compute second moment over the pixel extents. (including bias)
    //moments.y = depth * depth + 0.25 * (dx * dx + dy * dy);

    return moments; // TODO: better use linear depth 
}
#else
void psMain(VSOut vsOut, uint triangleIndex : SV_PrimitiveID) : SV_Target
{
    printSetPixel(vsOut.posH.xy);
    if (alphaTest(vsOut, triangleIndex)) discard;
}
#endif




