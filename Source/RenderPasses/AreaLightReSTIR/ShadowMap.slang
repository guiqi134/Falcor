import Scene.Raster; // include Scene.Scene
import Utils.Debug.PixelDebug;


cbuffer PerFrameCB
{
    float4x4 gLightSpaceMat;
    float nearZ;
    float farZ;
};

//struct VertexOut
//{
//    nointerpolation GeometryInstanceID instanceID : INSTANCE_ID;  ///< Geometry instance ID.
//
//    float4 posH : SV_POSITION; // similar to gl_Position, must be binded which will be used by rasterizer
//};

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

void psMain(VSOut vsOut, uint triangleIndex : SV_PrimitiveID) : SV_TARGET
{
    if (alphaTest(vsOut, triangleIndex)) discard;

    /*float depth = vsOut.posH.z;
    float linearDepth = LinearizeDepth(depth) / farZ;
    return float4(float3(linearDepth), 1.0f);*/
}
