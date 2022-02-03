import Scene.Raster; // include Scene.Scene
import Utils.Debug.PixelDebug;


cbuffer PerFrameCB
{
    float4x4 gLightSpaceMat;
    float nearZ;
    float farZ;
};

struct VertexOut
{
    nointerpolation GeometryInstanceID instanceID : INSTANCE_ID;  ///< Geometry instance ID.

    float4 posH : SV_POSITION; // similar to gl_Position, must be binded which will be used by rasterizer
};

VertexOut vsMain(VSIn vIn)
{
    VertexOut vsOut;
    const GeometryInstanceID instanceID = { vIn.meshInstanceID };

    float4x4 worldMat = gScene.getWorldMatrix(instanceID);
    float3 posW = mul(float4(vIn.pos, 1.f), worldMat).xyz;
    vsOut.posH = mul(float4(posW.xyz, 1.0f), gLightSpaceMat);
    vsOut.instanceID = instanceID;

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

float4 psMain(VertexOut vsOut) : SV_Target0
{
    // Discard the pixel on the plane mesh
    if (vsOut.instanceID.index == 0) discard;

    float depth = vsOut.posH.z;
    float linearDepth = LinearizeDepth(depth) / farZ;
    return float4(float3(linearDepth), 1.0f);
}
