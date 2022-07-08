import Scene.Raster;
import Utils.Debug.PixelDebug;

#define USE_ALPHA_TEST 0

cbuffer ZBufferCB
{
    float4x4 gWorldToCamera;
    float gNearPlane;
    float gFarPlane;
    uint2 gPrevCurrLayerIndex;
}

Texture2D gPrevZBuffer;

struct ZBufferVSOut
{
    float2 texCoords : TEXCRD;
    float4 posCS : PosCameraSpace; // TODO: this can be further reduce to only z component

    nointerpolation GeometryInstanceID instanceID : INSTANCE_ID;
    nointerpolation uint materialID : MATERIAL_ID;

    float4 posH : SV_Position;
};


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




ZBufferVSOut vsMain(VSIn vsIn) 
{
    ZBufferVSOut vsOut;
    const GeometryInstanceID instanceID = { vsIn.instanceID };

    float4x4 objectToWorld = gScene.getWorldMatrix(instanceID);
    float3 posW = mul(float4(vsIn.pos, 1.f), objectToWorld).xyz;

    vsOut.posCS = mul(float4(posW, 1.f), gWorldToCamera);
    vsOut.posH = mul(float4(posW, 1.f), gScene.camera.getViewProj());
    vsOut.instanceID = instanceID;
    vsOut.materialID = gScene.getMaterialID(instanceID);

    return vsOut;
}

float4 psMain(ZBufferVSOut vsOut) : SV_Target0
{
    uint2 pixelPos = uint2(vsOut.posH.xy);
    printSetPixel(pixelPos);

#if USE_ALPHA_TEST
    VertexData v = {};
    v.texC = vsOut.texCoords;
    float lod = 0.f;
    if (gScene.materials.alphaTest(v, vsOut.materialID, lod)) discard;
#endif

    // Use previous pass's camera space depth (negative) to peel the scene
    float4 prevDepths = gPrevZBuffer[pixelPos]; 
    float currLayerDepth = vsOut.posCS.z;
    if (gPrevCurrLayerIndex.y != 0 && currLayerDepth >= prevDepths[gPrevCurrLayerIndex.x])
        discard;

    // If not discard, store current depth to corresponding channel
    float4 result = prevDepths;
    result[gPrevCurrLayerIndex.y] = currLayerDepth;

    // Then the peeled result will go to the depth test
    return result;
}