import Experimental.Scene.Material.MaterialHelpers;
import Scene.Raytracing;
import Scene.ShadingData;
import Utils.Math.MathHelpers;

// GBuffer sturct: 152B
struct GBuffer
{
    float4 posW;
    float4 normalW;
    float4 tangentW;
    float4 faceNormal;
    float4 viewW;
    float4 mtlDiffOpacity;
    float4 mtlSpecRough;
    float4 mtlEmissive;
    float4 mtlParams;
    float2 mvec;
};

// Internal buffer data
RWStructuredBuffer<GBuffer> gGBuffer;

/** Helper to load the material attributes.
*/
MaterialParams loadMaterialParams(uint index)
{
    MaterialParams matParams;
    matParams.diffuseOpacity = gGBuffer[index].mtlDiffOpacity;
    matParams.specularRoughness = gGBuffer[index].mtlSpecRough;
    matParams.emissive = gGBuffer[index].mtlEmissive;
    matParams.extraParams = gGBuffer[index].mtlParams;

    return matParams;
}

/** Returns the primary ray's direction.
*/
float3 getPrimaryRayDir(uint index, uint2 launchIndex, uint2 launchDim, const Camera camera)
{
    float4 viewW = gGBuffer[index].viewW;
    if (any(viewW)) 
    {
        // If we have the view vector bound as a buffer, just fetch it. No need to compute anything.
        return -viewW.xyz;
    }
    else
    {
        // Compute the view vector. This must exactly match what the G-buffer pass is doing (jitter etc.).
        // Note that we do not take depth-of-field into account as it would require exactly matching the
        // sample generator between the passes, which is error prone. The host side will issue a warning instead.
        return camera.computeRayPinhole(launchIndex, launchDim).dir;
    }
}

ShadingData prepareShadingDataHelper(uint index, const GBuffer pixelGBuffer)
{
    const float3 rayDir = -pixelGBuffer.viewW.xyz;
    const float4 worldPos = pixelGBuffer.posW;
    float3 normal = pixelGBuffer.normalW.xyz;
    float4 tangent = pixelGBuffer.tangentW;
    float3 faceNormal = pixelGBuffer.faceNormal.xyz;
    GeometryParams geoParams = prepareGeometryParams(worldPos.xyz, -rayDir, normal, tangent, faceNormal);
    MaterialParams matParams = loadMaterialParams(index); 
    ShadingData sd = prepareShadingData(geoParams, matParams);

    return sd;
}

// void updateLastFrameGBuffer(uint index, RWStructuredBuffer<GBuffer> lastFrameGBuffer)
// {

//     lastFrameGBuffer[index].posW = gGBuffer[index].posW;
//     lastFrameGBuffer[index].normalW = gGBuffer[index].normalW;
//     lastFrameGBuffer[index].tangentW = gGBuffer[index].tangentW;
//     lastFrameGBuffer[index].faceNormal = gGBuffer[index].faceNormal;
//     lastFrameGBuffer[index].viewW = gGBuffer[index].viewW;
//     lastFrameGBuffer[index].mtlDiffOpacity = gGBuffer[index].mtlDiffOpacity;
//     lastFrameGBuffer[index].mtlSpecRough = gGBuffer[index].mtlSpecRough;
//     lastFrameGBuffer[index].mtlEmissive = gGBuffer[index].mtlEmissive;
//     lastFrameGBuffer[index].mtlParams = gGBuffer[index].mtlParams;
//     lastFrameGBuffer[index].mvec = gGBuffer[index].mvec;
// }