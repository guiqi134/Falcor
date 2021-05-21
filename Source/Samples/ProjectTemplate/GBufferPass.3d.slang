import Scene.Shading;
import Scene.Raster;
import Utils.Helpers;

cbuffer PerFrameCB
{
    float2  frameSize;
    float3  cameraPos;
};

RWTexture2D<float2> motionVectors;
RWTexture2D<float2> gPosNormalFwidth;
RWTexture2D<float2> gLinearZAndDeriv;

struct GBuffer 
{
	float4 albedo    : SV_Target0;
    float4 spec      : SV_Target1;
    float4 positon   : SV_Target2;
    float4 normal    : SV_Target3;
    float4 extra     : SV_Target4;
    float4 emissive  : SV_Target5;
    float4 faceN     : SV_Target6;
};

VSOut vsMain(VSIn vsIn)
{
    return defaultVS(vsIn);
}

[earlydepthstencil]
GBuffer psMain(VSOut vOut, uint triangleIndex : SV_PrimitiveID)
{
    int2 ipos = int2(vOut.posH.xy);

    float3 faceNormal = gScene.getFaceNormalW(vOut.meshInstanceID, triangleIndex);
    VertexData v = prepareVertexData(vOut, faceNormal);

    if (alphaTest(v, gScene.materials[vOut.materialID], gScene.materialResources[vOut.materialID])) 
    {
        discard;
    }

    float3 viewDir = normalize(gScene.camera.getPosition() - vOut.posW);
    ShadingData sd = prepareShadingData(vOut, triangleIndex, viewDir);

    adjustShadingNormal(sd, v);

    GBuffer gBufOut;
    gBufOut.positon  = float4(sd.posW, 1.f);
    gBufOut.normal   = float4(sd.N, length(sd.posW - cameraPos));
	gBufOut.albedo   = float4(sd.diffuse, sd.opacity);
    gBufOut.spec     = float4(sd.specular.rgb, sd.linearRoughness);
    gBufOut.extra    = float4(sd.IoR, sd.doubleSided ? 1.f : 0.f, sd.metallic, 0.f);
    gBufOut.emissive = float4(sd.emissive, 1.f);
    gBufOut.faceN    = float4(sd.faceN,0.f);                  

    // Motion Vectors
    const float2 pixelPos = ipos + float2(0.5, 0.5); // Current sample in pixel coords.
    const float4 prevPosH = vOut.prevPosH; // Sample in previous frame in clip space coords, no jittering applied.
    const float2 mv = calcMotionVector(pixelPos, prevPosH, frameSize) + float2(gScene.camera.data.jitterX, -gScene.camera.data.jitterY); // Remove camera jitter from motion vector
    motionVectors[ipos] = mv;

    // Pos and Normal's Derivatives 
    gPosNormalFwidth[ipos] = float2(length(fwidth(sd.posW)), length(fwidth(sd.N)));

    // Linear z and its derivative
    const float linearZ = vOut.posH.z * vOut.posH.w;
    gLinearZAndDeriv[ipos] = float2(linearZ, max(abs(ddx(linearZ)), abs(ddy(linearZ))));
    
    return gBufOut;
}
