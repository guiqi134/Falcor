import Scene.Scene;
import Utils.Debug.PixelDebug;
import ISMHelpers;
import Utils.Math.PackedFormats;
import Utils.Geometry.GeometryHelpers;

cbuffer CB
{
    uint gTotalPointSamples;
    uint gLaunchWidth;
    uint2 gScreenSize;
    float gCameraNear;
    uint gStride;
    int gDebugLightID;
    bool gVisualizeFaceSamples;
    uint gFirstSpotLightID;
}

static const float3 COLORS[6] = {
    float3(1, 0, 0), float3(0, 1, 0), float3(0, 0, 1),
    float3(1, 1, 0), float3(0, 1, 1), float3(1, 0, 1) // Yellow, Cyan, Magenta
};

static const float kDepthEpsilon = 0.001f;

StructuredBuffer<LightShadowMapData> gLightShadowDataBuffer;
StructuredBuffer<PackedIsmPointSample> gPointsBuffer;
Buffer<float4> gGBuffer; // g-buffer created by rtxdi
RWTexture2D gOutputColor;

[numthreads(16, 16, 1)]
void allPointSamples(uint3 threadID : SV_DispatchThreadID)
{
    uint globalThreadID = threadID.y * gLaunchWidth + threadID.x;
    if (globalThreadID >= gTotalPointSamples || globalThreadID % gStride != 0) return;

    // printSetPixel(threadID.xy);

    // Get the point info
    IsmPointSample pointSample = unpackIsmPointSample(gPointsBuffer[globalThreadID]);
    GeometryInstanceID instanceID = { pointSample.instanceID };
    float3 posW = mul(float4(pointSample.pos, 1.0f), gScene.getWorldMatrix(instanceID)).xyz;
    uint selectedLightID = pointSample.selectedLightID;


    // Project point to camera
    float4 posClip = mul(float4(posW, 1.0f), gScene.camera.getViewProj());
    float3 posNDC = posClip.xyz / posClip.w;
    float3 posScreenNormalized = float3(posNDC.xy * float2(0.5f, -0.5f) + 0.5f, posNDC.z);
    uint2 posScreen = uint2(posScreenNormalized.xy * gScreenSize);

    printSetPixel(posScreen.xy);

    print("posW = ", posW);
    print("posClip = ", posClip);
    print("gStride = ", gStride);

    // Cull the point if it's outside the texture space
    if (any(posScreenNormalized.xy < 0.0f) || any(posScreenNormalized.xy > 1.0f)) return;

    // Cull the point if it is behind the near plane. posClip.w = -posView.z
    if (posClip.w < gCameraNear) return;

    float4 outputColor;
    if (gDebugLightID < 0)
    {
        // Set all point samples' pixel color to black
        outputColor = float4(float3(0.0f), 1.0f);
    }
    else
    {
        // Set the pixel to red if it belongs to debug light
        if (gDebugLightID == pointSample.selectedLightID)
            outputColor = float4(1.0f, 0.0f, 0.0f, 1.0f);

        if (gVisualizeFaceSamples)
        {
            // Get the sample light info
            LightShadowMapData lightShadowData = gLightShadowDataBuffer[selectedLightID];

            // Project the point to the light face
            float3 lightCenterW = lightShadowData.centerPosW;
            float3 lightToPoint = posW - lightCenterW;
            uint faceIdx = selectedLightID < gFirstSpotLightID ? getFaceIndex(normalize(lightToPoint)) : 0;
            outputColor = float4(COLORS[faceIdx], 1.0f);
        }
    }

    // Get the depth on current pixel position in screen's G-Buffer
    bool valid = all(posScreen < gScreenSize); // Is pixelPosition in the image?
    uint idx = valid ? posScreen.x + (posScreen.y * gScreenSize.x) : 0x0u;
    float viewDepth = gGBuffer[idx].w;
    uint3 packedData = asuint(gGBuffer[idx].xyz);
    float3 normalW = decodeNormal2x16(packedData.x);

    // Compute the depth of offseted point sample to camera
    float3 cameraPosW = gScene.camera.getPosition();
    float3 newPosW = computeRayOrigin(posW, normalW);
    float dist = (1.0 - kDepthEpsilon) * distance(cameraPosW, newPosW);
    if (dist <= viewDepth)
    {
        gOutputColor[posScreen] = outputColor;
    }
}
