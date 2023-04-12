import Scene.Scene;
import Utils.Debug.PixelDebug;
import ISMHelpers;

cbuffer CB
{
    uint gLaunchWidth;
    uint gTotalNonEmisTriCount;
}

Buffer<uint2> gNonEmisInstTriOffsetBuffer;
RWBuffer<float4> gNonEmissiveTriVerticesBuffer;

[numthreads(16, 16, 1)]
void getTriVertices(uint3 threadID : SV_DispatchThreadID)
{
    uint globalTriangleIdx = threadID.y * gLaunchWidth + threadID.x;
    if (globalTriangleIdx >= gTotalNonEmisTriCount) return;

    // Get instanceID using binary search
    uint id = findInterval(gNonEmisInstTriOffsetBuffer, globalTriangleIdx);
    GeometryInstanceID instanceID = { gNonEmisInstTriOffsetBuffer[id].y };

    // Get the three vertices of current triangle (without world transform)
    uint triangleIndex = globalTriangleIdx - gNonEmisInstTriOffsetBuffer[id].x;
    uint3 vtxIndices = gScene.getIndices(instanceID, triangleIndex);
    for (uint i = 0; i < 3; i++)
        gNonEmissiveTriVerticesBuffer[globalTriangleIdx * 3 + i] = float4(gScene.vertices[vtxIndices[i]].position, instanceID.index);
}
