// import Utils.Debug.PixelDebug;
import Scene.Scene;

cbuffer CB
{
    uint dummy;
}

Buffer<uint> gHistogramBuffer;
RWBuffer<uint2> gKeyValueBuffer;

[numthreads(1024, 1, 1)]
void buildKeyValuePairs(uint3 threadId : SV_DispatchThreadID)
{
    uint lightMeshIndex = threadId.x;
    gKeyValueBuffer[threadId.x] = uint2(lightMeshIndex, gHistogramBuffer[lightMeshIndex]);
}
