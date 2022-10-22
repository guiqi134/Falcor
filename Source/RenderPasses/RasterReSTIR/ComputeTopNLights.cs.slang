// import Utils.Debug.PixelDebug;
import Utils.Sampling.SampleGenerator;
import Scene.Scene;

cbuffer CB
{
    uint2 gScreenSize;
    uint gFrameIndex;
    uint gTotalLightMeshCount;
}

// Generate random numbers for input buffer
RWBuffer<int> gInputBuffer;

[numthreads(256, 1, 1)]
void initializeBuffer(uint3 threadId : SV_DispatchThreadID)
{
    SampleGenerator sg = SampleGenerator(threadId.xy, 1);
    uint bufferIndex = threadId.y * gScreenSize.x + threadId.x;
    int16_t random = sampleNext1D(sg) * gTotalLightMeshCount;
    gInputBuffer[bufferIndex] = random;
}

// 1. Reduction pass for counting the duplicate mesh light ID within the input texture
Buffer<int> gPrevLightMeshSelectionBuffer;
RWBuffer<uint> gFinalHistogramBuffer;
RWBuffer<uint2> gSortBuffer;

groupshared uint sBlockHistogram[8192]; // 16384

[numthreads(1024, 1, 1)]
void computeHistogramTwoAdd(uint3 threadId: SV_DispatchThreadID, uint3 groupThreadId: SV_GroupThreadID, uint3 groupId: SV_GroupID)
{
    // printSetPixel(threadId.xy);

    uint nSections = (gTotalLightMeshCount - 1) / 1024 + 1;
    // print("nSections = ", nSections);

    // Initialize shared memory
    for (uint i = 0; i < nSections; i++)
    {
        uint sharedIndex = groupThreadId.x + (i * 1024);
        sBlockHistogram[sharedIndex] = 0;
    }
    GroupMemoryBarrierWithGroupSync();

    // Add to the shared memory. Only 1024 data will be loaded
    uint lightMeshIndex = gPrevLightMeshSelectionBuffer[threadId.x];
    InterlockedAdd(sBlockHistogram[lightMeshIndex], 1);
    GroupMemoryBarrierWithGroupSync();

    // Add to global memory
    if (nSections == 1)
    {
        if (groupThreadId.x < gTotalLightMeshCount)
        {
            uint sum = sBlockHistogram[groupThreadId.x];
            if (sum != 0) InterlockedAdd(gFinalHistogramBuffer[groupThreadId.x], sum);
        }
    }
    else
    {
        for (uint i = 0; i < nSections; i++)
        {
            uint sharedIndex = groupThreadId.x + (i * 1024);
            if (sharedIndex < gTotalLightMeshCount)
            {
                uint sum = sBlockHistogram[sharedIndex];
                if (sum != 0) InterlockedAdd(gFinalHistogramBuffer[sharedIndex], sum);
            }
        }
    }
    // AllMemoryBarrierWithGroupSync();

    // if (threadId.x < gTotalLightMeshCount)
    // {
    //     gSortBuffer[threadId.x] = uint2(threadId.x, gFinalHistogramBuffer[threadId.x]);
    // }
}



