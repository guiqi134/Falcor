// --------------------------------------------------------------------
// 1. Reduction pass for counting the duplicate mesh light ID within the input texture
// --------------------------------------------------------------------
import Utils.Debug.PixelDebug;

cbuffer histogramCB
{
    uint gTotalBinCount;
}

Buffer<int> gPrevLightSelectionBuffer;
RWBuffer<uint> gFinalHistogramBuffer;

groupshared uint sBlockHistogram[8192]; // Note: total number of light faces should be smaller than 8K

// One atomic add in shared memory and one in global memory
[numthreads(1024, 1, 1)]
void computeHistogramTwoAdds(uint3 threadId: SV_DispatchThreadID, uint3 groupThreadId: SV_GroupThreadID, uint3 groupId: SV_GroupID)
{
    printSetPixel(threadId.xy);

    uint nSections = (gTotalBinCount - 1) / 1024 + 1;
    // print("nSections = ", nSections);

    // Initialize shared memory
    for (uint i = 0; i < nSections; i++)
    {
        uint sharedIndex = groupThreadId.x + (i * 1024);
        sBlockHistogram[sharedIndex] = 0;
    }
    GroupMemoryBarrierWithGroupSync();

    // Add to the shared memory. Only 1024 data will be loaded
    int lightMeshIndex = gPrevLightSelectionBuffer[threadId.x];
    if (lightMeshIndex != -1)
        InterlockedAdd(sBlockHistogram[(uint)lightMeshIndex], 1);
    GroupMemoryBarrierWithGroupSync();

    // Add to global memory
    if (nSections == 1)
    {
        if (groupThreadId.x < gTotalBinCount)
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
            if (sharedIndex < gTotalBinCount)
            {
                uint sum = sBlockHistogram[sharedIndex];
                if (sum != 0) InterlockedAdd(gFinalHistogramBuffer[sharedIndex], sum);
            }
        }
    }
}

RWBuffer<uint2> gSortBuffer;
RWBuffer<uint> gTotalValidPixels;

[numthreads(256, 1, 1)]
void createKeyValuePairs(uint3 threadID : SV_DispatchThreadID)
{
    gSortBuffer[threadID.x] = uint2(threadID.x, gFinalHistogramBuffer[threadID.x]);
    InterlockedAdd(gTotalValidPixels[0], gFinalHistogramBuffer[threadID.x]);
}



// The following part is another unchecked counting method

/*
#define BLOCK_SIZE 128
#define WORD_COUNT 4 // this should be ceil(gTotalLightMeshCount / 32)

// Max shared memory size = 32KB. We only need 1-bit to store mark each position in shared memory
groupshared uint sPerBlockHistogram[WORD_COUNT * BLOCK_SIZE];

// TODO: Test this method later if two atmoic add method is too slow
[numthreads(BLOCK_SIZE, 1, 1)]
void computeHistogramTesting(uint3 threadId: SV_DispatchThreadID, uint3 groupThreadId: SV_GroupThreadID, uint3 groupId: SV_GroupID)
{
    printSetPixel(threadId.xy);

    // Initialize shared memory
    [unroll]
    for (uint i = 0; i < WORD_COUNT; i++)
    {
        // Each thread initializes one column
        sPerBlockHistogram[groupThreadId.x + i * BLOCK_SIZE] = 0;
    }
    GroupMemoryBarrierWithGroupSync();

    // Get light index data from device memory
    uint lightMeshIndex = gPrevLightMeshSelectionBuffer[threadId.x];

    // First get the index of word (4-byte data) for current thread and then offset into the bit position of that word
    uint wordIndex = floor(lightMeshIndex / 32);
    uint bitIndex = lightMeshIndex % 32;
    uint word = (1 << bitIndex);

    sPerBlockHistogram[wordIndex * BLOCK_SIZE + groupThreadId.x] = word;
    GroupMemoryBarrierWithGroupSync();

    // Accumulate each row of per-block histogram and write to device memory.
    // Each thread will process one row, but what if light mesh count > block size? -> maybe each thread will process two/three rows
    if (groupThreadId.x < gTotalLightMeshCount)
    {
        lightMeshIndex = groupThreadId.x;
        wordIndex = floor(lightMeshIndex / 32);
        bitIndex = lightMeshIndex % 32;

        uint sum = 0;

        // TODO: handle 16-way bank conflicts
        [unroll]
        for (uint i = 0; i < BLOCK_SIZE; i++)
        {
            uint wordData = sPerBlockHistogram[wordIndex * BLOCK_SIZE + i];
            uint bitData = wordData & (1 << bitIndex) == 0 ? 0 : 1; // 0 or 1
            sum += bitData;
        }

        // InterlockedAdd(gFinalHistogramBuffer[lightMeshIndex], sum);
        // gFinalHistogramBuffer[groupId.x * BLOCK_SIZE + groupThreadId.x] = sum;
    }
}
*/


