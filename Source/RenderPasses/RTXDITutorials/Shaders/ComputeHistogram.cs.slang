// --------------------------------------------------------------------
// 1. Reduction pass for counting the duplicate mesh light ID within the input texture
// --------------------------------------------------------------------
import Utils.Debug.PixelDebug;

#define BLOCK_SIZE 128

cbuffer histogramCB
{
    uint gTotalLightMeshCount;
}

Buffer<int> gPrevLightMeshSelectionBuffer;
RWBuffer<uint> gFinalHistogramBuffer;

groupshared uint sBlockHistogram[BLOCK_SIZE];

// One atomic add in shared memory and one in global memory
[numthreads(BLOCK_SIZE, 1, 1)]
void computeHistogramTwoAdds(uint3 threadId: SV_DispatchThreadID, uint3 groupThreadId: SV_GroupThreadID, uint3 groupId: SV_GroupID)
{
    // printSetPixel(threadId.xy);

    // Initialize shared memory
    sBlockHistogram[groupThreadId.x] = 0;
    GroupMemoryBarrierWithGroupSync();

    // Add to the shared memory
    int lightMeshIndex = gPrevLightMeshSelectionBuffer[threadId.x];
    if (lightMeshIndex != -1) InterlockedAdd(sBlockHistogram[lightMeshIndex], 1);
    GroupMemoryBarrierWithGroupSync();

    // Add to global memory
    if (groupThreadId.x < gTotalLightMeshCount)
    {
        uint sum = sBlockHistogram[groupThreadId.x];
        InterlockedAdd(gFinalHistogramBuffer[groupThreadId.x], sum);
        GroupMemoryBarrierWithGroupSync();
    }
}

// The following part is another unchecked counting method

#define WORD_COUNT 4 // this should be ceil(gTotalLightMeshCount / 32)

// Max shared memory size = 19KB. We only need 1-bit to store mark each position in shared memory
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



