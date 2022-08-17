import Utils.Debug.PixelDebug;
import Utils.Sampling.SampleGenerator;

#define TOTAL_LIGHT_MESH_COUNT 102 // this should be the size of output histogram buffer

cbuffer CB
{
    // uint gLightMeshCount; // Can this be changed into a compile-time constant?
    uint2 gScreenSize;
    uint gFrameIndex;
}

// Generate random numbers for input buffer
RWBuffer<int> gInputBuffer;

[numthreads(256, 1, 1)]
void initializeBuffer(uint3 threadId : SV_DispatchThreadID)
{
    SampleGenerator sg = SampleGenerator(threadId.xy, gFrameIndex);
    uint bufferIndex = threadId.y * gScreenSize.x + threadId.x;
    int16_t random = sampleNext1D(sg) * TOTAL_LIGHT_MESH_COUNT;
    gInputBuffer[bufferIndex] = random;
}


// 1. Reduction pass for counting the duplicate mesh light ID within the input texture
Buffer<int> gPrevLightMeshSelectionBuffer;
RWBuffer<uint> gFinalHistogramBuffer;

// TODO: later move to Falcor side
#define BLOCK_SIZE 128
#define WORD_COUNT 4

// Max shared memory size = 19KB. We only need 1-bit to store mark each position in shared memory
groupshared uint sPerBlockHistogram[WORD_COUNT * BLOCK_SIZE];

// TODO: Test this method later if two atmoic add method is too slow
[numthreads(BLOCK_SIZE, 1, 1)]
void computeHistogram(uint3 threadId : SV_DispatchThreadID, uint3 groupThreadId : SV_GroupThreadID, uint3 groupId : SV_GroupID)
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
    if (groupThreadId.x < TOTAL_LIGHT_MESH_COUNT)
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

groupshared uint sBlockHistogram[BLOCK_SIZE];

[numthreads(BLOCK_SIZE, 1, 1)]
void computeHistogramTwoAdd(uint3 threadId: SV_DispatchThreadID, uint3 groupThreadId: SV_GroupThreadID, uint3 groupId: SV_GroupID)
{
    // Initialize shared memory
    sBlockHistogram[groupThreadId.x] = 0;
    GroupMemoryBarrierWithGroupSync();

    // Add to the shared memory
    uint lightMeshIndex = gPrevLightMeshSelectionBuffer[threadId.x];
    InterlockedAdd(sBlockHistogram[lightMeshIndex], 1);
    GroupMemoryBarrierWithGroupSync();

    // Add to global memory
    if (groupThreadId.x < TOTAL_LIGHT_MESH_COUNT)
    {
        uint sum = sBlockHistogram[groupThreadId.x];
        InterlockedAdd(gFinalHistogramBuffer[groupThreadId.x], sum); 
    }
}

// 2. Sort the input buffer from previous counting result
#include "Utils/NVAPI.slangh" // We need this to get shuffle-xor operations.

#define GROUP_SIZE 256
#define CHUNK_SIZE 128 

#if (NV_WARP_SIZE != 32)
#error Kernel assumes warp size 32
#endif

cbuffer sortCB
{
    uint gTotalSize; ///< Total number of elements.
};

Buffer<uint> gKeysBuffer;
Buffer<uint> gValuesBuffer;
RWBuffer<uint2> gSortedLightMeshBuffer;

groupshared uint sKeys[GROUP_SIZE * 2];
groupshared uint sValues[GROUP_SIZE * 2];

/** Within warp bitonic sort, for iterations {j, j/2, ..., 1}, where j <= 16.
    \param[in,out] value The current thread's value.
    \param[in] i Global element index.
    \param[in] j Start element offset j<=16.
    \param[in] dir Sorting ascending (true) or descending (false).
*/
void bitonicSortInWarp(inout uint key, inout uint value, uint i, uint j, bool dir)
{
    for (; j > 0; j >>= 1)
    {
        // Get index of sorting partner in chunk whose thread id is i ^ j
        uint key_ixj = NvShflXor(key, j);
        uint value_ixj = NvShflXor(value, j);

        // Decide whether to swap.
        bool pred = (((i & j) == 0) != dir) == value < value_ixj;
        if (pred)
        {
            key = key_ixj;
            value = value_ixj;
        }
    }
}

// The kernel is currently written for a 1:1 mapping between elements to sort and threads.
[numthreads(GROUP_SIZE, 1, 1)]
void sortLightMeshHistogram(uint3 groupID: SV_GroupID, uint groupIdx: SV_GroupIndex)
{
    const uint group = groupID.x; // Sequential group index.
    const uint thid = groupIdx;                            // Local thread index in group (range 0..GROUP_SIZE-1).

    const uint globalIdx = group * GROUP_SIZE + thid; // Global element index in gData

    const uint N = CHUNK_SIZE;          // Number of elements per chunk to sort. Must be a power-of-two.
    const uint i = globalIdx & (N - 1); // i = local index of element in chunk, range [0,N).

    // Load value from memory.
    // Out-of-bounds elements are set to UINT_MAX (-1) to be placed last and allow data that is not a multiple of chunk size.
    uint2 keyValue = uint2(globalIdx, -1);
    if (globalIdx < gTotalSize)
    {
        keyValue = uint2(gKeysBuffer[globalIdx], gValuesBuffer[globalIdx]);
    }

    // Major steps for k = {2,4,...,32} are done within warp.
    for (uint k = 2; k <= min(N, 32); k <<= 1)
    {
        // Minor steps for iterations j = {16, 8, ..., 1} in warp.
        const bool dir = ((i & k) == 0); // Sort ascending (true) or descending (false)
        uint j = k >> 1;                 // j <= 16
        bitonicSortInWarp(keyValue.x, keyValue.y, i, j, dir);
    }

#if (CHUNK_SIZE > 32)
    // Load data into shared memory.
    sKeys[thid * 2] = keyValue.x;
    sValues[thid * 2] = keyValue.y;
    GroupMemoryBarrierWithGroupSync();

    // Major steps for k = {64,128,...N} are done in shared memory.
    for (uint k = 64; k <= N; k <<= 1)
    {
        const bool dir = ((i & k) == 0); // Sort ascending (true) or descending (false)

        // We ping-pong data in shared memory between adjacent addresses, using offset = {0, 1} to denote which one.
        uint offset = 0;

        // Minor steps for iterations j = {k/2, k/4, ..., 32} in shared memory.
        for (uint j = k >> 1; j >= 32; j >>= 1)
        {
            // Get sorting partner.
            uint2 keyValue_ixj = uint2(sKeys[(thid ^ j) * 2 + offset], sValues[(thid ^ j) * 2 + offset]);

            // Decide whether to swap. See comments in bitonicSortInWarp().
            bool pred = (((i & j) == 0) != dir) == keyValue.y < keyValue_ixj.y;
            if (pred) keyValue = keyValue_ixj;

            // Store result for next minor step (except for last iteration). Write to offset address to avoid race condition.
            if (j > 32)
            {
                sKeys[thid * 2 + (offset ^ 1)] = keyValue.x;
                sValues[thid * 2 + (offset ^ 1)] = keyValue.y;

                GroupMemoryBarrierWithGroupSync();
                offset ^= 1;
            }
        }

        // Minor steps for iterations j = {16, 8, ..., 1} in warp.
        uint jStart = min(k >> 1, 16);
        bitonicSortInWarp(keyValue.x, keyValue.y, i, jStart, dir);

        // Store result for major step (except for last iteration).
        if (k < N)
        {
            sKeys[thid * 2] = keyValue.x;
            sValues[thid * 2] = keyValue.y;
            GroupMemoryBarrierWithGroupSync();
        }
    }
#endif

    // Write result to memory.
    if (globalIdx < gTotalSize)
    {
        gSortedLightMeshBuffer[gTotalSize - globalIdx - 1] = keyValue;
    }
}
