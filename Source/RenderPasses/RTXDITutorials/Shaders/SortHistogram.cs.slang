// --------------------------------------------------------------------
// 2. Sort the input buffer from previous counting result
// --------------------------------------------------------------------
#include "Utils/NVAPI.slangh" // We need this to get shuffle-xor operations.
import Utils.Debug.PixelDebug;

#define GROUP_SIZE 256
#define CHUNK_SIZE 128 // Must be a power-of-two. Set this value larger than gTotalLightMeshCount

#if (NV_WARP_SIZE != 32)
#error Kernel assumes warp size 32
#endif

cbuffer sortCB
{
    uint gTotalLightMeshCount; ///< Total number of elements.
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
        // The shuffle function seems won't change the value in another thread
        uint key_ixj = NvShflXor(key, j);
        uint value_ixj = NvShflXor(value, j);

        // print("sorting partner index = ", i ^ j);
        print("keyValue = ", uint2(key, value));
        print("keyValue_ixj = ", uint2(key_ixj, value_ixj));

        // Decide whether to swap.
        // i & j operation is to get the bit value in log2(j)-th location
        bool pred = (((i & j) == 0) != dir) == value < value_ixj;
        print("pred = ", pred);

        if (pred && value != value_ixj)
        {
            key = key_ixj;
            value = value_ixj;
        }

        // uint key_ixj2 = NvShflXor(key, j);
        // uint value_ixj2 = NvShflXor(value, j);
        // print("keyValue_ixj2 = ", uint2(key_ixj2, value_ixj2));

    }
}

// The kernel is currently written for a 1:1 mapping between elements to sort and threads.
[numthreads(GROUP_SIZE, 1, 1)]
void sortLightMeshHistogram(uint3 threadId: SV_DispatchThreadID, uint3 groupID: SV_GroupID, uint groupIdx: SV_GroupIndex)
{
    // printSetPixel(threadId.xy);

    const uint group = groupID.x; // Sequential group index.
    const uint thid = groupIdx;   // Local thread index in group (range 0..GROUP_SIZE-1).

    const uint globalIdx = group * GROUP_SIZE + thid; // Global element index in gData

    const uint N = CHUNK_SIZE;          // Number of elements per chunk to sort. Must be a power-of-two.
    const uint i = globalIdx & (N - 1); // i = local index of element in chunk, range [0,N).

    // Load value from memory.
    // Out-of-bounds elements are set to UINT_MAX (-1) to be placed last and allow data that is not a multiple of chunk size.
    uint2 keyValue = uint2(globalIdx, -1);
    if (globalIdx < gTotalLightMeshCount)
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
        print("", 1);
        print("dir = ", dir);

        // We ping-pong data in shared memory between adjacent addresses, using offset = {0, 1} to denote which one.
        uint offset = 0;

        // Minor steps for iterations j = {k/2, k/4, ..., 32} in shared memory.
        for (uint j = k >> 1; j >= 32; j >>= 1)
        {
            // Get sorting partner.
            uint2 keyValue_ixj = uint2(sKeys[(thid ^ j) * 2 + offset], sValues[(thid ^ j) * 2 + offset]);

            // print("keyValue_ixj = ", keyValue_ixj);

            // Decide whether to swap. See comments in bitonicSortInWarp().
            bool pred = (((i & j) == 0) != dir) == keyValue.y <= keyValue_ixj.y;
            if (pred) keyValue = keyValue_ixj;

            // print("keyValue = ", keyValue);

            // Store result for next minor step (except for last iteration). Write to offset address to avoid race condition.
            if (j > 32)
            {
                sKeys[thid * 2 + (offset ^ 1)] = keyValue.x;
                sValues[thid * 2 + (offset ^ 1)] = keyValue.y;

                GroupMemoryBarrierWithGroupSync();
                offset ^= 1;
            }
        }

        print("keyValue = ", keyValue);

        // Minor steps for iterations j = {16, 8, ..., 1} in warp.
        uint jStart = min(k >> 1, 16);
        bitonicSortInWarp(keyValue.x, keyValue.y, i, jStart, dir);

        // print("keyValue = ", keyValue);

        // Store result for major step (except for last iteration).
        if (k < N)
        {
            sKeys[thid * 2] = keyValue.x;
            sValues[thid * 2] = keyValue.y;
            GroupMemoryBarrierWithGroupSync();
        }
    }
#endif

    print("keyValue = ", keyValue);

    // Write result to memory.
    if (globalIdx < gTotalLightMeshCount)
    {
        gSortedLightMeshBuffer[gTotalLightMeshCount - globalIdx - 1] = keyValue;
        // gSortedLightMeshBuffer[globalIdx] = keyValue;
    }
}