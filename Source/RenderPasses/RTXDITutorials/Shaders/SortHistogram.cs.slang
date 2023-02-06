// Copyright (c) Microsoft. All rights reserved.
// This code is licensed under the MIT License (MIT).
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
// Developed by Minigraph
//
// Author:  James Stanard

import Utils.Debug.PixelDebug;

cbuffer sortCB
{
    uint gTotalElementCount; ///< Total number of elements.

    // A sort key that will end up at the end of the list; to be used to pad
    // lists in LDS (always 2048 items).
    //   Descending:  0x00000000
    //   Ascending:   0xffffffff
    // Also used by the ShouldSwap() function to invert ordering.
    uint gNullItem;
};

RWBuffer<uint2> gSortBuffer;

groupshared uint sKeys[2048];
groupshared uint sValues[2048];

// Takes Value and widens it by one bit at the location of the bit
// in the mask.  A one is inserted in the space.  OneBitMask must
// have one and only one bit set.
// - If the origin location is 0, then it equals to value + oneBitMask
// - If the origin location is 1, then it equals to value
uint InsertOneBit(uint Value, uint OneBitMask)
{
    uint Mask = OneBitMask - 1;
    return (Value & ~Mask) << 1 | (Value & Mask) | OneBitMask;
}

// Determines if two sort keys should be swapped in the list.  gNullItem is
// either 0 or 0xffffffff.  XOR with the gNullItem will either invert the bits
// (effectively a negation) or leave the bits alone.  When the the gNullItem is
// 0, we are sorting descending, so when A < B, they should swap.  For an
// ascending sort, ~A < ~B should swap.
bool ShouldSwap(uint A, uint B)
{
    return (A ^ gNullItem) < (B ^ gNullItem);
}

void fillSortKeys(uint elementIdx, bool innerPass = false)
{
    uint sharedIdx = elementIdx & 2047;

    // Unused elements must sort to the end
    if (innerPass || elementIdx < gTotalElementCount)
    {
        uint2 keyValuePair = gSortBuffer[elementIdx];
        sKeys[sharedIdx] = keyValuePair.x;
        sValues[sharedIdx] = keyValuePair.y;
    }
    else
    {
        sValues[sharedIdx] = gNullItem;
    }
}

void storeKeyValuePair(uint elementIdx)
{
    uint sharedIdx = elementIdx & 2047;

    if (elementIdx < gTotalElementCount)
        gSortBuffer[elementIdx] = uint2(sKeys[sharedIdx], sValues[sharedIdx]);
}

[numthreads(1024, 1, 1)]
void preSortCS(uint3 groupID: SV_GroupID, uint groupIdx: SV_GroupIndex)
{
    // Item index of the start of this group. This is not the thread global index!!!
    const uint elementGroupStart = groupID.x * 2048;
    const uint elementGlobalIdx = elementGroupStart + groupIdx;

    // Pass global data to each group's shared memory. Each thread is going to handle two data location
    fillSortKeys(elementGlobalIdx);
    fillSortKeys(elementGlobalIdx + 1024);

    GroupMemoryBarrierWithGroupSync();

    uint k;

    // This is better unrolled because it reduces ALU and because some
    // architectures can load/store two LDS items in a single instruction
    // as long as their separation is a compile-time constant.
    [unroll]
    for (k = 2; k <= 2048; k <<= 1)
    {
        for (uint j = k / 2; j > 0; j /= 2)
        {
            uint index2 = InsertOneBit(groupIdx, j);
            uint index1 = index2 ^ (k == 2 * j ? k - 1 : j);

            uint A = sValues[index1];
            uint B = sValues[index2];

            if (ShouldSwap(A, B))
            {
                // Swap the values
                sValues[index1] = B;
                sValues[index2] = A;

                // Then swap the keys
                A = sKeys[index1];
                B = sKeys[index2];
                sKeys[index1] = B;
                sKeys[index2] = A;
            }

            GroupMemoryBarrierWithGroupSync();
        }
    }

    // Write sorted results to memory
    storeKeyValuePair(elementGlobalIdx);
    storeKeyValuePair(elementGlobalIdx + 1024);
}

[numthreads(1024, 1, 1)]
void innerSortCS(uint3 groupID: SV_GroupID, uint groupIdx: SV_GroupIndex)
{
    // Item index of the start of this group
    const uint elementGroupStart = groupID.x * 2048;
    const uint elementGlobalIdx = elementGroupStart + groupIdx;

    // Load from memory into LDS to prepare sort. We don't need to fill nullItem any more, because elementIdx always < total element count
    fillSortKeys(elementGlobalIdx, true);
    fillSortKeys(elementGlobalIdx + 1024, true);

    GroupMemoryBarrierWithGroupSync();

    [unroll]
    for (uint j = 1024; j > 0; j /= 2)
    {
        uint index2 = InsertOneBit(groupIdx, j);
        uint index1 = index2 ^ j;

        uint A = sValues[index1];
        uint B = sValues[index2];

        if (ShouldSwap(A, B))
        {
            // Swap the values
            sValues[index1] = B;
            sValues[index2] = A;

            // Then swap the keys
            A = sKeys[index1];
            B = sKeys[index2];
            sKeys[index1] = B;
            sKeys[index2] = A;
        }

        GroupMemoryBarrierWithGroupSync();
    }

    // Write sorted results to memory
    storeKeyValuePair(elementGlobalIdx);
    storeKeyValuePair(elementGlobalIdx + 1024);
}

cbuffer outerCB
{
    uint k; // k >= 4096
    uint j; // j >= 2048 && j < k
}

[numthreads(1024, 1, 1)]
void outerSortCS(uint3 threadId: SV_DispatchThreadID)
{
    // Form unique index pair from dispatch thread ID
    uint index2 = InsertOneBit(threadId.x, j);
    uint index1 = index2 ^ (k == 2 * j ? k - 1 : j);

    if (index2 >= gTotalElementCount)
        return;

    uint2 A = gSortBuffer[index1];
    uint2 B = gSortBuffer[index2];

    if (ShouldSwap(A.y, B.y))
    {
        gSortBuffer[index1] = B;
        gSortBuffer[index2] = A;
    }
}
