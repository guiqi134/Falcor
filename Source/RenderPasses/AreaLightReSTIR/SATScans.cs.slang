import Utils.Debug.PixelDebug;

static const uint kBlockSize = 1024;

Texture2D gInput;
RWTexture2D gOutput;

groupshared float4 partialSums[kBlockSize * 2]; // ping-pong shared memory

void naiveScan(uint2 threadIdx)
{
    uint pOut = 0, pIn = 1;
#ifdef _ROW_SCAN
    uint threadIdx1D = threadIdx.x;
#else
    uint threadIdx1D = threadIdx.y;
#endif

    // Each thread read its input from device memory to shared memory
    partialSums[pOut * kBlockSize + threadIdx1D] = gInput[threadIdx];
    GroupMemoryBarrierWithGroupSync();

    // Iterate log(n) times
    for (uint stride = 1; stride < kBlockSize; stride <<= 1)
    {
        // Swap double buffer indices
        pOut = 1 - pOut;
        pIn = 1 - pOut;
        uint outputIdx = pOut * kBlockSize + threadIdx1D;
        uint inputIdx = pIn * kBlockSize + threadIdx1D;

        // Add pairs of elements, stride elements apart
        if (threadIdx1D >= stride)
            partialSums[outputIdx] = partialSums[inputIdx] + partialSums[inputIdx - stride];
        else
            partialSums[outputIdx] = partialSums[inputIdx];

        // Wait all thread writes to output
        GroupMemoryBarrierWithGroupSync();
    }

    gOutput[threadIdx] = partialSums[pOut * kBlockSize + threadIdx1D];
}

#ifdef _ROW_SCAN
[numthreads(kBlockSize, 1, 1)]
#else
[numthreads(1, kBlockSize, 1)]
#endif
void main(uint3 DTid : SV_DispatchThreadID, uint GI : SV_GroupIndex) // GI: thread 1D-index in one group
{
    uint2 threadIdx = DTid.xy;
    printSetPixel(threadIdx);

    naiveScan(threadIdx);
}


