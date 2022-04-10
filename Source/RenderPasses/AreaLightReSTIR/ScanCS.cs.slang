import Utils.Debug.PixelDebug;

Texture2D Input;
RWTexture2D gTemp;
RWTexture2D Result;

groupshared float4 partialSums[groupthreads * 2];

// If shadow map size = n, then block size = n/2. So each block will have n/2 threads, and each thread will handle two locations (outputs)
// in the shadow map. Thus the shared memory size is n.
void CSScan(uint2 DTid, uint2 Gid, uint2 GTid)
{
#ifdef _ROW_SCAN
    uint groupThreadIdx = GTid.x;
    uint2 texCoords = uint2(2 * DTid.x, DTid.y);
    uint2 texCoordsPlueOne = uint2(2 * DTid.x + 1, DTid.y);
#else
    uint groupThreadIdx = GTid.y;
    uint2 texCoords = uint2(DTid.x, 2 * DTid.y);
    uint2 texCoordsPlueOne = uint2(DTid.x, 2 * DTid.y + 1);
#endif

    print(groupThreadIdx);
    print(Gid);
    print(texCoords);
    print(texCoordsPlueOne);

    print(Input[texCoords]);
    print(Input[texCoordsPlueOne]);

    // Load data from device to shared memeory
    partialSums[2 * groupThreadIdx] = Input[texCoords];
    partialSums[2 * groupThreadIdx + 1] = Input[texCoordsPlueOne];


    // Up sweep
    [unroll]
    for (uint stride = 1; stride <= groupthreads; stride <<= 1)
    {
        GroupMemoryBarrierWithGroupSync();
        uint index = (groupThreadIdx + 1) * stride * 2 - 1;
        if (index < 2 * groupthreads)
            partialSums[index] += partialSums[index - stride];
    }

    // Down sweep
    [unroll]
    for (int stride = groupthreads / 2; stride > 0; stride >>= 1)
    {
        GroupMemoryBarrierWithGroupSync();
        uint index = (groupThreadIdx + 1) * stride * 2 - 1;
        if ((index + stride) < 2 * groupthreads)
            partialSums[index + stride] += partialSums[index];
    }
    GroupMemoryBarrierWithGroupSync();

    // Write result back to output 
    Result[texCoords] = partialSums[2 * groupThreadIdx];
    Result[texCoordsPlueOne] = partialSums[2 * groupThreadIdx + 1];
    GroupMemoryBarrierWithGroupSync();

    print(Result[texCoords]);
    print(Result[texCoordsPlueOne]);

    // Write each block's last value (the sum of each block) to temp output
    if (groupThreadIdx == groupthreads - 1)
    {
        gTemp[Gid] = partialSums[2 * groupthreads - 1];
    }
}


// scan in each group (one thread handles two elements)
#ifdef _ROW_SCAN
[numthreads(groupthreads, 1, 1)]
#else
[numthreads(1, groupthreads, 1)]
#endif
void CSScanInBucket(uint3 DTid : SV_DispatchThreadID, uint3 Gid : SV_GroupID, uint3 GTid : SV_GroupThreadID)
{
    //printSetPixel(DTid.xy);
    CSScan(DTid.xy, Gid.xy, GTid.xy);
}

// record and scan the sum of each bucket
#ifdef _ROW_SCAN
[numthreads(groupthreads, 1, 1)] // 4k shadow size -> kBlockSize = 2
#else
[numthreads(1, groupthreads, 1)]
#endif
void CSScanBucketResult(uint3 DTid : SV_DispatchThreadID, uint3 Gid : SV_GroupID, uint3 GTid : SV_GroupThreadID)
{
    //printSetPixel(DTid.xy);
    CSScan(DTid.xy, Gid.xy, GTid.xy);
}

// add the bucket scanned result to each bucket to get the final result
#ifdef _ROW_SCAN
[numthreads(groupthreads, 1, 1)]
#else
[numthreads(1, groupthreads, 1)]
#endif
void CSScanAddBucketResult(uint3 DTid : SV_DispatchThreadID, uint3 Gid : SV_GroupID, uint3 GTid : SV_GroupThreadID)
{
#ifdef _ROW_SCAN
    uint2 blockIdx = uint2(Gid.x / 2 - 1, Gid.y);
#else
    uint2 blockIdx = uint2(Gid.x, Gid.y / 2 - 1);
#endif

    //printSetPixel(DTid.xy);

    Result[DTid.xy] = Result[DTid.xy] + Input[blockIdx];
}
