import Utils.Debug.PixelDebug;

// TODO: define on CPU side
#ifndef NUM_ISM_TEXTURE_ARRAYS
#define NUM_ISM_TEXTURE_ARRAYS 3
#endif

cbuffer convertCB
{
    uint gMaxIsmPerArray;
}

Texture2DArray<uint> gSortedLightsISMsUint[NUM_ISM_TEXTURE_ARRAYS];
RWTexture2DArray<float> gSortedLightsISMs[NUM_ISM_TEXTURE_ARRAYS];

[numthreads(16, 16, 1)]
void uint2FloatCS(uint3 threadID : SV_DispatchThreadID)
{
    uint3 ismTexArrayLoc = uint3(threadID.xy, threadID.z % gMaxIsmPerArray);
    uint whichIsmTexArray = uint(threadID.z / gMaxIsmPerArray);
    gSortedLightsISMs[whichIsmTexArray][ismTexArrayLoc] = asfloat(gSortedLightsISMsUint[whichIsmTexArray][ismTexArrayLoc]);
}
