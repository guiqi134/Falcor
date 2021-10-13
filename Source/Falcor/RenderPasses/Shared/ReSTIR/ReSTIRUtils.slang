import Utils.Sampling.SampleGenerator;

// WRS (Algo. 2)
// Size: 24B
struct Reservoir
{
    uint y;
    float2 uv; 
    float wSum;
    uint M;
    float W;

    [mutating]
    bool update(uint xi, float2 uv, float wi, inout SampleGenerator sg)
    {
        wSum += wi;
        M++;
        if (sampleNext1D(sg) < wi / wSum)
        {
            y = xi;
            uv = uv;
            return true;
        }
        else
            return false;
    }
};

RWStructuredBuffer<Reservoir> gReservoirs;

uint getBufferIndex(uint2 pixelPos, uint2 dims)
{
    return dims.x * pixelPos.y + pixelPos.x;
}
