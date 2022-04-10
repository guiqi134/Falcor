// Reference: https://developer.nvidia.com/gpugems/gpugems3/part-ii-light-and-shadows/chapter-8-summed-area-variance-shadow-maps
import Utils.Debug.PixelDebug;

// TODO: const should later change to defines and pass through CPU side
static const float kMinVariance = 1e-4f;
static const float2 kMinFilterWidth = 0.00001f;
static const float2 kMaxFilterWidth = 0.9f;
static const float2 evsmExponents = float2(5.5f, 2.0f); // TODO: need to be tuned

float linstep(float min, float max, float v)
{
    return clamp((v - min) / (max - min), 0, 1);
}

// Light bleeding reduction
float LBR(float p)
{
    //return smoothstep(kLBRThreshold, 1.0f, p);
    return linstep(_LBR_THRESHOLD, 1.0f, p);
}

float ChebyshevUpperBound(float2 moments, float t)
{
    // Compute variance.
    float variance = moments.y - moments.x * moments.x;
    variance = max(variance, kMinVariance);

    // Compute probabilistic upper bound.
    float d = t - moments.x;
    float p_max = variance / (variance + d * d);

    // One-tailed inequality valid if t > Moments.x
    float visibility = t <= moments.x ? 1.0f : p_max;

    // Light bleeding reduction
    visibility = LBR(visibility);

    return visibility;
}

// -------------------------------
// EVSM methods
// -------------------------------

float2 getEvsmExponents(float depth)
{
    depth = 2.0f * depth - 1.0f;
    float2 expDepth;
    expDepth.x = exp(evsmExponents.x * depth);
    expDepth.y = -exp(-evsmExponents.y * depth);
    return expDepth;
}

// -------------------------------
// SAT methods
// -------------------------------


// Returns coordinates for the four samples surround a given fragment.
// The four samples are compuated start from the bottom-left and do a counter clock-wise:
// BL -> BR -> UR -> UL
float2 getBilCoordsAndWeights(float2 centerCoords, float2 texSize, out float4 weights)
{
    float2 texelSize = 1 / texSize;
    float2 texelCoords = centerCoords * texSize;

    // Compute weights
    weights.xy = frac(texelCoords + 0.5f);
    weights.zw = 1 - weights.xy;
    weights = weights.zxxz * weights.yyww;

    // Compute upper-left pixel coordinates
    return (floor(texelCoords - 0.5f) + 0.5f) * texelSize;
}

float bilinearChebyshev(float4 filterBound, float4 weights, float distance, Texture2D texSAT, SamplerState sampler)
{
    // Sample SAT bilinear and compute the sum

    // Average the sum

    // Bilinear interpolation two moments
    float2 moments;

    return ChebyshevUpperBound(moments, distance);
}

// Use this if hardware bilinear is enabled for the SAT, it causes more precision problems...
float4 sampleSAT(float4 filterBound, float2 texSize, float depth, Texture2D texSAT, SamplerState sampler)
{
    float2 texelSize = 1.f / texSize;

    // Work out normalized coordinates and area
    // We need shift current filter bound one texel to the upper-left, then we can get the right texture coordinate
    //float4 realCoords = filterBound - texelSize.xyxy;
    float4 realCoords = filterBound;
    print(realCoords);
    realCoords = saturate(realCoords);
    //print(realCoords);
    filterBound = saturate(filterBound);
    float2 dims = (filterBound.zw - filterBound.xy) * texSize; // screen space filter area
    //print(dims.x * dims.y);


    // Sample four SAT corners
    // filterBound.xy is upper left, filterBound.zw is lower right.
    float4 nn = texSAT.SampleLevel(sampler, realCoords.xy, 0); // (min, min)
    float4 np = texSAT.SampleLevel(sampler, realCoords.xw, 0); // (min, max)
    float4 pn = texSAT.SampleLevel(sampler, realCoords.zy, 0); // (max, min)
    float4 pp = texSAT.SampleLevel(sampler, realCoords.zw, 0); // (max, max)

    print(nn);
    print(np);
    print(pn);
    print(pp);

    // Compute sum and divide to get average moments
    float4 avg = (pp - pn - np + nn) / (dims.x * dims.y);

    return avg;
}

float vsmShadowContribSAT(float4 moments, float depth)
{
    float visibility = ChebyshevUpperBound(moments.xy, depth);

    // Light bleeding reduction
    visibility = LBR(visibility);

    return visibility;
}

float evsmShadowContribSAT(float4 moments, float depth)
{
    float2 expDepth = getEvsmExponents(depth);

    // Positive & Negative contribution
    float posV = ChebyshevUpperBound(moments.xy, expDepth.x);
    float negV = ChebyshevUpperBound(moments.zw, expDepth.y);

    print(expDepth);
    print(posV);
    print(negV);

    return min(posV, negV);
}
