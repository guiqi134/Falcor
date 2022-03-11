// Reference: https://developer.nvidia.com/gpugems/gpugems3/part-ii-light-and-shadows/chapter-8-summed-area-variance-shadow-maps
import Utils.Debug.PixelDebug;

// TODO: const should later change to defines and pass through CPU side
static const float kMinVariance = 1e-4f;
static const float2 kMinFilterWidth = 0.001f;
static const float2 kMaxFilterWidth = 0.9f;
static const float kLBRThreshold = 0.05f;

float linstep(float min, float max, float v)
{
    return clamp((v - min) / (max - min), 0, 1);
}

// Light bleeding reduction
float LBR(float p)
{
    //return smoothstep(kLBRThreshold, 1.0f, p);
    return linstep(kLBRThreshold, 1.0f, p);
}

float ChebyshevUpperBound(float2 moments, float t)
{
    // One-tailed inequality valid if t > Moments.x
    float p = (t <= moments.x);

    // Compute variance.
    float variance = moments.y - moments.x * moments.x;
    print("variance = ", variance);
    variance = max(variance, kMinVariance);

    // Compute probabilistic upper bound.
    float d = t - moments.x;
    float p_max = variance / (variance + d * d);

    return max(p, p_max);
}

// -------------------------------
// SAT methods
// -------------------------------

float2 getFilterSize(float2 texCoords, float2 dx, float2 dy, float2 texSize, out float2 size)
{
    float2 texelSize = 1.f / texSize;

    // Compute filter size based on derivatives
    size = 2.f * (abs(dx) + abs(dy)) * texSize;
    //sizeD = clamp(sizeD, kMinFilterWidth, kMaxFilterWidth);

    // Force an integer tile size (in pixels) so that bilinear weights are consistent
    //size = round(sizeD) * texelSize; 
    //return (texCoords - 0.5 * (size - texelSize));

    size = float2(0.03f);

    // The upper-left position
    return (texCoords - 0.5 * size);
}

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
float pointChebyshev(float4 filterBound, float2 texSize, float distance, Texture2D texSAT, SamplerState sampler)
{
    float2 texelSize = 1.f / texSize;

    // Work out normalized coordinates and area
    // We need shift current filter bound one texel to the upper-left, then we can get the right texture coordinate
    float4 realCoords = filterBound - texelSize.xyxy;
    float2 dims = (filterBound.zw - filterBound.xy) * texSize; // screen space filter area

    // Handle special case: when realCoords is partially outside the texture
    if (realCoords.x < 0.0f || realCoords.y < 0.0f || realCoords.z > 1.0f || realCoords.w > 1.0f) return 1.0f;

    print(realCoords);
    print(dims);

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
    float2 moments = avg.xy;


    print(moments);
    print(distance);

    return ChebyshevUpperBound(moments, distance);
}

float shadowContribSAT(float2 texCoords, float2 filterSize, float2 shadowMapSize, float distance, Texture2D texSAT, SamplerState sampler)
{
    float2 texelSize = 1 / shadowMapSize;
    filterSize = clamp(filterSize, kMinFilterWidth, kMaxFilterWidth);
    //filterSize = float2(0.05f);

    // Get filter size
    float2 coordsUL = texCoords - 0.5f * filterSize;
    float4 filterBound = coordsUL.xyxy + float4(0.f, 0.f, filterSize.xy);
    print("texCoords = ", texCoords);
    print("filterSize = ", filterSize);
    print("coordsUL = ", coordsUL);

    float visibility = pointChebyshev(filterBound, shadowMapSize, distance, texSAT, sampler);
    //visibility = LBR(visibility);

    print("visibility = ", visibility);

    return visibility;
}
