// A single iteration of Bob Jenkins' One-At-A-Time hashing algorithm.
uint hash( uint x ) {
    x += ( x << 10u );
    x ^= ( x >>  6u );
    x += ( x <<  3u );
    x ^= ( x >> 11u );
    x += ( x << 15u );
    return x;
}

// Compound versions of the hashing algorithm I whipped together.
uint hash(uint2 v) { return hash(v.x ^ hash(v.y)); }
uint hash(uint3 v) { return hash(v.x ^ hash(v.y) ^ hash(v.z)); }
uint hash(uint4 v) { return hash(v.x ^ hash(v.y) ^ hash(v.z) ^ hash(v.w)); }

// Construct a float with half-open range [0:1] using low 23 bits.
// All zeroes yields 0.0, all ones yields the next smallest representable value below 1.0.
float floatConstruct(uint m) {
    const uint ieeeMantissa = 0x007FFFFFu; // binary32 mantissa bitmask
    const uint ieeeOne = 0x3F800000u;      // 1.0 in IEEE binary32

    m &= ieeeMantissa; // Keep only mantissa bits (fractional part)
    m |= ieeeOne;      // Add fractional part to 1.0

    float f = asfloat(m); // Range [1:2]
    return f - 1.0;               // Range [0:1]
}

float random(float3 v) 
{ 
    return floatConstruct(hash(asuint(v))); 
}

// Paraboloid projection. We are in right-hand system, the view direction will be -z
float3 paraboloidProjection(float3 pointLightView, float near, float far)
{
    // Because the origin is at 0 the proj-vector matches the vertex-position
    float dist = length(pointLightView);
    // float dist = abs(pointLightView.z);
    float3 pos = pointLightView / dist;

    // calc "normal" on intersection, by adding the reflection-vector(0,0,1) and divide through
    // his z to get the texture coords
    pos.z = 1.0f - pos.z;
    pos.xy /= pos.z;

    // set z for z-buffering
    pos.z = (dist - near) / (far - near); // [0, 1]

    return pos;
}

float3 paraboloidProjection2(float3 pointLightView, float zFar, bool preserveSign)
{
    float3 v = pointLightView;
    float distToCamera = length(pointLightView);
    float signOfV = sign(v.z);

    // paraboloid projection
    v.xyz /= distToCamera;
    v.z = 1.0 - v.z;
    v.xy /= v.z;
    v.z = distToCamera / zFar;
    if (preserveSign)
        v.z *= -signOfV;

    // scale and bias to texcoords
    // v.xy += 1.0;
    // v.xy /= 2.0;

    return v;
}

float3 sphericalMap(float3 posCam, float far) {
    float3 pos = posCam / far;
    float pz = length(pos);
    if (pos.z > 0.0) {
        pz = -1.0;
    }

    pos = normalize(pos);
    float Pi = 4.0 * atan(1.0);
    float theta = acos(-pos.z);
    if (theta > Pi * 0.5) {
        theta = Pi - theta;
    }

    float len = sqrt(pos.x * pos.x + pos.y * pos.y);
    return float3(pos.xy / len * theta / (Pi * 0.5), pz);
}