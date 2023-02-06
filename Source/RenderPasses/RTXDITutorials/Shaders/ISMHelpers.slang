// Reference: https://github.com/TheRealMJP/BakingLab/blob/1a043117506ac5b5bcade5c86d808485f3c70b12/SampleFramework11/v1.02/Graphics/Textures.h#L124
uint getFaceIndex(float3 direction)
{
    // Assert_(texData.NumSlices == 6);

    float maxComponent = max(max(abs(direction.x), abs(direction.y)), abs(direction.z));
    uint faceIdx = 0;
    float2 uv = float2(direction.y, direction.z);

    if (direction.x == maxComponent)
    {
        faceIdx = 0;
        uv = float2(-direction.z, -direction.y) / direction.x;
    }
    else if (-direction.x == maxComponent)
    {
        faceIdx = 1;
        uv = float2(direction.z, -direction.y) / -direction.x;
    }
    else if (direction.y == maxComponent)
    {
        faceIdx = 2;
        uv = float2(direction.x, direction.z) / direction.y;
    }
    else if (-direction.y == maxComponent)
    {
        faceIdx = 3;
        uv = float2(direction.x, -direction.z) / -direction.y;
    }
    else if (direction.z == maxComponent)
    {
        faceIdx = 4;
        uv = float2(direction.x, -direction.y) / direction.z;
    }
    else if (-direction.z == maxComponent)
    {
        faceIdx = 5;
        uv = float2(-direction.x, -direction.y) / -direction.z;
    }

    uv = uv * float2(0.5f, 0.5f) + float2(0.5f, 0.5f);
    return faceIdx;
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

// Find interval using binary search -> std::upper_bound()
uint findInterval(Buffer<uint> targetBuffer, float u)
{
    uint bufferSize;
    targetBuffer.GetDimensions(bufferSize);

    int first = 0;
    uint len = bufferSize;

    while (len > 0)
    {
        uint h = len >> 1;
        uint middle = first + h;
        if (u < targetBuffer[middle])
        {
            len = h;
        }
        else
        {
            first = (int)middle + 1;
            len -= h + 1;
        }
    }

    return clamp(first - 1, 0, bufferSize - 2);
}

// Find interval using binary search -> std::upper_bound()
uint findInterval(Buffer<float> targetBuffer, float u)
{
    uint bufferSize;
    targetBuffer.GetDimensions(bufferSize);

    int first = 0;
    uint len = bufferSize;

    while (len > 0)
    {
        uint h = len >> 1;
        uint middle = first + h;
        if (u < targetBuffer[middle])
        {
            len = h;
        }
        else
        {
            first = (int)middle + 1;
            len -= h + 1;
        }
    }

    return clamp(first - 1, 0, bufferSize - 2);
}

uint2 getLocalLightAndFaceIdx(uint lightOrGlobalFaceIdx, uint sortingRules, uint shadowMapsPerLight)
{
    uint localLightIdx = sortingRules == 2 ? lightOrGlobalFaceIdx / shadowMapsPerLight : lightOrGlobalFaceIdx;
    uint faceIdx = sortingRules == 2 ? lightOrGlobalFaceIdx % shadowMapsPerLight : 0u;

    return uint2(localLightIdx, faceIdx);
}


// TODO: remove all codes below

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
