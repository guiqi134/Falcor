#include "../HostDeviceSharedDefinitions.h"

import Utils.Debug.PixelDebug;
import Utils.Sampling.TinyUniformSampleGenerator;

// TODO: lots of function below can be optimized using Slang's interface and generic feature

// Matrix types and operations
static const float4x4 identity4x4 = {
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0,
};

static const float3x3 identity3x3 = {
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
};

inline float4x4 translation(float x, float y, float z)
{
    return float4x4(
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        x, y, z, 1.0
    );
}

void printMatrix4x4(float4x4 matrix, String msg)
{
    for (uint i = 0; i < 4; i++)
        print(msg, matrix[i]);
}

void printMatrix3x3(float3x3 matrix, String msg)
{
    for (uint i = 0; i < 3; i++)
        print(msg, matrix[i]);
}

// Compute view matrix
float4x4 lookAtRH(const float3 eye, const float3 center, const float3 up)
{
    float3 forward = normalize(eye - center);
    float3 right = normalize(cross(up, forward));
    float3 newUp = cross(forward, right);

    return float4x4(
        right.x, newUp.x, forward.x, 0.0,
        right.y, newUp.y, forward.y, 0.0,
        right.z, newUp.z, forward.z, 0.0,
        -dot(right, eye), -dot(newUp, eye), -dot(forward, eye), 1.0
    );
}

uint getClosestFace(float3 targetDir, LightShadowMapData lightShadowData)
{
    float maxCosValue = 0.0f; // the delta angle should < 90 (45) degree
    uint closestFaceIdx = 0u;
    for (uint face = 0; face < lightShadowData.numShadowMaps; face++)
    {
        float3 viewDirection = -lightShadowData.lightFaceData[face].viewMat[2].xyz;
        float cosValue = dot(targetDir, normalize(viewDirection));
        if (cosValue > maxCosValue)
        {
            maxCosValue = cosValue;
            closestFaceIdx = face;
        }
    }

    // print("maxCosValue = ", maxCosValue);
    // print("maxCosValue angle = ", acos(maxCosValue));
    // print("closestFaceIdx = ", closestFaceIdx);

    return closestFaceIdx;
}

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

    // uv = uv * float2(0.5f, 0.5f) + float2(0.5f, 0.5f);
    return faceIdx;
}

/** D3DXMatrixPerspectiveFovRH (be care, it is row major, so it is multiplied in right!!!):
    xScale     0          0              0
    0        yScale       0              0
    0          0      zf/(zn-zf)        -1
    0          0      zn*zf/(zn-zf)      0

    Transform from NDC to camera space
*/
float linearizeDepth(float depthNDC, float near, float far)
{
    return abs((near * far) / (depthNDC * (far - near) - far)); // return positive depth
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

uint findInterval(Buffer<uint2> targetBuffer, float u)
{
    uint bufferSize;
    targetBuffer.GetDimensions(bufferSize);

    int first = 0;
    uint len = bufferSize;

    while (len > 0)
    {
        uint h = len >> 1;
        uint middle = first + h;
        if (u < targetBuffer[middle].x)
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

uint2 getLocalLightIdAndFaceIdx(uint globalFaceID, uint firstSpotLightID)
{
    uint localLightID = globalFaceID < firstSpotLightID * kShadowMapsPerPointLight ? globalFaceID / kShadowMapsPerPointLight :
        globalFaceID - firstSpotLightID * kShadowMapsPerPointLight + firstSpotLightID;
    uint faceIdx = globalFaceID < firstSpotLightID * kShadowMapsPerPointLight ? globalFaceID % kShadowMapsPerPointLight : 0u;

    return uint2(localLightID, faceIdx);
}

// ISM point sample data structure on GPU
struct IsmPointSample
{
    float3 pos;
    uint selectedLightID;
    uint instanceID;
};

static const uint kIsmPointSample_LightIdShift = 0;
static const uint kIsmPointSample_InstanceIdShift = 16;

PackedIsmPointSample packIsmPointSample(IsmPointSample pointSample)
{
    PackedIsmPointSample packedSample;
    // packedSample.pos = float16_t3(pointSample.pos);
    packedSample.pos = pointSample.pos;
    packedSample.packedLightInstanceID = (pointSample.selectedLightID & 0xFFFF) | ((pointSample.instanceID & 0xFFFF) << 16);

    return packedSample;
}

IsmPointSample unpackIsmPointSample(PackedIsmPointSample packedSample)
{
    IsmPointSample sample;
    sample.pos = float3(packedSample.pos);
    sample.selectedLightID = packedSample.packedLightInstanceID & 0xFFFF;
    sample.instanceID = packedSample.packedLightInstanceID >> 16;

    return sample;
}




// TODO: remove all codes below

// float3 sphericalMap(float3 posCam, float far) {
//     float3 pos = posCam / far;
//     float pz = length(pos);
//     if (pos.z > 0.0) {
//         pz = -1.0;
//     }

//     pos = normalize(pos);
//     float Pi = 4.0 * atan(1.0);
//     float theta = acos(-pos.z);
//     if (theta > Pi * 0.5) {
//         theta = Pi - theta;
//     }

//     float len = sqrt(pos.x * pos.x + pos.y * pos.y);
//     return float3(pos.xy / len * theta / (Pi * 0.5), pz);
// }

float4x4 inverse(float4x4 m)
{
    float n11 = m[0][0], n12 = m[1][0], n13 = m[2][0], n14 = m[3][0];
    float n21 = m[0][1], n22 = m[1][1], n23 = m[2][1], n24 = m[3][1];
    float n31 = m[0][2], n32 = m[1][2], n33 = m[2][2], n34 = m[3][2];
    float n41 = m[0][3], n42 = m[1][3], n43 = m[2][3], n44 = m[3][3];

    float t11 = n23 * n34 * n42 - n24 * n33 * n42 + n24 * n32 * n43 - n22 * n34 * n43 - n23 * n32 * n44 + n22 * n33 * n44;
    float t12 = n14 * n33 * n42 - n13 * n34 * n42 - n14 * n32 * n43 + n12 * n34 * n43 + n13 * n32 * n44 - n12 * n33 * n44;
    float t13 = n13 * n24 * n42 - n14 * n23 * n42 + n14 * n22 * n43 - n12 * n24 * n43 - n13 * n22 * n44 + n12 * n23 * n44;
    float t14 = n14 * n23 * n32 - n13 * n24 * n32 - n14 * n22 * n33 + n12 * n24 * n33 + n13 * n22 * n34 - n12 * n23 * n34;

    float det = n11 * t11 + n21 * t12 + n31 * t13 + n41 * t14;
    float idet = 1.0f / det;

    float4x4 ret;

    ret[0][0] = t11 * idet;
    ret[0][1] = (n24 * n33 * n41 - n23 * n34 * n41 - n24 * n31 * n43 + n21 * n34 * n43 + n23 * n31 * n44 - n21 * n33 * n44) * idet;
    ret[0][2] = (n22 * n34 * n41 - n24 * n32 * n41 + n24 * n31 * n42 - n21 * n34 * n42 - n22 * n31 * n44 + n21 * n32 * n44) * idet;
    ret[0][3] = (n23 * n32 * n41 - n22 * n33 * n41 - n23 * n31 * n42 + n21 * n33 * n42 + n22 * n31 * n43 - n21 * n32 * n43) * idet;

    ret[1][0] = t12 * idet;
    ret[1][1] = (n13 * n34 * n41 - n14 * n33 * n41 + n14 * n31 * n43 - n11 * n34 * n43 - n13 * n31 * n44 + n11 * n33 * n44) * idet;
    ret[1][2] = (n14 * n32 * n41 - n12 * n34 * n41 - n14 * n31 * n42 + n11 * n34 * n42 + n12 * n31 * n44 - n11 * n32 * n44) * idet;
    ret[1][3] = (n12 * n33 * n41 - n13 * n32 * n41 + n13 * n31 * n42 - n11 * n33 * n42 - n12 * n31 * n43 + n11 * n32 * n43) * idet;

    ret[2][0] = t13 * idet;
    ret[2][1] = (n14 * n23 * n41 - n13 * n24 * n41 - n14 * n21 * n43 + n11 * n24 * n43 + n13 * n21 * n44 - n11 * n23 * n44) * idet;
    ret[2][2] = (n12 * n24 * n41 - n14 * n22 * n41 + n14 * n21 * n42 - n11 * n24 * n42 - n12 * n21 * n44 + n11 * n22 * n44) * idet;
    ret[2][3] = (n13 * n22 * n41 - n12 * n23 * n41 - n13 * n21 * n42 + n11 * n23 * n42 + n12 * n21 * n43 - n11 * n22 * n43) * idet;

    ret[3][0] = t14 * idet;
    ret[3][1] = (n13 * n24 * n31 - n14 * n23 * n31 + n14 * n21 * n33 - n11 * n24 * n33 - n13 * n21 * n34 + n11 * n23 * n34) * idet;
    ret[3][2] = (n14 * n22 * n31 - n12 * n24 * n31 - n14 * n21 * n32 + n11 * n24 * n32 + n12 * n21 * n34 - n11 * n22 * n34) * idet;
    ret[3][3] = (n12 * n23 * n31 - n13 * n22 * n31 + n13 * n21 * n32 - n11 * n23 * n32 - n12 * n21 * n33 + n11 * n22 * n33) * idet;

    return ret;
}
