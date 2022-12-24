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
