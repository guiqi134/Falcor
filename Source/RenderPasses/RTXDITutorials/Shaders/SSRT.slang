import Utils.Debug.PixelDebug;

cbuffer SSRT_CB
{
    float4x4 gWorldToCamera;
    float4x4 gCameraToClip;
    uint gVisibilityMode; // 0: Shadow Ray, 1: SSRT

    float2 gCsRayOriginLenEpsilon;
    float2 gScreenSize;
    float gNearPlaneZ;
    float gCsZThickness;
    float gStride;
    float gJitterFraction;
    float gMaxSteps;
    uint gLayers;
    bool gClipToFrustum;
    bool gShadowRayForShading;
    SamplerState gPointSampler;
}

Texture2D gCameraSpaceZBuffer; // Current or previous z-buffer
RWTexture2D gDebug;

void swap(inout float a, inout float b)
{
    float temp = a;
    a = b;
    b = temp;
}

float distanceSquared(float2 a, float2 b) 
{
    a -= b;
    return dot(a, a);
}

float4 toHomegeneousScreenSpace(float4 clipCoords, float2 screenSize)
{
    return clipCoords * float4(screenSize, 1.0f, 1.0f);
}


/** Screen-space ray tracing from Morgan McGuire and Michael Mara, Efficient GPU Screen-Space Ray Tracing, Journal of Computer Graphics Techniques (JCGT), vol. 3, no. 4, 73-85, 2014
    Origin source code is in https://jcgt.org/published/0003/04/04/

    \param csOrigin Camera-space ray origin, which must be within the view volume and must have z < -0.01 and project within the valid screen rectangle

    \param csDirection Unit length camera-space ray direction

    \param cameraToHomogeneous A projection matrix that maps to pixel coordinates (not [-1, +1] normalized device coordinates), so this needs two transformation matrix:
     perspective projection matrix and viewport scale matrix (note: d3d flips Y axis up and down)

    \param csZBuffer The depth or camera-space Z buffer, depending on the value of \a csZBufferIsHyperbolic

    \param gScreenSize Dimensions of csZBuffer

    \param csZThickness Camera space thickness to ascribe to each pixel in the depth buffer
    
    \param csZBufferIsHyperbolic True if csZBuffer is an OpenGL depth buffer, false (faster) if
     csZBuffer contains (negative) "linear" camera space z values. Const so that the compiler can evaluate the branch based on it at compile time

    \param nearPlaneZ Negative number

    \param stride Step in horizontal or vertical pixels between samples. This is a float
     because integer math is slow on GPUs, but should be set to an integer >= 1

    \param jitterFraction  Number between 0 and 1 for how far to bump the ray in stride units
     to conceal banding artifacts -> default as 0.0f

    \param maxSteps Maximum number of iterations. Higher gives better images but may be slow

    \param maxRayTraceDistance Maximum camera-space distance to trace before returning a miss. For shadow ray, we need to compute this value 
     instead of manually set it

    \param hitPixel Pixel coordinates of the first intersection with the scene

    \return true is visible, false is occluded
*/
bool traceScreenSpaceShadowRay(
    float3 rayOrigin, 
    float3 rayDirection, // normalized
    float maxRayTraceDistance,
)
{
    // Transform to camera space
    float3 csOrigin = mul(float4(rayOrigin, 1.0f), gWorldToCamera).xyz;
    float3 csDirection = mul(float4(rayDirection, 0.0f), gWorldToCamera).xyz; // Note: direction needs to be 0.0f for w

    print("maxRayTraceDistance = ", maxRayTraceDistance);

    // Analog to trace shadow ray, we need to include a small bias to the ray's origin and max distance (end point)
    csOrigin += min(gCsRayOriginLenEpsilon.x, maxRayTraceDistance) * csDirection; // cannot larger than ray length
    maxRayTraceDistance = max(maxRayTraceDistance - gCsRayOriginLenEpsilon.x - gCsRayOriginLenEpsilon.y, 0.0f); // cannot be negative

    // Clip ray to a near plane in 3D (doesn't have to be *the* near plane, although that would be a good idea)
    // Note: all camera space z value is in negative measure
    float rayLength = ((csOrigin.z + csDirection.z * maxRayTraceDistance) > gNearPlaneZ) ?
                        (gNearPlaneZ - csOrigin.z) / csDirection.z : // use the ray equation to solve value t to perform clipping
                        maxRayTraceDistance;
	float3 csEndPoint = csDirection * rayLength + csOrigin;

    print("rayLength = ", rayLength);
    print("csOrigin = ", csOrigin);
    print("csEndPoint = ", csEndPoint);
    print("csDireciton = ", csDirection);
    // print("rayLength = ", rayLength);

    // Project into clip space
    float4 H0 = mul(float4(csOrigin, 1.0), gCameraToClip);
    float4 H1 = mul(float4(csEndPoint, 1.0), gCameraToClip);

    // There are a lot of divisions by w that can be turned into multiplications at some minor precision loss...and we need to interpolate these 1/w values anyway.
    // Because the caller was required to clip to the near plane, this homogeneous division (projecting from 4D to 2D) is guaranteed to succeed. 
    float k0 = 1.0 / H0.w; // LH system -> w = -z
    float k1 = 1.0 / H1.w;

    print("k0 = ", k0);
    print("k1 = ", k1);

    // Switch the original points to values that interpolate linearly in 2D.
    // Note: the "Q" notation from the paper refers to the camera space point without multipling with "k" and the z value in Q is in the range [0, 1]  
    float3 Q0 = csOrigin * k0; 
    float3 Q1 = csEndPoint * k1;

    print("Q0 = ", Q0);
    print("Q1 = ", Q1);

    // Clip space -> NDC -> Screen-space endpoints 
    float2 P0 = (H0.xy * k0  * float2(0.5f, -0.5f) + float2(0.5f)) * gScreenSize; 
    float2 P1 = (H1.xy * k1  * float2(0.5f, -0.5f) + float2(0.5f)) * gScreenSize; 

    // [Optional clipping to frustum sides here]
    if (gClipToFrustum)
    {
        float xMin = 0.5f, xMax = gScreenSize.x - 0.5f;
        float yMin = 0.5f, yMax = gScreenSize.y - 0.5f;
        float alpha = 0.0f;
        
        // Assume P0 is in the viewport (P1 - P0 is never zero when clipping)
        if (P1.y > yMax || P1.y < yMin)
            alpha = (P1.y - ((P1.y > yMax) ? yMax : yMin)) / (P1.y - P0.y);

        if (P1.x > xMax || P1.x < xMin)
            alpha = max(alpha, (P1.x - ((P1.x > xMax) ? xMax : xMin)) / (P1.x - P0.x));

        P1 = lerp(P1, P0, alpha); 
        k1 = lerp(k1, k0, alpha); 
        Q1 = lerp(Q1, Q0, alpha);
    }

    // Initialize to off screen
    float2 hitPixel = float2(-1.0, -1.0); // This actually is int2
    int hitLayer = -1; 

    // If the line is degenerate, make it cover at least one pixel to avoid handling zero-pixel extent as a special case later
    P1 += float2((distanceSquared(P0, P1) < 0.0001) ? 0.01 : 0.0);

    float2 delta = P1 - P0;

    // Permute so that the primary iteration is in x to reduce large branches later
    bool permute = false;
	if (abs(delta.x) < abs(delta.y)) 
    {
		// More-vertical line. Create a permutation that swaps x and y in the output
		permute = true;

        // Directly swizzle the inputs
		delta = delta.yx;
		P1 = P1.yx;
		P0 = P0.yx;
	}

    print("permute = ", permute);
    print("P0 = ", P0);
    print("P1 = ", P1);
    print("delta = ", delta);
    
	// From now on, "x" is the primary iteration direction and "y" is the secondary one

    float screenStepDirection = sign(delta.x); // -1 or 1
    float invdx = screenStepDirection / delta.x; // Transform to +x direction
    float2 dP = float2(screenStepDirection, invdx * delta.y); // e.g. dP = (1, y/x), it is the differential of x, y in Eq.1
    print("dP = ", dP);

    // Track the derivatives of Q and k
    float3 dQ = (Q1 - Q0) * invdx;
    float dk = (k1 - k0) * invdx;

    print("dQ = ", dQ);
    print("dk = ", dk);

    // Scale derivatives by the desired pixel stride
	dP *= gStride; 
    dQ *= gStride; 
    dk *= gStride;

    // Offset the starting values by the jitter fraction
	P0 += dP * gJitterFraction; 
    Q0 += dQ * gJitterFraction; 
    k0 += dk * gJitterFraction;

    print("after jittering P0 = ", P0);

	// Slide P from P0 to P1, (now-homogeneous) Q from Q0 to Q1, and k from k0 to k1
    float3 Q = Q0;
    float k = k0;

	// We track the ray depth at +/- 1/2 pixel to treat pixels as clip-space solid voxels. Because the depth at -1/2 for a given pixel will be the same as at 
	// +1/2 for the previous iteration, we actually only have to compute one value per iteration.
	float prevZMaxEstimate = csOrigin.z;
    float stepCount = 0.0;
    float rayZMax = prevZMaxEstimate, rayZMin = prevZMaxEstimate;
    float4 sceneZMax;

    // P1.x is never modified after this point, so pre-scale it by the step direction for a signed comparison.
    float end = P1.x * screenStepDirection;
    print("end = ", end);

    // We only advance the z field of Q in the inner loop, since Q.xy is never used until after the loop terminates.

    float2 P = P0;
    for (; ((P.x * screenStepDirection) <= end) && (stepCount < gMaxSteps);
        P += dP, Q.z += dQ.z, k += dk, stepCount += 1.0) // TODO: remove Q.z and dQ.z?
    {
        
        // Get the true pixel coordinate (not normalized)
		hitPixel = permute ? P.yx : P;

        // The depth range that the ray covers within this loop iteration. Assume that the ray is moving in increasing z
        // and swap if backwards.  Because one end of the interval is shared between adjacent iterations, we track the previous
        // value and then swap as needed to ensure correct ordering
        rayZMin = prevZMaxEstimate;

        // Compute the value at 1/2 pixel into the future
        rayZMax = (Q.z + dQ.z * 0.5) / (k + dk * 0.5); // by dividing k, we transform back to the camera space (z is negative)
		prevZMaxEstimate = rayZMax;
        if (rayZMin > rayZMax) swap(rayZMin, rayZMax);

        // Camera-space z of the background. We don't need to use sample method.
        sceneZMax = gCameraSpaceZBuffer[int2(hitPixel)];
        float4 sceneZMin = sceneZMax - gCsZThickness;

        // Go over all layers to check whether there is a hit in one of the layer
        // TODO: can we reduce the number of iteration by ignoring some dummy layers where depth is on the far plane?
        for (uint layer = 0; layer < gLayers; layer++)
        {
            if (((rayZMax >= sceneZMin[layer]) && (rayZMin <= sceneZMax[layer])) || // A hit is found if there is a overlap between scene depth range and ray depth range 
                (sceneZMax[layer] == 0)) // This is to perform implicit viewport clipping for out-of bound depth buffer sample
            {
                hitLayer = layer;
                break;
            }
        }

        // TODO: Ours approach for using multi-layers
        // for (uint layer = 0; layer < gLayers; layer += 2)
        // {

        // }
        
        // Hit found also break the outer loop
        if (hitLayer != -1)
            break;
    }

    print("P = ", P);
    print("hitPixel = ", hitPixel);
    print("stepCount = ", stepCount);
    print("rayZMax = ", rayZMax);
    print("rayZMin = ", rayZMin);
    print("sceneZMax = ", sceneZMax);
    print("hitLayer = ", hitLayer);

    Q.xy += dQ.xy * stepCount;
    float3 csHitPoint = Q * (1.0 / k);
    // print("csHitPoint = ", csHitPoint);

    // Check the visibility
    bool isVisible = hitLayer == -1                                                      ? true // Hit or not?
                     : all(abs(hitPixel - (gScreenSize * 0.5)) <= gScreenSize * 0.5) ? false // If hit, is it a vaild screen point? (this has the same functionality to frustum clipping)
                                                                                         : true;

    // isVisible = hitLayer == -1 ? true : false;
    print("isVisible = ", isVisible);
    return isVisible;
}

// Fail Case: 
// This case occurs when shadow ray intersect the border of an object, so in the screen the intersection is in a very tiny interval.
// If uniform stepping is not small enough, it will step over the interval where the intersection is. It creates banding artifacts.
bool ssrtUniformStepBinarySearch(
    float3 rayOrigin, 
    float3 rayDirection,
    float maxRayTraceDistance,
)
{
    // Transform to camera space
    float3 csOrigin = mul(float4(rayOrigin, 1.0f), gWorldToCamera).xyz;
    float3 csDirection = mul(float4(rayDirection, 0.0f), gWorldToCamera).xyz; // Note: direction needs to be 0.0f for w

    // Analog to trace shadow ray, we need to include a small bias to the ray's origin and max distance (end point)
    csOrigin += min(gCsRayOriginLenEpsilon.x, maxRayTraceDistance) * csDirection; // cannot larger than ray length
    maxRayTraceDistance = max(maxRayTraceDistance - gCsRayOriginLenEpsilon.x - gCsRayOriginLenEpsilon.y, 0.0f); // cannot be negative

    // Clip ray to a near plane in 3D (doesn't have to be *the* near plane, although that would be a good idea)
    // Note: all camera space z value is in negative measure
    float rayLength = ((csOrigin.z + csDirection.z * maxRayTraceDistance) > gNearPlaneZ) ?
                        (gNearPlaneZ - csOrigin.z) / csDirection.z : // use the ray equation to solve value t to perform clipping
                        maxRayTraceDistance;
	float3 csEndPoint = csDirection * rayLength + csOrigin;

    // Project into clip space
    float4 H0 = mul(float4(csOrigin, 1.0), gCameraToClip);
    float4 H1 = mul(float4(csEndPoint, 1.0), gCameraToClip);

    // There are a lot of divisions by w that can be turned into multiplications at some minor precision loss...and we need to interpolate these 1/w values anyway.
    // Because the caller was required to clip to the near plane, this homogeneous division (projecting from 4D to 2D) is guaranteed to succeed. 
    float k0 = 1.0 / H0.w; // LH system -> w = -z
    float k1 = 1.0 / H1.w;

    // Switch the original points to values that interpolate linearly in 2D.
    // Note: the "Q" notation from the paper refers to the camera space point without multipling with "k" and the z value in Q is in the range [0, 1]  
    float3 Q0 = csOrigin * k0; 
    float3 Q1 = csEndPoint * k1;

    // Clip space -> NDC -> Screen-space endpoints 
    float2 P0 = (H0.xy * k0  * float2(0.5f, -0.5f) + float2(0.5f)) * gScreenSize; 
    float2 P1 = (H1.xy * k1  * float2(0.5f, -0.5f) + float2(0.5f)) * gScreenSize; 

    // [Optional clipping to frustum sides here]
    if (gClipToFrustum)
    {
        float xMin = 0.5f, xMax = gScreenSize.x - 0.5f;
        float yMin = 0.5f, yMax = gScreenSize.y - 0.5f;
        float alpha = 0.0f;
        
        // Assume P0 is in the viewport (P1 - P0 is never zero when clipping)
        if (P1.y > yMax || P1.y < yMin)
            alpha = (P1.y - ((P1.y > yMax) ? yMax : yMin)) / (P1.y - P0.y);

        if (P1.x > xMax || P1.x < xMin)
            alpha = max(alpha, (P1.x - ((P1.x > xMax) ? xMax : xMin)) / (P1.x - P0.x));

        P1 = lerp(P1, P0, alpha); 
        k1 = lerp(k1, k0, alpha); 
        Q1 = lerp(Q1, Q0, alpha);
    }

    // If the line is degenerate, make it cover at least one pixel to avoid handling zero-pixel extent as a special case later
    P1 += float2((distanceSquared(P0, P1) < 0.0001) ? 0.01 : 0.0);

    float2 delta = P1 - P0;

    // Permute so that the primary iteration is in x to reduce large branches later
    bool permute = false;
	if (abs(delta.x) < abs(delta.y)) 
    {
		// More-vertical line. Create a permutation that swaps x and y in the output
		permute = true;

        // Directly swizzle the inputs
		delta = delta.yx;
		P1 = P1.yx;
		P0 = P0.yx;
	}

    // From now on, "x" is the primary iteration direction and "y" is the secondary one

    float screenStepDirection = sign(delta.x); // -1 or 1
    float invdx = screenStepDirection / delta.x; // Transform to +x direction
    float2 dP = float2(screenStepDirection, invdx * delta.y); // e.g. dP = (1, y/x), it is the differential of x, y in Eq.1

    // Track the derivatives of Q and k
    float3 dQ = (Q1 - Q0) * invdx; // both Q.z is -1, so dQ.z is 0
    float dk = (k1 - k0) * invdx;

    // Scale derivatives by the desired pixel stride
    float relativeStride = ceil(0.1 * abs(delta.x)); // TODO: use relative stride 
	dP *= gStride; 
    dQ *= gStride; 
    dk *= gStride;

    // Offset the starting values by the jitter fraction
	P0 += dP * gJitterFraction; 
    Q0 += dQ * gJitterFraction; 
    k0 += dk * gJitterFraction;

	// Slide P from P0 to P1, (now-homogeneous) Q from Q0 to Q1, and k from k0 to k1
    float3 Q = Q0;
    float k = k0;

    // P1.x is never modified after this point, so pre-scale it by the step direction for a signed comparison.
    float end = P1.x * screenStepDirection;

    float2 P = P0;
    float2 pixelCoord = float2(-1.0f);
    float stepCount = 0.0f;
    float prevDeltaZSign = -1;
    float sceneZMax;

    int hitLayer = -1;
    float2 hitPixel = float2(-1);

    print("P0 = ", P0);
    print("P1 = ", P1);

    // 1. Uniform stepping over the empty space to find the correct interval where there is an intersection
    // Note: P is used to get scene's z value, Q.z and k are used to compute ray's z value
    for (; ((P.x * screenStepDirection) <= end) /*&& (stepCount < gMaxSteps)*/;
        P += dP, Q.z += dQ.z, k += dk, stepCount += 1.0) // TODO: remove Q.z and dQ.z?
    {
        // Get the true pixel coordinate (not normalized)
        pixelCoord = permute ? P.yx : P;

        // The ray can either march towards the camera or away from the camera. In order to combine those two cases,
        // we need to check when the difference between ray depth and scene depth changes. That is the interval we need to search
        float rayZCenter = Q.z / k;
        sceneZMax = gCameraSpaceZBuffer[int2(pixelCoord)].r;
        float sceneZMin = sceneZMax - gCsZThickness;
        float raySceneDeltaZ = rayZCenter - sceneZMax;
        float currDeltaZSign = sign(raySceneDeltaZ);

        print("index = ", stepCount);
        print("rayCenter = ", rayZCenter);
        print("sceneZMax = ", sceneZMax);
        print("sceneZMin = ", sceneZMin);
        print("prevDeltaZSign = ", prevDeltaZSign);
        print("currDeltaZSign = ", currDeltaZSign);
        print("P = ", P);

        if (rayZCenter >= sceneZMin && rayZCenter <= sceneZMax)
        {
            hitLayer = 0;
            hitPixel = pixelCoord;
            break;
        }

        // There is a blocker between previous and current point -> binary search
        if (stepCount > 0 && sign(raySceneDeltaZ) != prevDeltaZSign)
        {
            prevDeltaZSign = sign(raySceneDeltaZ);
            break;
        }

        prevDeltaZSign = sign(raySceneDeltaZ);
    }

    float nextSearchDirection = -1.0f;
    print("After uniform stepping P = ", P);

    // If P go beyond the endpoint P1, clamp to endpoint
    bool outsideEndpoint = (P.x * screenStepDirection) > end;
    if (outsideEndpoint)
    {
        float scaleFraction = (P1.x - (P.x - dP.x)) / dP.x;
        P = P1; 
        Q = Q1; 
        k = k1;

        dP *= scaleFraction;
        dQ *= scaleFraction;
        dk *= scaleFraction;
    }

    // 2. Binary search the hit point in the interval if there is any
    while (hitLayer == -1 && abs(dP.x) >= 1) // stride should be larger than 1
    {
        // Reduce our stride by half
        dP *= 0.5; 
        dQ *= 0.5;
        dk *= 0.5;

        print("prevDeltaZSign = ", prevDeltaZSign);

        // Move our current location on the ray and shrink the interval to where their may have a hit
        // float signChanged =
        P += dP * nextSearchDirection;
        Q.z += dQ.z * nextSearchDirection;
        k += dk * nextSearchDirection; 

        pixelCoord = permute ? P.yx : P;

        print("P = ", P);

        float rayZCenter = Q.z / k;
        sceneZMax = gCameraSpaceZBuffer[int2(pixelCoord)].r;
        float sceneZMin = sceneZMax - gCsZThickness;

        print("rayCenter = ", rayZCenter);
        print("sceneZMax = ", sceneZMax);
        print("sceneZMin = ", sceneZMin);

        // Stride is within two texels, we can use ray range to test 
        if (true && abs(dP.x) <= 2)
        {
            float rayZMin = (Q.z - dQ.z * 0.5) / (k - dk * 0.5);
            float rayZMax = (Q.z + dQ.z * 0.5) / (k + dk * 0.5);
            if (rayZMin > rayZMax) swap(rayZMin, rayZMax);

            if (rayZMax >= sceneZMin && rayZMin <= sceneZMax)
            {
                hitLayer = 0;
                hitPixel = pixelCoord;
                break;
            }
        }
        else
        {
            if (rayZCenter >= sceneZMin && rayZCenter <= sceneZMax)
            {
                hitLayer = 0;
                hitPixel = pixelCoord;
                break;
            }
        }

        // Adjust our next search direction if current sign changed compare to previous one
        float currDeltaZSign = sign(rayZCenter - sceneZMin);
        nextSearchDirection = currDeltaZSign == prevDeltaZSign ? nextSearchDirection : -nextSearchDirection;
        prevDeltaZSign = currDeltaZSign;
    }

    print("After binary search P = ", P);
    print("hitLayer = ", hitLayer);

    // Check the visibility
    bool isVisible = hitLayer == -1                                                      ? true // Hit or not?
                     : all(abs(hitPixel - (gScreenSize * 0.5)) <= gScreenSize * 0.5) ? false // If hit, is it a vaild screen point? (this has the same functionality to frustum clipping)
                                                                                         : true;
                                                                                        
    print("isVisible = ", isVisible);
    return isVisible;
}


void myPrint<T : IPrintable>(float targetIndex, float currIndex, String msg, T value)
{
    if (targetIndex == currIndex) print(msg, value);
}

void myPrint<T : IPrintable, let N : int>(float targetIndex, float currIndex, String msg, vector<T, N> value)
{
    if (targetIndex == currIndex) print(msg, value);
}

struct MipLevelInfo
{
    float3 levelEndPoint;
    float2 levelCenter;
}

// Maximum Mipmaps intersection
bool ssrtMaxMips(
    float3 rayOrigin, 
    float3 rayDirection,
    float maxRayTraceDistance,
)
{
    // Get z-buffer texture size
    const float2 csZBufferSize;
    gCameraSpaceZBuffer.GetDimensions(csZBufferSize.x, csZBufferSize.y);

    // Transform to camera space
    float3 csOrigin = mul(float4(rayOrigin, 1.0f), gWorldToCamera).xyz;
    float3 csDirection = mul(float4(rayDirection, 0.0f), gWorldToCamera).xyz; // Note: direction needs to be 0.0f for w

    // Analog to trace shadow ray, we need to include a small bias to the ray's origin and max distance (end point)
    csOrigin += min(gCsRayOriginLenEpsilon.x, maxRayTraceDistance) * csDirection; // cannot larger than ray length
    maxRayTraceDistance = max(maxRayTraceDistance - gCsRayOriginLenEpsilon.x - gCsRayOriginLenEpsilon.y, 0.0f); // cannot be negative

    // Clip ray to a near plane in 3D (doesn't have to be *the* near plane, although that would be a good idea)
    float rayLength = ((csOrigin.z + csDirection.z * maxRayTraceDistance) > gNearPlaneZ) ?
                        (gNearPlaneZ - csOrigin.z) / csDirection.z : // use the ray equation to solve value t to perform clipping
                          maxRayTraceDistance;
    float3 csEndPoint = csDirection * rayLength + csOrigin;

    // Project into clip space
    float4 H0 = mul(float4(csOrigin, 1.0), gCameraToClip);
    float4 H1 = mul(float4(csEndPoint, 1.0), gCameraToClip);

    // There are a lot of divisions by w that can be turned into multiplications at some minor precision loss...and we need to interpolate these 1/w values anyway.
    // Because the caller was required to clip to the near plane, this homogeneous division (projecting from 4D to 2D) is guaranteed to succeed.
    float k0 = 1.0 / H0.w; // LH system -> w = -z
    float k1 = 1.0 / H1.w;

    // Switch the original points to values that interpolate linearly in 2D.
    // Note: the "Q" notation from the paper refers to the camera space point without multipling with "k" and the z value in Q is in the range [0, 1]
    float3 Q0 = csOrigin * k0;
    float3 Q1 = csEndPoint * k1;

    // Clip space -> NDC -> Screen-space endpoints
    float2 P0 = (H0.xy * k0 * float2(0.5f, -0.5f) + float2(0.5f)) * gScreenSize;
    float2 P1 = (H1.xy * k1 * float2(0.5f, -0.5f) + float2(0.5f)) * gScreenSize;

    // [Optional clipping to frustum sides here]
    if (gClipToFrustum)
    {
        float xMin = 0.5f, xMax = gScreenSize.x - 0.5f;
        float yMin = 0.5f, yMax = gScreenSize.y - 0.5f;
        float alpha = 0.0f;

        // Assume P0 is in the viewport (P1 - P0 is never zero when clipping)
        if (P1.y > yMax || P1.y < yMin)
            alpha = (P1.y - ((P1.y > yMax) ? yMax : yMin)) / (P1.y - P0.y);

        if (P1.x > xMax || P1.x < xMin)
            alpha = max(alpha, (P1.x - ((P1.x > xMax) ? xMax : xMin)) / (P1.x - P0.x));

        P1 = lerp(P1, P0, alpha);
        k1 = lerp(k1, k0, alpha);
        Q1 = lerp(Q1, Q0, alpha);
    }

    // If the line is degenerate, make it cover at least one pixel to avoid handling zero-pixel extent as a special case later
    P1 += float2((distanceSquared(P0, P1) < 0.0001) ? 0.01 : 0.0);

    float2 delta = P1 - P0;

    // Permute so that the primary iteration is in x to reduce large branches later
    bool permute = false;
    if (abs(delta.x) < abs(delta.y))
    {
        // More-vertical line. Create a permutation that swaps x and y in the output
        permute = true;

        // Directly swizzle the inputs
        // delta = delta.yx;
        // P1 = P1.yx;
        // P0 = P0.yx;
    }

    float2 screenStepDirection = float2(sign(delta.x), sign(delta.y));
    float invdx = screenStepDirection.x / delta.x; // Transform to +x direction = take absolute value
    float invdy = screenStepDirection.y / delta.y;
    float2 dxdyAndDydx = float2(invdy * delta.x, invdx * delta.y); // with direction

    // Track the derivatives of Q and k
    float2 dk = float2((k1 - k0) * invdx, (k1 - k0) * invdy);

    // Hit infos
    int hitLayer = -1;
    float2 hitPixel = float2(-1);

    print("permute = ", permute);
    print("P0 = ", P0);
    print("P1 = ", P1);
    print("normalizedDeltaXY = ", dxdyAndDydx);

    // Global endpoints with camera space z value
    float3 globalStartPoint = float3(P0, Q0.z / k0);
    float3 globalEndPoint = float3(P1, Q1.z / k1);

    // Variables we need to keep tracking
    const float maxMipLevel = log2(csZBufferSize.x); 
    float currMipLevel = maxMipLevel;
    float3 currLevelP0 = globalStartPoint; // also store the camera space z
    float3 currLevelP1 = globalEndPoint;
    float2 currLevelCenter = csZBufferSize;

    // Level info: center, interection points + current level end point P1. From MAX to 1
    MipLevelInfo levelInfos[11];

    float targetIndex = 51;
    float index = 0;

    // Traverse the max-mipmaps from the highest level
    // Note: I think there is no need to do permutation
    while (index <= gMaxSteps && currMipLevel >= 0 && currMipLevel <= maxMipLevel && hitLayer == -1 && all(currLevelP0 != globalEndPoint))
    {
        if (index >= 49 && index <= 51) targetIndex = index;

        myPrint(targetIndex, index, "l = ", currMipLevel);
        myPrint(targetIndex, index, "currLevelP0 = ", currLevelP0);
        myPrint(targetIndex, index, "currLevelP1 = ", currLevelP1);
        myPrint(targetIndex, index, "currLevelCenter = ", currLevelCenter);

        // Update current level latest info
        uint arrIndex = maxMipLevel - currMipLevel;
        levelInfos[arrIndex].levelCenter = currLevelCenter;
        levelInfos[arrIndex].levelEndPoint = currLevelP1;

        // Reach the boundary of current level -> ascend one level up
        if (all(currLevelP0 == currLevelP1))
        {
            currLevelP1 = levelInfos[arrIndex - 1].levelEndPoint;
            currLevelCenter = levelInfos[arrIndex - 1].levelCenter;
            currMipLevel++;

            continue;
        }

        // 1. Find intersection points with current level vertical (x) & horizontal (y) center line in screen space
        // TODO: I don't think we need to do it in finest level 
        float2 verticalIntersection = float2(currLevelCenter.x, -1);
        float2 horizontalIntersection = float2(-1, currLevelCenter.y);
        float verticalCameraSpaceZ = 1.0f, horizontalCameraSpaceZ = 1.0f;

        uint intervalCount = 1;
        float3 intervalPoints[4] = {
            currLevelP0, float3(0.0), float3(0.0), float3(0.0)
        };

        // Is there an intersection in vertical line?
        if (currLevelCenter.x != currLevelP0.x && currLevelCenter.x != currLevelP1.x && 
            sign(currLevelP0.x - currLevelCenter.x) != sign(currLevelP1.x - currLevelCenter.x))
        {
            float dx = abs(currLevelCenter.x - P0.x);
            verticalIntersection.y = P0.y + dx * dxdyAndDydx.y; // dP includes sign direction 
            verticalCameraSpaceZ = Q0.z / (k0 + dx * dk.x);
            intervalCount++;
        }

        // Is there an intersection in horizontal line?
        if (currLevelCenter.y != currLevelP0.y && currLevelCenter.y != currLevelP1.y && 
            sign(currLevelP0.y - currLevelCenter.y) != sign(currLevelP1.y - currLevelCenter.y))
        {
            float dy = abs(currLevelCenter.y - P0.y);
            horizontalIntersection.x = P0.x + dy * dxdyAndDydx.x;
            horizontalCameraSpaceZ = Q0.z / (k0 + dy * dk.y);
            intervalCount++;
        }

        myPrint(targetIndex, index, "verticalIntersection = ", verticalIntersection);
        myPrint(targetIndex, index, "horizontalIntersection = ", horizontalIntersection);

        // Check all the intersection cases and sort the points
        if (verticalIntersection.y == -1)
        {
            if (horizontalIntersection.x == -1)
            {
                intervalPoints[1] = currLevelP1;
            }
            else
            {
                intervalPoints[1] = float3(horizontalIntersection, horizontalCameraSpaceZ);
                intervalPoints[2] = currLevelP1;
            }
        }
        else
        {
            if (horizontalIntersection.x == -1)
            {
                intervalPoints[1] = float3(verticalIntersection, verticalCameraSpaceZ);
                intervalPoints[2] = currLevelP1;
            }
            else
            {
                float deltaVertical = abs(verticalIntersection.x - currLevelP0.x);
                float deltaHorizontal = abs(horizontalIntersection.x - currLevelP0.x);
                if (deltaVertical < deltaHorizontal)
                {
                    intervalPoints[1] = float3(verticalIntersection, verticalCameraSpaceZ);
                    intervalPoints[2] = float3(horizontalIntersection, horizontalCameraSpaceZ);
                    intervalPoints[3] = currLevelP1;
                }
                else if (deltaVertical == deltaHorizontal)
                {
                    intervalPoints[1] = float3(verticalIntersection, verticalCameraSpaceZ);
                    intervalPoints[2] = currLevelP1;
                    intervalCount = 2;
                }
                else
                {
                    intervalPoints[1] = float3(horizontalIntersection, horizontalCameraSpaceZ);
                    intervalPoints[2] = float3(verticalIntersection, verticalCameraSpaceZ);
                    intervalPoints[3] = currLevelP1;
                }
            }
        }


        myPrint(targetIndex, index, "intervalCount = ", intervalCount);

        // 2. Go over and check each interval one by one (max 3 intervals)
        for (uint i = 0; i < intervalCount; i++)
        {
            float3 intervalStartPoint = intervalPoints[i];
            float3 intervalEndPoint = intervalPoints[i + 1];

            myPrint(targetIndex, index, "intervalStartPoint = ", intervalStartPoint);
            myPrint(targetIndex, index, "intervalEndPoint = ", intervalEndPoint);

            // 2.1 Get the texel's position covered by current interval
            float2 intervalCenter = (intervalStartPoint.xy + intervalEndPoint.xy) / 2.0f;
            float2 whichTexel = float2(
                (intervalCenter.x < currLevelCenter.x) ? -1 : 1,
                (intervalCenter.y < currLevelCenter.y) ? -1 : 1
            );

            myPrint(targetIndex, index, "whichTexel = ", whichTexel);

            // 2.2 Check if there is an intersection in the camera space on the texel covered by the current interval

            // Sample the z-buffer texture
            float2 texelCenterScreenPos = currLevelCenter + 0.5 * whichTexel * pow(2, currMipLevel);
            float2 texCoord = texelCenterScreenPos / csZBufferSize;
            float intervalSceneZMax = gCameraSpaceZBuffer.SampleLevel(gPointSampler, texCoord, currMipLevel).x;
            float intervalSceneZMin = intervalSceneZMax - gCsZThickness;

            myPrint(targetIndex, index, "texelCenterScreenPos = ", texelCenterScreenPos);
            myPrint(targetIndex, index, "intervalSceneZMax = ", intervalSceneZMax);
            myPrint(targetIndex, index, "intervalSceneZMin = ", intervalSceneZMin);

            // Get camera space ray's z range
            float rayZMax = (intervalStartPoint.z > intervalEndPoint.z) ? intervalStartPoint.z : intervalEndPoint.z;
            float rayZMin = (intervalStartPoint.z > intervalEndPoint.z) ? intervalEndPoint.z : intervalStartPoint.z;

            myPrint(targetIndex, index, "rayZMax = ", rayZMax);
            myPrint(targetIndex, index, "rayZMin = ", rayZMin);

            // When we reach the level 0, check whether there is an intersection with the scene itself
            bool intersectScene = false;
            if (currMipLevel == 0)
            {
                intersectScene = (rayZMax >= intervalSceneZMin) && (rayZMin <= intervalSceneZMax);
                myPrint(targetIndex, index, "intersectScene = ", intersectScene);

                if (intersectScene)
                {
                    hitLayer = 0;
                    hitPixel = texelCenterScreenPos;
                    break;
                }
            }

            // Check whether there is an intersection with max-mipmaps in camera space
            bool intersectMaxMips = (rayZMax <= intervalSceneZMax);
            myPrint(targetIndex, index, "intersectMaxMips = ", intersectMaxMips);

            if (currMipLevel != 0 && intersectMaxMips)
            {
                // Update level points to lower level
                currLevelP0 = intervalStartPoint;
                currLevelP1 = intervalEndPoint;
                currLevelCenter = texelCenterScreenPos;

                // Descend one level down
                currMipLevel--;
                break;
            }

            // No intersection -> If not last interval, just continue to the next interval
            bool lastInterval = (i == intervalCount - 1) ? true : false;
            if (lastInterval && (!intersectMaxMips || currMipLevel == 0))
            {
                myPrint(targetIndex, index, "any(intervalEndPoint.xy != P1) = ", any(intervalEndPoint.xy != P1));

                // For last interval, we need to check whether there is another interval we can move to.
                // This interval should within a different texel than current interval in the upper level
                if (any(intervalEndPoint.xy != P1)) //-> we have an interval that can move to
                {
                    // Update level endpoints to upper level
                    currLevelP0 = currLevelP1;
                    currLevelP1 = levelInfos[arrIndex - 1].levelEndPoint;
                    currLevelCenter = levelInfos[arrIndex - 1].levelCenter;

                    myPrint(targetIndex, index, "upperLevelEndPoint = ", currLevelP1);
                    myPrint(targetIndex, index, "upperLevelCenter = ", currLevelCenter);

                    // Ascend one level up
                    currMipLevel++;
                }
                else
                {
                    // This will break the whole loop.
                    currLevelP0 = currLevelP1;
                }
            }
        }

        index++;
    }

    print("After traversing currMipLevel = ", currMipLevel);
    print("After traversing currLevelP0 = ", currLevelP0);
    print("After traversing currLevelP1 = ", currLevelP1);
    print("iterations = ", index - 1);
    print("hitPixel = ", hitPixel);

    float red, green, blue;
    red = 1 - index / 100;
    if (index >= gMaxSteps * 0.5f)
    {
        gDebug[uint2(P0)] = float4(1.0, 0.0f, 0.0f, 1.0f);
    }

    // Check the visibility
    bool isVisible = hitLayer == -1 ? true // Hit or not?
        : all(abs(hitPixel - (gScreenSize * 0.5)) <= gScreenSize * 0.5) ? false // If hit, is it a vaild screen point? (this has the same functionality to frustum clipping)
            : true;

    print("isVisible = ", isVisible);
    return isVisible;
}