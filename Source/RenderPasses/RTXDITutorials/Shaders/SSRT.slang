import Utils.Debug.PixelDebug;

cbuffer SSRT_CB
{
    float4x4 gWorldToCamera;
    float4x4 gCameraToClip;
    uint gVisibilityMode; // 0: Shadow Ray, 1: SSRT

    float2 gCsRayOriginLenEpsilon;
    float gNearPlaneZ;
    float gCsZThickness;
    float gStride;
    float gJitterFraction;
    float gMaxSteps;
    uint gLayers;
    bool gClipToFrustum;
    bool gShadowRayForShading;
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

    \param csZBufferSize Dimensions of csZBuffer

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
    float3 rayDirection,
    float maxRayTraceDistance,
)
{
    // Transform to camera space
    float3 csOrigin = mul(float4(rayOrigin, 1.0f), gWorldToCamera).xyz;
    float3 csDirection = mul(float4(rayDirection, 0.0f), gWorldToCamera).xyz; // Note: direction needs to be 0.0f for w
    float2 csZBufferSize;
    gCameraSpaceZBuffer.GetDimensions(csZBufferSize.x, csZBufferSize.y);

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
    float2 P0 = (H0.xy * k0  * float2(0.5f, -0.5f) + float2(0.5f)) * csZBufferSize; 
    float2 P1 = (H1.xy * k1  * float2(0.5f, -0.5f) + float2(0.5f)) * csZBufferSize; 

    // [Optional clipping to frustum sides here]
    if (gClipToFrustum)
    {
        float xMin = 0.5f, xMax = csZBufferSize.x - 0.5f;
        float yMin = 0.5f, yMax = csZBufferSize.y - 0.5f;
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

    float stepDirection = sign(delta.x); // -1 or 1
    float invdx = stepDirection / delta.x; // Transform to +x direction
    float2 dP = float2(stepDirection, invdx * delta.y); // e.g. dP = (1, y/x), it is the differential of x, y in Eq.1
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
    float end = P1.x * stepDirection;
    print("end = ", end);

    // We only advance the z field of Q in the inner loop, since Q.xy is never used until after the loop terminates.

    float2 P = P0;
    for (; ((P.x * stepDirection) <= end) && (stepCount < gMaxSteps);
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
                     : all(abs(hitPixel - (csZBufferSize * 0.5)) <= csZBufferSize * 0.5) ? false // If hit, is it a vaild screen point? (this has the same functionality to frustum clipping)
                                                                                         : true;

    // isVisible = hitLayer == -1 ? true : false;
    print("isVisible = ", isVisible);
    return isVisible;
}

