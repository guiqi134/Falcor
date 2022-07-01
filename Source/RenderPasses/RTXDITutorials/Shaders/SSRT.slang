import Utils.Debug.PixelDebug;

cbuffer SSRT_CB
{
    float4x4 gWorldToCamera;
    float4x4 gCameraToClip;
    uint gVisibilityMode; // 0: Shadow Ray, 1: SSRT
    float gNearPlaneZ;
    SamplerState gTrilinearSampler;
}

Texture2D gCameraSpaceZBuffer;

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

    \param which Which layer has the ray hit

    \return true is visible, false is occluded

    TODO: using depth-peeling to generate multi-layer depth buffer 
*/
bool traceScreenSpaceShadowRay(
    float3          csOrigin, 
    float3          csDirection,
    float2          csZBufferSize,
    float           maxRayTraceDistance,
    float           csZThickness = 0.f, 
    float			stride = 1.0f,
    float           jitterFraction = 0.0f,
    float           maxSteps = 1,
)
{
    // Clip ray to a near plane in 3D (doesn't have to be *the* near plane, although that would be a good idea)
    // Note: all camera space z value is in negative measure
    float rayLength = ((csOrigin.z + csDirection.z * maxRayTraceDistance) > gNearPlaneZ) ?
                        (gNearPlaneZ - csOrigin.z) / csDirection.z : // use the ray equation to solve value t to perform clipping
                        maxRayTraceDistance;
	float3 csEndPoint = csDirection * rayLength + csOrigin;

    print("csOrigin = ", csOrigin);
    print("csDireciton = ", csDirection);
    print("rayLength = ", rayLength);

    // Project into clip space
    float4 H0 = mul(float4(csOrigin, 1.0), gCameraToClip);
    float4 H1 = mul(float4(csEndPoint, 1.0), gCameraToClip);

    print("H0 = ", H0);

    // There are a lot of divisions by w that can be turned into multiplications at some minor precision loss...and we need to interpolate these 1/w values anyway.
    // Because the caller was required to clip to the near plane, this homogeneous division (projecting from 4D to 2D) is guaranteed to succeed. 
    float k0 = 1.0 / H0.w; // LH system -> w = -z
    float k1 = 1.0 / H1.w;

    // Switch the original points to values that interpolate linearly in 2D.
    // Note: the "Q" notation from the paper refers to the camera space point without multipling with "k" and the z value in Q is in the range [0, 1]  
    float3 Q0 = csOrigin * k0; 
    float3 Q1 = csEndPoint * k1;

    // Clip space -> NDC -> Screen-space endpoints 
    float2 P0 = (H0.xy * k0  * float2(0.5f, -0.5f) + float2(0.5f)) * csZBufferSize; 
    float2 P1 = (H1.xy * k1  * float2(0.5f, -0.5f) + float2(0.5f)) * csZBufferSize; 

    print("P0 = ", P0);
    print("P1 = ", P1);

    // TODO: [Optional clipping to frustum sides here]

    // Initialize to off screen
    float2 hitPixel = float2(-1.0, -1.0); // This actually is uint2
    int which = 0; // Only one layer

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

    float stepDirection = sign(delta.x); // -1 or 1
    float invdx = stepDirection / delta.x; // Transform to +x direction
    float2 dP = float2(stepDirection, invdx * delta.y); // e.g. dP = (1, y/x), it is the differential of x, y in Eq.1

    // Track the derivatives of Q and k
    float3 dQ = (Q1 - Q0) * invdx;
    float dk = (k1 - k0) * invdx;

    // Scale derivatives by the desired pixel stride
	dP *= stride; 
    dQ *= stride; 
    dk *= stride;

    // Offset the starting values by the jitter fraction
	P0 += dP * jitterFraction; 
    Q0 += dQ * jitterFraction; 
    k0 += dk * jitterFraction;

	// Slide P from P0 to P1, (now-homogeneous) Q from Q0 to Q1, and k from k0 to k1
    float3 Q = Q0;
    float k = k0;

	// We track the ray depth at +/- 1/2 pixel to treat pixels as clip-space solid voxels. Because the depth at -1/2 for a given pixel will be the same as at 
	// +1/2 for the previous iteration, we actually only have to compute one value per iteration.
	float prevZMaxEstimate = csOrigin.z;
    float stepCount = 0.0;
    float rayZMax = prevZMaxEstimate, rayZMin = prevZMaxEstimate;
    float sceneZMax = rayZMax + 1e4;

    // P1.x is never modified after this point, so pre-scale it by the step direction for a signed comparison, this is to transform point to +x direction
    float end = P1.x * stepDirection;

    // We only advance the z field of Q in the inner loop, since Q.xy is never used until after the loop terminates.

	for (float2 P = P0;
        ((P.x * stepDirection) <= end) && 
        (stepCount < maxSteps) && 
        ((rayZMax < sceneZMax - csZThickness) || // csZThickness is positive value, so we need to transform it to negative measure
            (rayZMin > sceneZMax)) && // If sceneZMax is not inside the range of rayZMax and rayZMin, then there is no hit found
        (sceneZMax != 0.0); // This is to perform implicit viewport clipping for out-of bound depth buffer sample  
        P += dP, Q.z += dQ.z, k += dk, stepCount += 1.0) 
    {
        
        // Get the true pixel coordinate (not normalized)
		hitPixel = permute ? P.yx : P;

        // The depth range that the ray covers within this loop iteration. Assume that the ray is moving in increasing z
        // and swap if backwards.  Because one end of the interval is shared between adjacent iterations, we track the previous
        // value and then swap as needed to ensure correct ordering
        rayZMin = prevZMaxEstimate;

        // Compute the value at 1/2 pixel into the future
        rayZMax = (dQ.z * 0.5 + Q.z) / (dk * 0.5 + k); // by dividing k, we transform back to the camera space (z is negative)
		prevZMaxEstimate = rayZMax;
        if (rayZMin > rayZMax) swap(rayZMin, rayZMax);

        // Camera-space z of the background
        // Note: the z buffer need to be in camera space, sample location needs to be normalized
        float2 texCoords = (uint2(hitPixel) + float2(0.5f)) / csZBufferSize; // Song: Do I need to add half pixel?
        print("texCoords = ", texCoords);
        sceneZMax = gCameraSpaceZBuffer.SampleLevel(gTrilinearSampler, texCoords, 0).r;
    }
    print("csOrigin.z = ", csOrigin.z);
    print("stepCount = ", stepCount);
    print("rayZMax = ", rayZMax);
    print("rayZMin = ", rayZMin);
    print("sceneZax = ", sceneZMax);

    Q.xy += dQ.xy * stepCount;
    float3 csHitPoint = Q * (1.0 / k);
    print("csHitPoint = ", csHitPoint);

    // Check whether the ray hit
    bool isVisible = !((rayZMax >= sceneZMax - csZThickness) && (rayZMin <= sceneZMax));
    return isVisible;
}
