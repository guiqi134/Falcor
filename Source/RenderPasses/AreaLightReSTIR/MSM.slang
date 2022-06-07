// Moment Shadow Map by MJP
// https://github.com/TheRealMJP/Shadows/blob/master/Shadows/MSM.hlsl

import Utils.Debug.PixelDebug;

// 4.3 Optimized Moment Quantization
// Those methods are only needed when using 16-bits float 
// -----------------------------------------------
float4 getOptimizedMoments(float depth)
{
    float square = depth * depth;
    float4 moments = float4(depth, square, square * depth, square * square);

    // The affine transformation matrix 
    float4 optimized = mul(moments, float4x4(-2.07224649f, 13.7948857237f, 0.105877704f, 9.7924062118f,
        32.23703778f, -59.4683975703f, -1.9077466311f, -33.7652110555f,
        -68.571074599f, 82.0359750338f, 9.3496555107f, 47.9456096605f,
        39.3703274134f, -35.364903257f, -6.6543490743f, -23.9728048165f));
    optimized[0] += 0.035955884801f;

    return optimized;
}

float4 convertOptimizedMoments(in float4 optimizedMoments)
{
    optimizedMoments[0] -= 0.035955884801f;
    return mul(optimizedMoments, float4x4(0.2227744146f, 0.1549679261f, 0.1451988946f, 0.163127443f,
        0.0771972861f, 0.1394629426f, 0.2120202157f, 0.2591432266f,
        0.7926986636f, 0.7963415838f, 0.7258694464f, 0.6539092497f,
        0.0319417555f, -0.1722823173f, -0.2758014811f, -0.3376131734f));
}

// Unbounded fragment depth input
float computeHamburger4MSM(float4 moments, float fragDepth, float momentBias)
{
    // Bias the moment to avoid numeical noise
    //float4 b = lerp(moments, float4(0.5f, 0.5f, 0.5f, 0.5f), momentBias); // too much artifacts
    float4 b = lerp(moments, float4(0.0f, 0.63f, 0.0f, 0.63f), momentBias);
    float3 z;
    z[0] = fragDepth;

    // Compute a Cholesky factorization of the Hankel matrix B storing only non-
    // trivial entries or related products
    float L32D22 = mad(-b[0], b[1], b[2]);
    float D22 = mad(-b[0], b[0], b[1]);
    float squaredDepthVariance = mad(-b[1], b[1], b[3]);
    float D33D22 = dot(float2(squaredDepthVariance, -L32D22), float2(D22, L32D22));
    float InvD22 = 1.0f / D22;
    float L32 = L32D22 * InvD22;

    // Obtain a scaled inverse image of bz = (1,z[0],z[0]*z[0])^T
    float3 c = float3(1.0f, z[0], z[0] * z[0]);

    // Forward substitution to solve L*c1=bz
    c[1] -= b.x;
    c[2] -= b.y + L32 * c[1];

    // Scaling to solve D*c2=c1
    c[1] *= InvD22;
    c[2] *= D22 / D33D22;

    // Backward substitution to solve L^T*c3=c2
    c[1] -= L32 * c[2];
    c[0] -= dot(c.yz, b.xy);

    // Solve z2 and z3 using quadratic formula
    float p = c[1] / c[2];
    float q = c[0] / c[2];
    // D2 has the potiential to be < 0, maybe it's caused by precision issue. It leads to artifacts. How to handle this?
    float D2 = (p * p * 0.25f) - q;
    float D = sqrt(D2);
    z[1] = -p * 0.5f - D;
    z[2] = -p * 0.5f + D;

    // Compute the shadow intensity by summing the appropriate weights
    float4 switchVal = (z[2] < z[0]) ? float4(z[1], z[0], 1.0f, 1.0f) : // Line 6: zf > z3 
        ((z[1] < z[0]) ? float4(z[0], z[1], 0.0f, 1.0f) : // Line 5: zf > z2 
            float4(0.0f, 0.0f, 0.0f, 0.0f)); // Line 4: zf <= z2 
    float quotient = (switchVal[0] * z[2] - b[0] * (switchVal[0] + z[2]) + b[1]) / ((z[2] - switchVal[1]) * (z[0] - z[1]));
    float shadowIntensity = switchVal[2] + switchVal[3] * quotient;

    return 1.0f - saturate(shadowIntensity);
}

// Fragment depth in bounded in [0, 1]
float computeHausdorff4MSM(float4 moments, float fragDepth, float momentBias)
{
    // Use short-hands for the many formulae to come
    float4 b = lerp(moments, float4(0.0f, 0.63f, 0.0f, 0.63f), momentBias);
    float3 z;
    float OutShadowIntensity;
    z[0] = fragDepth;

    // Compute a Cholesky factorization of the Hankel matrix B storing only non-
    // trivial entries or related products
    float L21D11 = mad(-b[0], b[1], b[2]);
    float D11 = mad(-b[0], b[0], b[1]);
    float SquaredDepthVariance = mad(-b[1], b[1], b[3]);
    float D22D11 = dot(float2(SquaredDepthVariance, -L21D11), float2(D11, L21D11));
    float InvD11 = 1.0f / D11;
    float L21 = L21D11 * InvD11;

    // Obtain a scaled inverse image of bz=(1,z[0],z[0]*z[0])^T
    float3 c = float3(1.0f, z[0], z[0] * z[0]);
    // Forward substitution to solve L*c1=bz
    c[1] -= b.x;
    c[2] -= b.y + L21 * c[1];
    // Scaling to solve D*c2=c1
    c[1] *= InvD11;
    c[2] *= D11 / D22D11;
    // Backward substitution to solve L^T*c3=c2
    c[1] -= L21 * c[2];
    c[0] -= dot(c.yz, b.xy);
    // Solve the quadratic equation c[0]+c[1]*z+c[2]*z^2 to obtain solutions z[1] 
    // and z[2]
    float InvC2 = 1.0f / c[2];
    float p = c[1] * InvC2;
    float q = c[0] * InvC2;
    float D = ((p * p) / 4.0f) - q;
    float r = sqrt(D);
    z[1] = -(p / 2.0f) - r;
    z[2] = -(p / 2.0f) + r;

    // Use a solution made of four deltas if the solution with three deltas is 
    // invalid
    [branch] if (z[1] < -1.0f || z[2]>1.0f) {
        float zFree = ((b[0] - b[2]) * z[0] + b[3] - b[1]) / (z[0] + b[2] - b[0] - b[1] * z[0]);
        float w1Factor = (z[0] > zFree) ? 1.0f : 0.0f;
        // Construct a polynomial taking value zero at z[0] and 1, value 1 at -1 and 
        // value w1Factor at zFree. Start with a linear part and then multiply by 
        // linear factors to get the roots.
        float2 Normalizers;
        Normalizers.x = w1Factor / ((zFree - z[0]) * mad(zFree, zFree, -1.0f));
        Normalizers.y = 0.5f / ((zFree + 1.0f) * (z[0] + 1.0f));
        float4 Polynomial;
        Polynomial[0] = mad(zFree, Normalizers.y, Normalizers.x);
        Polynomial[1] = Normalizers.x - Normalizers.y;
        // Multiply the polynomial by (z-z[0])
        Polynomial[2] = Polynomial[1];
        Polynomial[1] = mad(Polynomial[1], -z[0], Polynomial[0]);
        Polynomial[0] *= -z[0];
        // Multiply the polynomial by (z-1)
        Polynomial[3] = Polynomial[2];
        Polynomial.yz = Polynomial.xy - Polynomial.yz;
        Polynomial[0] *= -1.0f;
        // The shadow intensity is the dot product of the coefficients of this 
        // polynomial and the power moments for the respective powers
        OutShadowIntensity = dot(Polynomial, float4(1.0f, b.xyz));
    }
    // Use the solution with three deltas
    else {
        float4 Switch =
            (z[2] < z[0]) ? float4(z[1], z[0], 1.0f, 1.0f) : (
                (z[1] < z[0]) ? float4(z[0], z[1], 0.0f, 1.0f) :
                float4(0.0f, 0.0f, 0.0f, 0.0f));
        float Quotient = (Switch[0] * z[2] - b[0] * (Switch[0] + z[2]) + b[1]) / ((z[2] - Switch[1]) * (z[0] - z[1]));
        OutShadowIntensity = Switch[2] + Switch[3] * Quotient;
    }
    return 1.0f - saturate(OutShadowIntensity);
}
