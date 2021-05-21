/**********************************************************************************************************************
# Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
# following conditions are met:
#  * Redistributions of code must retain the copyright notice, this list of conditions and the following disclaimer.
#  * Neither the name of NVIDIA CORPORATION nor the names of its contributors may be used to endorse or promote products
#    derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT
# SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**********************************************************************************************************************/
#include "Utils/HostDeviceShared.slangh"
#include "Utils/Math/MathConstants.slangh"

import Scene.Lights.Lights;
import Scene.Shading;
import Scene.Raytracing;
import Utils.Helpers;

import diffuseGIUtils;

// A constant buffer we'll populate from our C++ code  (used for our ray generation shader)
cbuffer RayGenCB
{
	float gMinT;           // Min distance to start a ray to avoid self-occlusion
	uint  gFrameCount;     // An integer changing every frame to update the random number
	bool  gDoIndirectGI;   // A boolean determining if we should shoot indirect GI rays
	bool  gCosSampling;    // Use cosine sampling (true) or uniform sampling (false)
	bool  gDirectShadow;   // Should we shoot shadow rays from our first hit point?
}

// Input and out textures that need to be set by the C++ code (for the ray gen shader)
Texture2D<float4> gPos;
Texture2D<float4> gNorm;
Texture2D<float4> gDiffuseMatl;
RWTexture2D<float4> gOutput;

// Payload for our shadow rays. 
struct ShadowRayPayload
{
	float visFactor;  // Will be 1.0 for fully lit, 0.0 for fully shadowed
};

// A utility function to trace a shadow ray and return 1 if no shadow and 0 if shadowed.
//    -> Note:  This assumes the shadow hit programs and miss programs are index 0!
float shadowRayVisibility(float3 origin, float3 direction, float minT, float maxT)
{
	// Setup our shadow ray
	RayDesc ray;
	ray.Origin = origin;        // Where does it start?
	ray.Direction = direction;  // What direction do we shoot it?
	ray.TMin = minT;            // The closest distance we'll count as a hit
	ray.TMax = maxT;            // The farthest distance we'll count as a hit

	// Our shadow rays are *assumed* to hit geometry; this miss shader changes this to 1.0 for "visible"
	ShadowRayPayload payload = { 0.0f };

	// Query if anything is between the current point and the light
	TraceRay(gRtScene,
		RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH | RAY_FLAG_SKIP_CLOSEST_HIT_SHADER,
		0xFF, 0, hitProgramCount, 0, ray, payload);

	// Return our ray payload (which is 1 for visible, 0 for occluded)
	return payload.visFactor;
}

// What code is executed when our ray misses all geometry?
[shader("miss")]
void ShadowMiss(inout ShadowRayPayload rayData)
{
	// If we miss all geometry, then the light is visibile
	rayData.visFactor = 1.0f;
}

// What code is executed when our ray hits a potentially transparent surface?
[shader("anyhit")]
void ShadowAnyHit(uniform HitShaderParams hitParams, inout ShadowRayPayload rayData, BuiltInTriangleIntersectionAttributes attribs)
{
	// Alpha test for non-opaque geometry.
    if (alphaTestFails(hitParams, attribs))
        IgnoreHit();
}

// What code is executed when we have a new closest hitpoint?
[shader("closesthit")]
void ShadowClosestHit(inout ShadowRayPayload rayData, BuiltInTriangleIntersectionAttributes attribs)
{
}

// The payload used for our indirect global illumination rays
struct IndirectRayPayload
{
	float3 color;    // The (returned) color in the ray's direction
	uint   rndSeed;  // Our random seed, so we pick uncorrelated RNGs along our ray
};

// A utility function to trace an idirect ray and return the color it sees.
//    -> Note:  This assumes the indirect hit programs and miss programs are index 1!
float3 shootIndirectRay(float3 rayOrigin, float3 rayDir, float minT, uint seed)
{
	// Setup shadow ray
	RayDesc rayColor;
	rayColor.Origin = rayOrigin;  // Where does it start?
	rayColor.Direction = rayDir;  // What direction do we shoot it?
	rayColor.TMin = minT;         // The closest distance we'll count as a hit
	rayColor.TMax = 1.0e38f;      // The farthest distance we'll count as a hit

	// Initialize the ray's payload data with black return color and the current rng seed
	IndirectRayPayload payload;
	payload.color = float3(0, 0, 0);  
	payload.rndSeed = seed;

	// Trace our ray to get a color in the indirect direction.  Use hit group #1 and miss shader #1
	TraceRay(gRtScene, 0, 0xFF, 1, hitProgramCount, 1, rayColor, payload);

	// Return the color we got from our ray
	return payload.color;
}

// What code is executed when our ray misses all geometry?
[shader("miss")]
void IndirectMiss(inout IndirectRayPayload rayData)
{
	rayData.color = float3(1.0f, 0.0f, 0.0f);
}

// What code is executed when our ray hits a potentially transparent surface?
[shader("anyhit")]
void IndirectAnyHit(uniform HitShaderParams hitParams, inout IndirectRayPayload rayData, BuiltInTriangleIntersectionAttributes attribs)
{
	// Is this a transparent part of the surface?  If so, ignore this hit
	if (alphaTestFails(hitParams, attribs))
		IgnoreHit();
}

// What code is executed when we have a new closest hitpoint?   Well, pick a random light,
//    shoot a shadow ray to that light, and shade using diffuse shading.
[shader("closesthit")]
void IndirectClosestHit(uniform HitShaderParams hitParams, inout IndirectRayPayload rayData, in BuiltInTriangleIntersectionAttributes attribs)
{
	// Run a helper functions to extract Falcor scene data for shading
	ShadingData shadeData = getHitShadingData(hitParams, attribs);

	// Pick a random light from our scene to shoot a shadow ray towards	
    int gLightsCount = gScene.getLightCount();
	int lightToSample = min(int(rand_next(rayData.rndSeed) * gLightsCount), gLightsCount - 1);

	// Query the scene to find info about the randomly selected light
	float distToLight;
	float3 lightIntensity;
	float3 toLight;
	getLightData(lightToSample, shadeData.posW, toLight, lightIntensity, distToLight);

	// Compute our lambertion term (L dot N)
	float LdotN = saturate(dot(shadeData.N, toLight));

	// Shoot our shadow ray to our randomly selected light
	float shadowMult = float(gLightsCount) * shadowRayVisibility(shadeData.posW, toLight, RayTMin(), distToLight);

	// Return the Lambertian shading color using the physically based Lambertian term (albedo / pi)
	rayData.color = shadowMult * LdotN * lightIntensity * shadeData.diffuse / M_PI;
}

// How do we shade our g-buffer and spawn indirect and shadow rays?
[shader("raygeneration")]
void SimpleDiffuseGIRayGen()
{
	// Where is this ray on screen?
	uint2 launchIndex = (uint2)DispatchRaysIndex().xy;
	uint2 launchDim   = (uint2)DispatchRaysDimensions().xy;

	// Load g-buffer data
	float4 worldPos     = gPos[launchIndex];
	float4 worldNorm    = gNorm[launchIndex];
	float4 difMatlColor = gDiffuseMatl[launchIndex];

	// If we don't hit any geometry, our difuse material contains our background color.
	float3 shadeColor = worldPos.w != 0.0f ? float3(0,0,0) : difMatlColor.rgb;

	// Initialize our random number generator
    uint randSeed = rand_init(launchIndex.x + launchIndex.y * launchDim.x, gFrameCount, 16);

	// Our camera sees the background if worldPos.w is 0, only do diffuse shading & GI elsewhere
	if (worldPos.w != 0.0f)
	{
		// Pick a random light from our scene to sample for direct lighting
        int gLightsCount = gScene.lightCount;
		int lightToSample = min(int(rand_next(randSeed) * gLightsCount), gLightsCount - 1);
        // LightData light = gScene.getLight(lightToSample);

		// We need to query our scene to find info about the current light
		float distToLight;
		float3 lightIntensity;
		float3 toLight;
		getLightData(lightToSample, worldPos.xyz, toLight, lightIntensity, distToLight);
        // LightSample ls = evalLight(light, ShadingData sd)

		// Compute our lambertion term (L dot N)
		float LdotN = saturate(dot(worldNorm.xyz, toLight));

		// Shoot our ray for our direct lighting
		float shadowMult = float(gLightsCount);
		if (gDirectShadow)
			shadowMult *= shadowRayVisibility(worldPos.xyz, toLight, gMinT, distToLight);

		// Compute our Lambertian shading color using the physically based Lambertian term (albedo / pi)
		shadeColor += shadowMult * LdotN * lightIntensity * difMatlColor.rgb / M_PI;

		// Now do our indirect illumination
		if (gDoIndirectGI)
		{
			// Select a random direction for our diffuse interreflection ray.
			float3 bounceDir;
			if (gCosSampling)
				bounceDir = getCosHemisphereSample(randSeed, worldNorm.xyz);      // Use cosine sampling
			else
				bounceDir = getUniformHemisphereSample(randSeed, worldNorm.xyz);  // Use uniform random samples

			// Get NdotL for our selected ray direction
			float NdotL = saturate(dot(worldNorm.xyz, bounceDir));

			// Shoot our indirect global illumination ray
			float3 bounceColor = shootIndirectRay(worldPos.xyz, bounceDir, gMinT, randSeed);

			//bounceColor = (NdotL > 0.50f) ? float3(0, 0, 0) : bounceColor;

			// Probability of selecting this ray ( cos/pi for cosine sampling, 1/2pi for uniform sampling )
			float sampleProb = gCosSampling ? (NdotL / M_PI) : (1.0f / (2.0f * M_PI));

			// Accumulate the color.  For performance, terms could (and should) be cancelled here.
			//shadeColor += (NdotL * bounceColor * difMatlColor.rgb / M_PI) / sampleProb;

			shadeColor += (NdotL * bounceColor * difMatlColor.rgb / M_PI) / sampleProb;
		}
	}
	
	// Save out our AO color
	gOutput[launchIndex] = float4(shadeColor, 1.0f);
}
