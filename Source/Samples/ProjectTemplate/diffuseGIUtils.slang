import Scene.Lights.Lights;
import Scene.Shading;
import Scene.Raytracing;
import Utils.Helpers;


// A helper to extract important light data from internal Falcor data structures.  What's going on isn't particularly
//     important -- any framework you use will expose internal scene data in some way.  Use your framework's utilities.
void getLightData(in int index, in float3 hitPos, out float3 toLight, out float3 lightIntensity, out float distToLight)
{
	// Use built-in Falcor functions and data structures to fill in a LightSample data structure
	//   -> See "Lights.slang" for it's definition
	LightSample ls;

	// Is it a directional light?
	if (gScene.lights[index].type == uint(LightType::Directional))
		ls = evalDirectionalLight(gScene.lights[index], hitPos);

	// No?  Must be a point light.
	else
		ls = evalPointLight(gScene.lights[index], hitPos);

	// Convert the LightSample structure into simpler data
	toLight = normalize(ls.L);
	lightIntensity = ls.diffuse;
	// distToLight = ls.distance;
    distToLight = length(ls.posW - hitPos);
}

bool alphaTestFails(HitShaderParams hitParams, BuiltInTriangleIntersectionAttributes attribs)
{
    // Alpha test for non-opaque geometry.
    VertexData v = getVertexData(hitParams, PrimitiveIndex(), attribs);
    uint materialID = gScene.getMaterialID(hitParams.getGlobalHitID());
    // return alphaTest(v, gScene.materials[materialID], gScene.materialResources[materialID], 0.f);
    return false;
}

ShadingData getHitShadingData(HitShaderParams hitParams, BuiltInTriangleIntersectionAttributes attribs)
{
    VertexData v = getVertexData(hitParams, PrimitiveIndex(), attribs);
    uint materialID = gScene.getMaterialID(hitParams.getGlobalHitID());
    return prepareShadingData(v, materialID, gScene.materials[materialID], gScene.materialResources[materialID], -WorldRayDirection(), 0);
}
// Utility function to get a vector perpendicular to an input vector 
//    (from "Efficient Construction of Perpendicular Vectors Without Branching")
float3 getPerpendicularVector(float3 u)
{
	float3 a = abs(u);
	uint xm = ((a.x - a.y)<0 && (a.x - a.z)<0) ? 1 : 0;
	uint ym = (a.y - a.z)<0 ? (1 ^ xm) : 0;
	uint zm = 1 ^ (xm | ym);
	return cross(u, float3(xm, ym, zm));
}

// Get a cosine-weighted random vector centered around a specified normal direction.
float3 getCosHemisphereSample(inout uint randSeed, float3 hitNorm)
{
	// Get 2 random numbers to select our sample with
	float2 randVal = float2(rand_next(randSeed), rand_next(randSeed));

	// Cosine weighted hemisphere sample from RNG
	float3 bitangent = getPerpendicularVector(hitNorm);
	float3 tangent = cross(bitangent, hitNorm);
	float r = sqrt(randVal.x);
	float phi = 2.0f * 3.14159265f * randVal.y;

	// Get our cosine-weighted hemisphere lobe sample direction
	return tangent * (r * cos(phi).x) + bitangent * (r * sin(phi)) + hitNorm.xyz * sqrt(max(0.0,1.0f - randVal.x));
}

// Get a uniform weighted random vector centered around a specified normal direction.
float3 getUniformHemisphereSample(inout uint randSeed, float3 hitNorm)
{
	// Get 2 random numbers to select our sample with
	float2 randVal = float2(rand_next(randSeed), rand_next(randSeed));

	// Cosine weighted hemisphere sample from RNG
	float3 bitangent = getPerpendicularVector(hitNorm);
	float3 tangent = cross(bitangent, hitNorm);
	float r = sqrt(max(0.0f,1.0f - randVal.x*randVal.x));
	float phi = 2.0f * 3.14159265f * randVal.y;

	// Get our uniform hemisphere lobe sample direction
	return tangent * (r * cos(phi).x) + bitangent * (r * sin(phi)) + hitNorm.xyz * randVal.x;
}