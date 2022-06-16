/***************************************************************************
 # Copyright (c) 2015-21, NVIDIA CORPORATION. All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions
 # are met:
 #  * Redistributions of source code must retain the above copyright
 #    notice, this list of conditions and the following disclaimer.
 #  * Redistributions in binary form must reproduce the above copyright
 #    notice, this list of conditions and the following disclaimer in the
 #    documentation and/or other materials provided with the distribution.
 #  * Neither the name of NVIDIA CORPORATION nor the names of its
 #    contributors may be used to endorse or promote products derived
 #    from this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS "AS IS" AND ANY
 # EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 # IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 # PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 # CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 # PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 # PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 # OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 # OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **************************************************************************/

/** This shader is, perhaps, not necessary.
  
   The idea is to load surface / primary hit / G-buffer data into a format the RTXDI
   bridge understands and uses for resampling and computing lighting.
  
   Why not just load the primary hits / G-buffer data directly into the RAB_Surface
   structure required by RTXDI?  This might be a fine idea, and it's certainly doable.
   However, RAB_Surfaces are loaded multiple times during resampling, whereas
   traditional G-buffers might be read once during final shading or examined one
   texture plane at a time.  It may be wise to compress or otherwise reformat the
   data knowing this access pattern, in order to reduce overhead during resampling.
  
   For this Falcor demonstration, we use a V-buffer, so full primary hit data has
   *never* been generated previously.  Since geometry positions, normals, and other
   attributes are stored somewhat differently (in Falcor) than raster geometry
   attributes, this pass creates a contiguous element per pixel that plays well with
   GPU caches.  By doing this a single time, rather than when loading a RAB_Surface
   we can save bandwidth.
*/

import Scene.Scene;
import Scene.Shading;
import Scene.HitInfo;
import Scene.Material.ShadingUtils;
import Utils.Math.MathHelpers;
import Utils.Math.PackedFormats;
import Utils.Math.BitTricks;
import Utils.Color.ColorHelpers;
import Rendering.Materials.TexLODHelpers;
#include "Utils/Math/MathConstants.slangh"

cbuffer CB
{
    uint2 gScreenRes;
    bool  gEnvironmentLightPresent;
    float gRelativeTrianglePower;
    float gRelativeEnvironmentPower;
    bool  gUsePrimaryHitTextureLoD;   // Use texture gradiants for G-buffer (should always be true?)
    uint  gGBufferOffset;             // We have 2 G-buffers we bounce between.  Where should we write now?
}

// The geometry buffer we're creating to pass to RTXDI
RWBuffer<float4> gOutputGeom;

// An image containing directly-seen emission from lights & environment maps
RWTexture2D<float4> gOutputEmissives;

// We're taking in a standard Falcor V-buffer
Texture2D<PackedHitInfo> gVbuffer;


uint packRGBAtoUint(float4 c)
{
    uint4 packed = uint4(round(saturate(c) * 255.0));
    return packed.x | (packed.y << 8u) | (packed.z << 16u) | (packed.w << 24u);
}

[numthreads(16, 16, 1)]
void main(uint3 threadId : SV_DispatchThreadID)
{
    if (any(threadId.xy >= gScreenRes)) return;

    // Convert pixel ID to buffer location for passing data to RTXDI.
    uint bufferIndex = threadId.x + (threadId.y * gScreenRes.x);

    // Shading structures
    ShadingData sd = {};
    let lod = ExplicitLodTextureSampler(0.f);

    // Flag to see, when we load our shading data for the pixel, if we have a valid surface (or instead the background)
    bool isValidSurface = false;

    // Reconstruct our ray direction from the pixel ID (assuming a pinhole camera model)
    float3 rayDir = gScene.camera.computeRayPinhole(threadId.xy, gScreenRes).dir;

    // Load the shading data for this pixel, if it exists.
    {
        // Read from the V-buffer
        HitInfo hit = HitInfo(gVbuffer[threadId.xy]);
        if (hit.isValid() && hit.getType() == HitType::Triangle)
        {
            // Extract a triangle hit from the V-buffer
            const TriangleHit triangleHit = hit.getTriangleHit();

            // Evaluate Falcor's material parameters at the hit point.
            const VertexData v = gScene.getVertexData(triangleHit);
            const uint materialID = gScene.getMaterialID(triangleHit.instanceID);
            sd = gScene.materials.prepareShadingData(v, materialID, -rayDir, lod);

            // Adjust shading normals to reduce black pixels from "back facing" shading
            // normals that can arise from perturbing geometric normals via normal maps
            adjustShadingNormal(sd, v);
            isValidSurface = true;
        }

        sd.V = -rayDir;
    }

    // Does this pixel contain a visible triangle?
    if (isValidSurface)
    {
        // Create BSDF instance and query its properties.
        let bsdf = gScene.materials.getBSDF(sd, lod);
        let bsdfProperties = bsdf.getProperties(sd);

        // Is the material emissive?
        float3 emissiveColor = float3(0.0f);
        if (sd.mtl.isEmissive())
        {
            emissiveColor = gRelativeTrianglePower * gScene.materials.evalEmissive(sd.materialID, sd.uv);
        }

        // For emissive surfaces, we need to store out emissive color to a buffer
        gOutputEmissives[threadId.xy] = float4(emissiveColor, 1.0f);

        // What's the ratio of specular to diffuse lobe?
        float weightDif = luminance(bsdfProperties.diffuseReflectionAlbedo);
        float sumWeights = weightDif + luminance(bsdfProperties.specularReflectance);
        float dPdf = sumWeights < 1e-7f ? 1.0f : weightDif / sumWeights;

        // Compute output data
        float3 offsetHit = sd.computeNewRayOrigin();
        float4 diffuse = float4(bsdfProperties.diffuseReflectionAlbedo, dPdf);
        float4 specular = float4(bsdfProperties.specularReflectance, bsdfProperties.roughness);

        // Output the geometry hit data in our desired packed format
        gOutputGeom[gGBufferOffset + bufferIndex] = float4(
            asfloat(encodeNormal2x16(sd.N)),         // A packed shading normal
            asfloat(packRGBAtoUint(diffuse)),        // A packed diffuse BRDF color (plus importance of diffuse lobe)
            asfloat(packRGBAtoUint(specular)),       // A packed specular BRDF color (plus a surface roughness)
            distance(gScene.camera.getPosition(),    // A distance from camera to the hit point.
                 sd.computeNewRayOrigin()));
    }
    else
    {
        // For invalid pixels (i.e., the background) set to the correct "invalid" buffer element
        gOutputGeom[gGBufferOffset + bufferIndex] = float4(0.0f, 0.0f, 0.0f, -1.0f);

        // Output the visible environment map color if we have one.
        if (gEnvironmentLightPresent)
        {
            gOutputEmissives[threadId.xy] = float4(gRelativeEnvironmentPower * gScene.envMap.eval(rayDir), 1.0f);
        }
    }  
}
