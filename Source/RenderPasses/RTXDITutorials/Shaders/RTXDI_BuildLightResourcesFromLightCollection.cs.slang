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



import Scene.Scene;
import Utils.Math.MathHelpers;
import Utils.Math.PackedFormats;
import Utils.Math.BitTricks;
import Utils.Color.ColorHelpers;
#include "Utils/Math/MathConstants.slangh"

// We don't need all of RTXDI or the bridge for this shader, just 
// RTXDI_LinearIndexToZCurve(), which is in RTXDI's math header.
#include "rtxdi/RtxdiMath.hlsli"

// Some parameters
cbuffer CB
{
    uint gLaunchWidth;
    uint gNumPrimitives;
    bool gUpdatePrimitiveGeometry;
    bool gUpdatePrimitiveFlux;
}

// Our output PDF texture used for importance sampling primitves
RWTexture2D<float> gPrimitivePDFTexture;

// Output packed light data for later RTXDI consumption.
RWBuffer<float4>   gPackedLightGeom;

[numthreads(16, 16, 1)]
void main(uint3 threadId : SV_DispatchThreadID)
{
    // What light primitive are we processing?  (i.e., get a linear ID).
    //   -> Falcor has "active" emissive lights (i.e., those that *actually* emit light
    //      as opposed to those that are textured with a black emissive texture)
    uint activeTriId = threadId.y * gLaunchWidth + threadId.x;

    // Our launch may be too large; if that's not a valid primitive, stop
    if (activeTriId >= gNumPrimitives)
        return;

    // Get our mapping from active emissive triangles into the full list of potential emissives.
    uint triId = gScene.lightCollection.activeTriangles[activeTriId];

    // Get the unpacked geometry info about that triangle
    EmissiveTriangle triGeom = gScene.lightCollection.getTriangle(triId);
    EmissiveFlux     triFlux = gScene.lightCollection.fluxData[triId];

    // Light geometry may have moved, so update packed values representing positions / normals
    if (gUpdatePrimitiveGeometry)
    {
        // Compute the triangle edges
        float3 edge0 = triGeom.posW[1] - triGeom.posW[0];
        float3 edge1 = triGeom.posW[2] - triGeom.posW[0];

        // Pack this data into the format used for our RTXDI bridge.  
        uint3  packedEdges = uint3((f32tof16(edge0) << 16u) | f32tof16(edge1));
        gPackedLightGeom[(activeTriId << 1u) + 0u] = float4(triGeom.posW[0], 0.0f);
        gPackedLightGeom[(activeTriId << 1u) + 1u] = float4(asfloat(packedEdges.xyz), triFlux.flux);
    }

    // Triangle emission may have changed, so update values representing flux
    if (gUpdatePrimitiveFlux)
    {
        // Find where in the pdf texture RTXDI expects to find this primitive.
        uint2 pdfTexturePosition = RTXDI_LinearIndexToZCurve(activeTriId);

        // Save a weight RTXDI uses to importance sample our emissives (i.e., proprtional to flux)
        gPrimitivePDFTexture[pdfTexturePosition] = triFlux.flux;
    }
}
