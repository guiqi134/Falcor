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

/** A shader used to shoot shadow rays at the sample in the reservoir,
    particularly useful for shooting visibility after generating initial candidates
*/

#include "RtxdiBridge.slangh"

cbuffer SampleCB
{
    uint  gReservoirIndex;   // Which of our reservoirs should we use for output?
}

[numthreads(16, 16, 1)]
void main(uint3 threadId : SV_DispatchThreadID)
{
    // Find where we are on the screen and in the reservoir
    const uint2 launchIdx = threadId.xy;
    const RTXDI_ResamplingRuntimeParameters params = gRtxdiParams;
    uint2 pixelPosition = RTXDI_ReservoirToPixelPos(launchIdx, params);

    // Get data about the pixel we're shading from our G-buffer
    RAB_Surface surface = RAB_GetGBufferSurface(pixelPosition, false);

    // Get the current pixel's reservour.
    RTXDI_Reservoir reservoir = RTXDI_LoadReservoir(params, launchIdx, gReservoirIndex);

    // Get the light sample
    uint lightIndex = RTXDI_GetReservoirLightIndex(reservoir);
    float2 lightUV = RTXDI_GetReservoirSampleUV(reservoir);
    RAB_LightInfo lightInfo = RAB_LoadLightInfo(lightIndex, false);
    RAB_LightSample lightSample = RAB_SamplePolymorphicLight(lightInfo, surface, lightUV);

    // Shoot a shadow ray and, if it's shadowed, update the visibility
    if (RAB_IsSurfaceValid(surface) && RTXDI_IsValidReservoir(reservoir))
    {
        if (!RAB_GetConservativeVisibility(surface, lightSample))  // Trace a shadow ray
        {
            RTXDI_StoreVisibilityInReservoir(reservoir, 0, true);
            RTXDI_StoreReservoir(reservoir, params, launchIdx, gReservoirIndex);
        }
    }
}
