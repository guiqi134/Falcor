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

/* This is a glorified copy shader.  It copies the environment map lightprobe
   into a grayscale PDF texture, with each texel weighted by it's (relative)
   solid angle.  We use this to importance sample probe texels to preferentially
   sample brighter texels.
*/

import Scene.Scene;
import Utils.Math.MathHelpers;
import Utils.Color.ColorHelpers;
#include "Utils/Math/MathConstants.slangh"

 // Our output PDF texture used for importance sampling primitves
RWTexture2D<float> gEnvMapPDFTexture;

[numthreads(16, 16, 1)]
void main(uint3 threadId : SV_DispatchThreadID)
{
    uint2 envSize = gScene.envMap.getDimensions();
    if (threadId.x >= envSize.x || threadId.y >= envSize.y)
        return;

    float4 envMapColor = gScene.envMap.envMap[threadId.xy];

    // Get a relative solid angle of texels in this row.  (Doesn't have to be absolutely correct, just
    // relatively, since traversing the PDF texture only relies on correct relative weights)
    float elevation = (((float(threadId.y) + 0.5) / float(envSize.y)) - 0.5f) * M_PI;
    float relativeSolidAngle = cos(elevation);

    // What's our texel weight?
    float weight = relativeSolidAngle * max(luminance(envMapColor.rgb), 0.0f);

    // Save our evnironment map's texel weight to the pdf texture.
    gEnvMapPDFTexture[threadId.xy] = clamp(weight, 0.0f, 65504.0);   // Clamp to 16-bit float range.
}
