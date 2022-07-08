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
import Scene.Camera.Camera;
import SSAOData;

cbuffer StaticCB
{
    SSAOData gData;
}

cbuffer PerFrameCB
{
    Camera gCamera;
}

SamplerState gNoiseSampler; // point + wrap sampler 
SamplerState gTextureSampler; // linear + clamp sampler

Texture2D gDepthTex;
Texture2D gNormalTex;
Texture2D gNoiseTex;

float4 getPosition(float2 uv)
{
    float4 pos;
    pos.x = uv.x * 2.0f - 1.0f;
    pos.y = (1.0f - uv.y) * 2.0f - 1.0f;
#ifdef FALCOR_FLIP_Y
    // NDC Y is top-to-bottom
    pos.y = -pos.y;
#endif
    pos.z = gDepthTex.SampleLevel(gTextureSampler, uv, 0).r;
    pos.w = 1.0f;

    float4 posW = mul(pos, gCamera.data.invViewProj);
    posW /= posW.w;

    return posW;
}

float4 main(float2 texC : TEXCOORD) : SV_TARGET0
{
    if (gDepthTex.SampleLevel(gTextureSampler, texC, 0).r >= 1)
    {
        return 1.0f;
    }

    // Calculate world position of pixel, z is the depth from the depth map
    float3 posW = getPosition(texC).xyz;

    float3 normal = normalize(gNormalTex.Sample(gTextureSampler, texC).xyz * 2.0f - 1.0f); // world space normal
    float originDist = length(posW - gCamera.data.posW);

    // NoiseScale = screen size / noise texture size (16)
    float3 randDir = gNoiseTex.Sample(gNoiseSampler, texC * gData.noiseScale).xyz * 2.0f - 1.0f; // z = -1.0f?

    float3 tangent = normalize(randDir - normal * dot(randDir, normal));
    float3 bitangent = cross(normal, tangent);
    float3x3 tbn = float3x3(tangent, bitangent, normal);

    float occlusion = 0.0f;
    for (uint i = 0; i < gData.kernelSize; i++)
    {
        // First we need to transfer the sampler position from current pixel's local tangent
        // space all the way along to the screen space 

        // Transform hemispherical samples from local tangent space to world space
        float3 kernelPos = mul(gData.sampleKernel[i].xyz, tbn); 

        // Calculate sample world space pos
        float3 samplePosW = posW + (kernelPos * gData.radius);
        float sampleDist = length(samplePosW - gCamera.data.posW);

        // Get screen space pos of sample
        float4 samplePosProj = mul(float4(samplePosW, 1.0f), gCamera.data.viewProjMat);
        samplePosProj /= samplePosProj.w;

        // Sample depth buffer at the same place as sample
#ifdef FALCOR_FLIP_Y
        // NDC Y is top-to-bottom
        samplePosProj.y = -samplePosProj.y;
#endif
        float2 sampleUV = saturate(float2(samplePosProj.x, -samplePosProj.y) * 0.5f + 0.5f);

        // Depth value for sample's projected position in screen
        float sampleDepth = length(getPosition(sampleUV).xyz - gCamera.data.posW);

        // We introduce a range check that makes sure a pixel contributes to the occlusion factor 
        // if its depth values is within the sample's radius.
        // float rangeCheck = step(abs(sampleDist - sampleDepth), gData.radius); // radius >= difference -> 1
        float rangeCheck = abs(originDist - sampleDepth) < gData.radius ? 1.0 : 0.0;
        occlusion += step(sampleDepth, sampleDist) * rangeCheck; // sampleDepth < sampleDist -> occluded
    }

    float factor = 1 - (occlusion / float(gData.kernelSize));
    return float4(factor.xxx, 1);
}
