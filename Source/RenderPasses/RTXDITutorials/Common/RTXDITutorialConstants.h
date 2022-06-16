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
#pragma once

#include "Falcor.h"

// Constants for shader filenames and directories
extern const std::string kShaderDirectory;
extern const std::string kRTXDI_CreateLightResources;
extern const std::string kRTXDI_CreateEnvMapPDFTexture;
extern const std::string kRTXDI_PresamplePrimitiveLights;
extern const std::string kRTXDI_PresampleEnvironmentLights;
extern const std::string kRTXDI_PrepareSurfaceData;
extern const std::string kRTXDI_MonteCarloBaseline;
extern const std::string kRTXDI_RISOnly_InitialSamples;
extern const std::string kRTXDI_RISOnly_Shade;
extern const std::string kRTXDI_InitialSamples;
extern const std::string kRTXDI_InitialVisibility;
extern const std::string kRTXDI_Shade;
extern const std::string kRTXDI_SpatialReuseOnly;
extern const std::string kRTXDI_TemporalReuseOnly;
extern const std::string kRTXDI_SpatiotemporalReuse;
extern const std::string kRTXDI_InitialSamplesForDecoupled;
extern const std::string kRTXDI_VisibilityForDecoupled;
extern const std::string kRTXDI_DecoupledShade;

// GUI constant identifiers (to use with buttons / dropdowns)
inline constexpr  uint32_t  kUniformSamplingMode          = 0u;
inline constexpr  uint32_t  kPdfSamplingMode              = 1u;
inline constexpr  uint32_t  kPrecomputedTileSamplingMode  = 2u;
inline constexpr  uint32_t  kMonteCarloBaselineMode       = 0u;   
inline constexpr  uint32_t  kTalbotRISMode                = 1u;
inline constexpr  uint32_t  kSpatialOnlyReuseMode         = 2u;
inline constexpr  uint32_t  kTemporalOnlyReuseMode        = 3u;
inline constexpr  uint32_t  kSpatiotemporalReuseMode      = 4u;
inline constexpr  uint32_t  kDecoupledReuseMode           = 5u;

// GUI dropdown / button constants
extern const Falcor::Gui::RadioButtonGroup kSampleList;
extern const Falcor::Gui::DropdownList kBiasCorrection;
extern const Falcor::Gui::DropdownList kDropList;

// GUI tooltip strings
extern const std::string kSampleCountToolTip;
extern const std::string kCandidateVisibilityToolTip;
extern const std::string kPerTileBuffersToolTip;
extern const std::string kSamplingTypeToolTip;
extern const std::string kNaiveSampleToolTip;
extern const std::string kBaiscRISSampleToolTip;
extern const std::string kPrecomputedTileToolTip;
