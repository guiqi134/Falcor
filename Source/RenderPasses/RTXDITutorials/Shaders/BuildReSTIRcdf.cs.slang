#include "../HostDeviceSharedDefinitions.h"

import Utils.Debug.PixelDebug;

Buffer<uint> gLightHistogramBuffer;
Buffer<uint> gTotalValidPixels;
StructuredBuffer<LightShadowMapData> gLightShadowDataBuffer;
RWBuffer<float> gLightPdfBuffer;

[numthreads(256, 1, 1)]
void buildPDF(uint3 threadID : SV_DispatchThreadID)
{
    uint lightID = threadID.x;

    // Sum of pixels belong to ISM lights
    float sum = gTotalValidPixels[1];

    // Only compute the PDF for ISM lights. SM lights set to 0.0
    gLightPdfBuffer[lightID] = gLightShadowDataBuffer[lightID].shadowMapType == 2 ? float(gLightHistogramBuffer[lightID]) / sum : 0.0f;
    // gLightPdfBuffer[lightID] = float(gLightHistogramBuffer[lightID]) / sum;
}

RWBuffer<float> gLightCdfBuffer;

// TODO: Computing CDF is just a prefix sum. Change to parallel reduction tree version if this part is slow
[numthreads(256, 1, 1)]
void buildCDF(uint3 threadID: SV_DispatchThreadID)
{
    float cdf = 0.0f;
    for (uint i = 0; i < threadID.x; i++)
    {
        cdf += gLightPdfBuffer[i];
    }

    gLightCdfBuffer[threadID.x] = cdf;
}
