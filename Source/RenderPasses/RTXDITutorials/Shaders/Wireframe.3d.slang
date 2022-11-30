#include "../HostDeviceSharedDefinitions.h"

import Scene.Raster;
import Scene.Scene;

cbuffer PerFrameCB
{
    float4 gColor;
};

struct GSOut
{
    float4 posH : SV_Position;
}

VSOut vsMain(VSIn vIn)
{
    VSOut vsOut;
    vsOut = defaultVS(vIn);

    return vsOut;
}

[maxvertexcount(3)]
// void gsMain(triangle VSOut gsIn[3], inout PointStream<GSOut> outStream)
void gsMain(triangle VSOut gsIn[3], inout TriangleStream<GSOut> outStream)
{
    GSOut gsOut;
    for (uint v = 0; v < 3; v++)
    {
        gsOut.posH = gsIn[v].posH;
        outStream.Append(gsOut);
    }
}

float4 psMain() : SV_TARGET
{
    return gColor;
}
