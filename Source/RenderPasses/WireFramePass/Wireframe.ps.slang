import Scene.Raster;

cbuffer PerFrameCB
{
    float4 gColor;
};

VSOut vsMain(VSIn vIn)
{
    VSOut vsOut;
    vsOut = defaultVS(vIn);

    return vsOut;
}

float4 main() : SV_TARGET
{
    return gColor;
}
