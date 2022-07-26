import Scene.Raytracing;

cbuffer CB
{
    float gFarPlaneZ;
}

Texture2D gRasterColorTexture;
RWTexture2D gZBufferTexture;

[numthreads(16, 16)]
void main(uint3 threadId : SV_DispatchThreadID)
{
    float2 colorTexSize;
    gRasterColorTexture.GetDimensions(colorTexSize.x, colorTexSize.y);

    if (threadId.x < colorTexSize.x && threadId.y < colorTexSize.y)
    {
        gZBufferTexture[threadId.xy] = gRasterColorTexture[threadId.xy];
    }
    else
    {
        gZBufferTexture[threadId.xy] = gFarPlaneZ;
    }
}