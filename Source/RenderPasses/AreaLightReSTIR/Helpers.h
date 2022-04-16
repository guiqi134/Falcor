#pragma once

void FillNeighborOffsetBuffer(uint8_t* buffer)
{
    //int R = 250; // ???
    int R = 2;
    const float phi2 = 1.0f / 1.3247179572447f;
    uint32_t num = 0;
    float u = 0.5f;
    float v = 0.5f;
    while (num < 8192 * 2)
    {
        u += phi2; // ~ 0.755
        v += phi2 * phi2; // ~ 0.570
        if (u >= 1.0f) u -= 1.0f; // [0, 1)
        if (v >= 1.0f) v -= 1.0f; // [0, 1)

        float rSq = (u - 0.5f) * (u - 0.5f) + (v - 0.5f) * (v - 0.5f);
        if (rSq > 0.25f)
        {
            continue;
        }

        buffer[num++] = int8_t((u - 0.5f) * R);
        buffer[num++] = int8_t((v - 0.5f) * R);
    }
}

Texture::SharedPtr createNeighborOffsetTexture(uint32_t sampleCount)
{
    std::unique_ptr<int8_t[]> offsets(new int8_t[sampleCount * 2]);
    const int R = 254;
    const float phi2 = 1.f / 1.3247179572447f;
    float u = 0.5f;
    float v = 0.5f;
    for (uint32_t index = 0; index < sampleCount * 2;)
    {
        u += phi2;
        v += phi2 * phi2;
        if (u >= 1.f) u -= 1.f;
        if (v >= 1.f) v -= 1.f;

        float rSq = (u - 0.5f) * (u - 0.5f) + (v - 0.5f) * (v - 0.5f);
        if (rSq > 0.25f) continue;

        offsets[index++] = int8_t((u - 0.5f) * R);
        offsets[index++] = int8_t((v - 0.5f) * R);
    }

    return Texture::create1D(sampleCount, ResourceFormat::RG8Snorm, 1, 1, offsets.get());
}


Texture::SharedPtr createLightSamplesTexture(const Scene::SharedPtr& pScene, RenderContext* pRenderContext, uint sampleCount,
    float4x4 lightView)
{
    std::random_device device;
    std::mt19937 rng(device());
    std::uniform_real_distribution<float> dist(0.0, 1.0);

    const auto& pLightCollection = pScene->getLightCollection(pRenderContext);
    uint triangleCount = pLightCollection->getActiveLightCount();
    //logInfo("ActiveLightCount = " + std::to_string(triangleCount));

    std::uniform_int_distribution<int> dist_int(0, triangleCount - 1);

    std::unique_ptr<float2[]> samples(new float2[sampleCount]);
    for (uint i = 0; i < sampleCount; i++)
    {
        // Select a mesh light triangle 
        float3 randoms = float3(dist(rng), dist(rng), dist(rng));
        uint triangleIndex = std::min((uint)(randoms.x * triangleCount), triangleCount - 1);
        auto triangle = pLightCollection->getMeshLightTriangles()[triangleIndex];
        auto vertices = triangle.vtx;

        // Calculate barycentrics
        float2 u = randoms.yz;
        float su = sqrt(u.x);
        float2 b = float2(1.f - su, u.y * su); 
        float3 barycentrics = float3(1.f - b.x - b.y, b.x, b.y);

        // Get sample position in light view space
        float3 samplePosW = vertices[0].pos * barycentrics[0] +
                vertices[1].pos * barycentrics[1] +
                vertices[2].pos * barycentrics[2];
        float3 samplePosLightView = lightView * float4(samplePosW, 1.0f);

        samples[i] = samplePosLightView.xy;
    }

    for (uint i = 0; i < 20; i++)
        logInfo("samples[i] = " + to_string(samples[i]));

    return Texture::create1D(sampleCount, ResourceFormat::RG32Float, 1, 1, samples.get());
}
