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
