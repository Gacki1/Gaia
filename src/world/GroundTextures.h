#pragma once

#include "world/Noise.h"
#include "world/PlanetConfig.h"
#include <vector>
#include <cstdint>
#include <cmath>

namespace planet {

inline constexpr int kGroundTexSize   = 256;

inline constexpr int kGroundTexLayers = kBiomeCount - 1;

inline float tileNoise(const Noise& n, float u, float v, float freq) {
    const float tau = 6.2831853f;
    const float a = u * tau, b = v * tau;

    const float r = freq / tau;
    return n.perlin(std::cos(a) * r, std::sin(a) * r + std::cos(b) * r,
                    std::sin(b) * r + 11.0f);
}

inline float tileFbm(const Noise& n, float u, float v, float freq, int octaves) {
    float sum = 0.0f, amp = 1.0f, norm = 0.0f, f = freq;
    for (int i = 0; i < octaves; ++i) {
        sum  += amp * tileNoise(n, u, v, f);
        norm += amp;
        amp  *= 0.5f;
        f    *= 2.0f;
    }
    return norm > 0.0f ? sum / norm : 0.0f;
}

inline std::vector<uint8_t> buildGroundTextures(unsigned seed = kNoiseSeed) {
    Noise n(seed);
    const int S = kGroundTexSize;
    std::vector<uint8_t> out(size_t(S) * S * 4 * kGroundTexLayers);

    for (int layer = 0; layer < kGroundTexLayers; ++layer) {
        for (int y = 0; y < S; ++y) {
            for (int x = 0; x < S; ++x) {
                const float u = (x + 0.5f) / S, v = (y + 0.5f) / S;
                float g = 0.0f;
                if (layer == 0) {

                    g = 0.5f + 0.40f * tileFbm(n, u, v, 18.0f, 4);
                } else if (layer == 1) {

                    const float warp = 0.12f * tileFbm(n, u, v, 3.0f, 2);
                    const float band = std::sin((v + warp) * 6.2831853f * 9.0f);
                    g = 0.5f + 0.26f * band + 0.18f * tileFbm(n, u, v, 24.0f, 3);
                } else {

                    const float f = tileFbm(n, u, v, 7.0f, 5);
                    const float crack = 1.0f - std::fabs(f);
                    g = 0.34f + 0.52f * crack * crack;
                }
                g = g < 0.0f ? 0.0f : (g > 1.0f ? 1.0f : g);
                const uint8_t c = uint8_t(std::lround(g * 255.0f));
                uint8_t* px = &out[((size_t(layer) * S + y) * S + x) * 4];
                px[0] = c; px[1] = c; px[2] = c; px[3] = 255;
            }
        }
    }
    return out;
}

}
