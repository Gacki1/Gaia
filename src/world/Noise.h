#pragma once

#include "math/SpaceMath.h"
#include "math/WorldMath.h"
#include <array>
#include <cmath>
#include <cstdint>

namespace planet {

class Noise {
public:
    explicit Noise(unsigned seed) { reseed(seed); }

    void reseed(unsigned seed) {

        std::array<int, 256> p{};
        for (int i = 0; i < 256; ++i) p[i] = i;
        uint32_t state = seed ? seed : 0x9E3779B9u;
        auto next = [&state]() { state = state * 1664525u + 1013904223u; return state; };
        for (int i = 255; i > 0; --i) {
            int j = static_cast<int>(next() % static_cast<uint32_t>(i + 1));
            std::swap(p[i], p[j]);
        }
        for (int i = 0; i < 512; ++i) perm_[i] = p[i & 255];
    }

    float perlin(float x, float y, float z) const {
        const int X = fastFloor(x) & 255;
        const int Y = fastFloor(y) & 255;
        const int Z = fastFloor(z) & 255;
        return perlinCell(X, Y, Z, x - std::floor(x), y - std::floor(y), z - std::floor(z));
    }

    float perlin(double x, double y, double z) const {
        const double fx = std::floor(x), fy = std::floor(y), fz = std::floor(z);

        const int X = static_cast<int>(std::fmod(fx, 256.0)) & 255;
        const int Y = static_cast<int>(std::fmod(fy, 256.0)) & 255;
        const int Z = static_cast<int>(std::fmod(fz, 256.0)) & 255;
        return perlinCell(X, Y, Z,
                          static_cast<float>(x - fx),
                          static_cast<float>(y - fy),
                          static_cast<float>(z - fz));
    }

    float fbm(const space::Vec3& pos, int octaves, float frequency,
              float lacunarity, float gain) const {
        float amp = 1.0f, freq = frequency, sum = 0.0f, norm = 0.0f;
        for (int i = 0; i < octaves; ++i) {
            sum  += amp * perlin(pos.x * freq, pos.y * freq, pos.z * freq);
            norm += amp;
            amp  *= gain;
            freq *= lacunarity;
        }
        return norm > 0.0f ? sum / norm : 0.0f;
    }

    float fbm(const space::DVec3& pos, int octaves, float frequency,
              float lacunarity, float gain) const {
        float amp = 1.0f, sum = 0.0f, norm = 0.0f;
        double freq = frequency;
        for (int i = 0; i < octaves; ++i) {
            sum  += amp * perlin(pos.x * freq, pos.y * freq, pos.z * freq);
            norm += amp;
            amp  *= gain;
            freq *= lacunarity;
        }
        return norm > 0.0f ? sum / norm : 0.0f;
    }

    float fbmPrefix(const space::DVec3& dir, int octaves, float frequency,
                    float lacunarity, float gain, int normOctaves) const {
        float amp = 1.0f, sum = 0.0f;
        double freq = frequency;
        for (int i = 0; i < octaves; ++i) {
            sum  += amp * perlin(dir.x * freq, dir.y * freq, dir.z * freq);
            amp  *= gain;
            freq *= lacunarity;
        }
        return sum / fixedNorm(gain, normOctaves);
    }

    static float fixedNorm(float gain, int n) {
        float amp = 1.0f, norm = 0.0f;
        for (int i = 0; i < n; ++i) { norm += amp; amp *= gain; }
        return norm > 0.0f ? norm : 1.0f;
    }

    struct DualFbm { float plain, ridged; };

    DualFbm fbmDual(const space::DVec3& dir, int octaves, float frequency,
                    float lacunarity, float gain, int ridgeOctaves) const {
        float amp = 1.0f, sumP = 0.0f, sumR = 0.0f, norm = 0.0f, normR = 0.0f;

        float weight = 1.0f;
        double freq = frequency;
        for (int i = 0; i < octaves; ++i) {
            const float n = perlin(dir.x * freq, dir.y * freq, dir.z * freq);
            sumP += amp * n;

            if (i < ridgeOctaves) {
                float sig = ridge(n) * weight;
                weight = sig * kRidgeMultiGain;
                weight = weight < 0.0f ? 0.0f : (weight > 1.0f ? 1.0f : weight);
                sumR  += amp * sig;
                normR += amp;
            }

            norm += amp;
            amp  *= gain;
            freq *= lacunarity;
        }
        DualFbm o{ 0.0f, 0.0f };
        if (norm  > 0.0f) o.plain  = sumP / norm;
        if (normR > 0.0f) o.ridged = sumR / normR;
        return o;
    }

    static constexpr float kRidgeMultiGain = 1.0f;

    static float ridge(float n) {
        const float r = 1.0f - std::fabs(n);
        return r * r;
    }

    static constexpr float kRidgeSoften = 0.28f;

    static float ridgeSoft(float n) {
        const float k = 1.0f / (std::sqrt(1.0f + kRidgeSoften * kRidgeSoften)
                                - kRidgeSoften);
        const float a = (std::sqrt(n * n + kRidgeSoften * kRidgeSoften)
                         - kRidgeSoften) * k;
        const float r = 1.0f - a;
        return r * r;
    }

    static float ridgeSigned(float n) { return 2.0f * ridge(n) - 1.0f; }

    static constexpr float kRidgeMean = 0.26108f;

    static constexpr float kRidgeSoftMean = 0.55887f;

    static float ridgeSoftSigned(float n) { return 2.0f * ridgeSoft(n) - 1.0f; }

    static float ridgeBalanced(float n) {
        return (ridgeSoftSigned(n) - kRidgeSoftMean) / (1.0f + kRidgeSoftMean);
    }

    static float ridgeMixForOctave(int i, float mix, int full, int fade) {
        if (i < full) return mix;
        if (fade <= 0) return 0.0f;
        const float t = static_cast<float>(i - full + 1) / static_cast<float>(fade + 1);
        return t >= 1.0f ? 0.0f : mix * (1.0f - t);
    }

    float fbmPrefixRidged(const space::DVec3& dir, int octaves, float frequency,
                          float lacunarity, float gain, int normOctaves,
                          float ridgeMix, int ridgeFull, int ridgeFade) const {
        float amp = 1.0f, sum = 0.0f;
        double freq = frequency;
        for (int i = 0; i < octaves; ++i) {
            const float n = perlin(dir.x * freq, dir.y * freq, dir.z * freq);
            const float m = ridgeMixForOctave(i, ridgeMix, ridgeFull, ridgeFade);
            sum  += amp * (n + m * (ridgeBalanced(n) - n));
            amp  *= gain;
            freq *= lacunarity;
        }
        return sum / fixedNorm(gain, normOctaves);
    }

private:
    using Vec3 = space::Vec3;

    float perlinCell(int X, int Y, int Z, float x, float y, float z) const {
        const float u = fade(x), v = fade(y), w = fade(z);

        const int A  = perm_[X] + Y,   AA = perm_[A] + Z,   AB = perm_[A + 1] + Z;
        const int B  = perm_[X+1] + Y, BA = perm_[B] + Z,   BB = perm_[B + 1] + Z;

        return lerp(w,
            lerp(v, lerp(u, grad(perm_[AA],   x,   y,   z),
                            grad(perm_[BA],   x-1, y,   z)),
                    lerp(u, grad(perm_[AB],   x,   y-1, z),
                            grad(perm_[BB],   x-1, y-1, z))),
            lerp(v, lerp(u, grad(perm_[AA+1], x,   y,   z-1),
                            grad(perm_[BA+1], x-1, y,   z-1)),
                    lerp(u, grad(perm_[AB+1], x,   y-1, z-1),
                            grad(perm_[BB+1], x-1, y-1, z-1))));
    }

    static int   fastFloor(float v) { int i = static_cast<int>(v); return (v < i) ? i - 1 : i; }
    static float fade(float t)      { return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f); }
    static float lerp(float t, float a, float b) { return a + t * (b - a); }
    static float grad(int hash, float x, float y, float z) {
        const int h = hash & 15;
        const float u = h < 8 ? x : y;
        const float v = h < 4 ? y : (h == 12 || h == 14 ? x : z);
        return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
    }

    std::array<int, 512> perm_{};
};

}
