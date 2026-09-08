#pragma once

#include "world/PlanetConfig.h"
#include "math/SpaceMath.h"

#include <cmath>
#include <vector>
#include <algorithm>

namespace planet {
namespace atmo {

using space::Vec3;

inline constexpr double kTopRadiiD = kAtmosphereTop / kPlanetRadius;
inline constexpr float  kTopRadii  = float(kTopRadiiD);

inline const float kHorizonSpan = float(std::sqrt(kTopRadiiD * (kTopRadiiD + 2.0)));

inline Vec3 mulc(const Vec3& a, const Vec3& b) { return { a.x * b.x, a.y * b.y, a.z * b.z }; }

inline const Vec3 kBetaRayleigh{ float(kBetaRayleighR), float(kBetaRayleighG),
                                 float(kBetaRayleighB) };

struct Hit {
    bool  hit;
    float t0;
    float t1;
};

inline Hit intersectShell(float a, float mu, float s) {
    const float b    = (1.0f + a) * mu;
    const float c    = (a - s) * (a + s + 2.0f);
    const float disc = b * b - c;
    if (disc < 0.0f) return { false, 0.0f, 0.0f };
    const float sq = std::sqrt(disc);
    return { true, -b - sq, -b + sq };
}

inline float distanceToTop(float a, float mu) {
    const Hit h = intersectShell(a, mu, kTopRadii);
    return h.hit ? std::max(h.t1, 0.0f) : 0.0f;
}

inline float distanceToGround(float a, float mu) {
    const Hit h = intersectShell(a, mu, 0.0f);
    if (!h.hit || h.t0 < 0.0f) return -1.0f;
    return h.t0;
}

inline bool hitsGround(float a, float mu) { return distanceToGround(a, mu) >= 0.0f; }

struct MarchSpan {
    float t0;
    float t1;
    bool  empty() const { return t1 <= t0; }
};

inline MarchSpan marchSpan(float a, float mu) {
    MarchSpan s{ 0.0f, -1.0f };
    if (a <= kTopRadii) {
        s.t0 = 0.0f;
        s.t1 = distanceToTop(a, mu);
    } else {
        const Hit h = intersectShell(a, mu, kTopRadii);
        if (!h.hit || h.t1 <= 0.0f) return s;
        s.t0 = std::max(h.t0, 0.0f);
        s.t1 = h.t1;
    }
    const float g = distanceToGround(a, mu);
    if (g >= 0.0f) s.t1 = std::min(s.t1, g);
    return s;
}

inline double heightAt(double h0, double mu, double t) {
    const double r0 = kPlanetRadius + h0;
    const double r  = std::sqrt(r0 * r0 + t * t + 2.0 * r0 * t * mu);
    return std::max(r - kPlanetRadius, 0.0);
}

struct OpticalDepth {
    double rayleigh;
    double mie;
};

inline OpticalDepth opticalDepth(double h0, double mu, double len, int steps) {
    if (len <= 0.0 || steps < 1) return { 0.0, 0.0 };
    const double dt = len / steps;
    double sumR = 0.0, sumM = 0.0;
    for (int i = 0; i <= steps; ++i) {
        const double h = heightAt(h0, mu, i * dt);
        const double w = (i == 0 || i == steps) ? 0.5 : 1.0;
        sumR += w * std::exp(-h / kRayleighScaleHeight);
        sumM += w * std::exp(-h / kMieScaleHeight);
    }
    return { sumR * dt, sumM * dt };
}

inline Vec3 transmittanceFrom(const OpticalDepth& od) {
    const double m = kBetaMieExtinct * od.mie;
    return { float(std::exp(-(kBetaRayleighR * od.rayleigh + m))),
             float(std::exp(-(kBetaRayleighG * od.rayleigh + m))),
             float(std::exp(-(kBetaRayleighB * od.rayleigh + m))) };
}

inline Vec3 transmittanceToTop(double h0, double mu, int steps = kTransmittanceSteps) {
    const double a   = h0 / kPlanetRadius;
    const double len = double(distanceToTop(float(a), float(mu))) * kPlanetRadius;
    return transmittanceFrom(opticalDepth(h0, mu, len, steps));
}

inline float rayleighPhase(float cosTheta) {
    return (3.0f / (16.0f * space::kPi)) * (1.0f + cosTheta * cosTheta);
}

inline float miePhase(float cosTheta, float g = kMieG) {
    const float g2 = g * g;
    const float d  = 1.0f + g2 - 2.0f * g * cosTheta;
    return (1.0f - g2) / (4.0f * space::kPi * d * std::sqrt(std::max(d, 1e-8f)));
}

inline float acesTonemap(float x) {
    const float a = 2.51f, b = 0.03f, c = 2.43f, d = 0.59f, e = 0.14f;
    const float y = (x * (a * x + b)) / (x * (c * x + d) + e);
    return std::min(std::max(y, 0.0f), 1.0f);
}

inline Vec3 acesTonemap(const Vec3& v) {
    return { acesTonemap(v.x), acesTonemap(v.y), acesTonemap(v.z) };
}

struct TransmittanceLut {
    int                width  = 0;
    int                height = 0;
    std::vector<float> rgba;

    bool valid() const {
        return width > 0 && height > 0 &&
               rgba.size() == size_t(width) * size_t(height) * 4;
    }
    size_t byteSize() const { return rgba.size() * sizeof(float); }
};

inline float lutAltitudeFromRow(float xr) {
    const float rho = xr * kHorizonSpan;
    return rho * rho / (std::sqrt(1.0f + rho * rho) + 1.0f);
}
inline float lutRowFromAltitude(float a) {
    return std::sqrt(std::max(a * (a + 2.0f), 0.0f)) / kHorizonSpan;
}

inline void lutPathRange(float a, float& dMin, float& dMax) {
    dMin = kTopRadii - a;
    dMax = std::sqrt(std::max(a * (a + 2.0f), 0.0f)) + kHorizonSpan;
}

inline float lutMuFromCol(float a, float xc) {
    float dMin, dMax;
    lutPathRange(a, dMin, dMax);
    const float d = dMin + xc * (dMax - dMin);
    if (d <= 1e-9f) return 1.0f;

    const float c  = (a - kTopRadii) * (a + kTopRadii + 2.0f);
    const float mu = -(d * d + c) / (2.0f * (1.0f + a) * d);
    return std::min(std::max(mu, -1.0f), 1.0f);
}

inline float lutColFromMu(float a, float mu) {
    float dMin, dMax;
    lutPathRange(a, dMin, dMax);
    const float span = dMax - dMin;
    if (span <= 1e-9f) return 0.0f;
    return std::min(std::max((distanceToTop(a, mu) - dMin) / span, 0.0f), 1.0f);
}

inline TransmittanceLut buildTransmittanceLut(int width  = kLutWidth,
                                              int height = kLutHeight,
                                              int steps  = kTransmittanceSteps) {
    TransmittanceLut lut;
    lut.width  = width;
    lut.height = height;
    lut.rgba.resize(size_t(width) * size_t(height) * 4);

    for (int y = 0; y < height; ++y) {
        const float  xr = float(y) / float(height - 1);
        const float  a  = lutAltitudeFromRow(xr);
        const double h  = double(a) * kPlanetRadius;
        for (int x = 0; x < width; ++x) {
            const float xc = float(x) / float(width - 1);
            const Vec3  T  = transmittanceToTop(h, lutMuFromCol(a, xc), steps);
            float* px = &lut.rgba[(size_t(y) * size_t(width) + size_t(x)) * 4];
            px[0] = T.x; px[1] = T.y; px[2] = T.z; px[3] = 1.0f;
        }
    }
    return lut;
}

inline float lutTexCoord(float x01, int n) {
    return (0.5f + x01 * float(n - 1)) / float(n);
}

inline Vec3 fetchLut(const TransmittanceLut& lut, float fx, float fy) {
    if (!lut.valid()) return { 1.0f, 1.0f, 1.0f };
    fx = std::min(std::max(fx, 0.0f), float(lut.width  - 1));
    fy = std::min(std::max(fy, 0.0f), float(lut.height - 1));
    const int   x0 = int(fx), y0 = int(fy);
    const int   x1 = std::min(x0 + 1, lut.width  - 1);
    const int   y1 = std::min(y0 + 1, lut.height - 1);
    const float tx = fx - float(x0), ty = fy - float(y0);

    const int   xs[2] = { x0, x1 }, ys[2] = { y0, y1 };
    const float ws[2][2] = { { (1.0f - tx) * (1.0f - ty), tx * (1.0f - ty) },
                             { (1.0f - tx) * ty,          tx * ty          } };
    Vec3 out{ 0.0f, 0.0f, 0.0f };
    for (int j = 0; j < 2; ++j)
        for (int i = 0; i < 2; ++i) {
            const float* px =
                &lut.rgba[(size_t(ys[j]) * size_t(lut.width) + size_t(xs[i])) * 4];
            out = out + Vec3{ px[0], px[1], px[2] } * ws[j][i];
        }
    return out;
}

inline Vec3 sampleLutUV(const TransmittanceLut& lut, float u, float v) {
    return fetchLut(lut, u * lut.width - 0.5f, v * lut.height - 0.5f);
}

inline Vec3 sampleLut(const TransmittanceLut& lut, float a, float mu) {
    if (!lut.valid()) return { 1.0f, 1.0f, 1.0f };
    return fetchLut(lut, lutColFromMu(a, mu) * float(lut.width  - 1),
                         lutRowFromAltitude(a) * float(lut.height - 1));
}

inline Vec3 skyRadiance(const Vec3& dir, const Vec3& up, float altM,
                        const Vec3& sunToward, const TransmittanceLut& lut,
                        float tEndM = 1e9f, int steps = kSkySteps,
                        Vec3* viewTransmittance = nullptr) {
    if (viewTransmittance) *viewTransmittance = { 1.0f, 1.0f, 1.0f };
    const float R  = float(kPlanetRadius);
    const float a  = altM / R;
    const float mu = dot(up, dir);

    const MarchSpan span = marchSpan(a, mu);
    if (span.empty()) return { 0.0f, 0.0f, 0.0f };
    const float t0 = span.t0;
    const float t1 = std::min(span.t1, tEndM / R);
    if (t1 <= t0) return { 0.0f, 0.0f, 0.0f };

    const float cosT = dot(dir, sunToward);
    const float phR  = rayleighPhase(cosT);
    const float phM  = miePhase(cosT);

    const float ds  = (t1 - t0) / float(steps);
    const float dsM = ds * R;

    double odR = 0.0, odM = 0.0;
    Vec3   sumR{ 0.0f, 0.0f, 0.0f }, sumM{ 0.0f, 0.0f, 0.0f };

    for (int i = 0; i < steps; ++i) {
        const float t = t0 + (float(i) + 0.5f) * ds;

        const float q  = a * (a + 2.0f) + t * (t + 2.0f * (1.0f + a) * mu);
        const float alt = q / (std::sqrt(1.0f + q) + 1.0f);
        const double hM = double(alt) * kPlanetRadius;
        const double dR = std::exp(-hM / kRayleighScaleHeight);
        const double dM = std::exp(-hM / kMieScaleHeight);

        odR += 0.5 * dR * dsM;  odM += 0.5 * dM * dsM;
        const Vec3 tView = transmittanceFrom({ odR, odM });
        odR += 0.5 * dR * dsM;  odM += 0.5 * dM * dsM;

        const Vec3  pos = up * (1.0f + a) + dir * t;
        const float rr  = length(pos);
        const float muS = dot(pos, sunToward) / rr;
        const Vec3  tSun = hitsGround(alt, muS) ? Vec3{ 0.0f, 0.0f, 0.0f }
                                               : sampleLut(lut, alt, muS);

        const Vec3 w = mulc(tView, tSun);
        sumR = sumR + w * float(dR * dsM);
        sumM = sumM + w * float(dM * dsM);
    }

    if (viewTransmittance) *viewTransmittance = transmittanceFrom({ odR, odM });

    const Vec3 rayleigh = mulc(kBetaRayleigh, sumR * phR);
    const Vec3 mie      = sumM * (float(kBetaMieScatter) * phM);
    return (rayleigh + mie) * kSunIrradiance;
}

inline float smoothstepf(float e0, float e1, float x) {
    const float t = std::min(std::max((x - e0) / (e1 - e0), 0.0f), 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

inline Vec3 sunDiscRadiance(const Vec3& dir, const Vec3& up, float altM,
                            const Vec3& sunToward, const TransmittanceLut& lut,
                            float pixelAngle) {
    const float cosR = std::cos(kSunAngularRadius);

    const float w    = std::max(pixelAngle * std::sin(kSunAngularRadius), 1e-9f);
    const float edge = smoothstepf(cosR - w, cosR + w, dot(dir, sunToward));
    if (edge <= 0.0f) return { 0.0f, 0.0f, 0.0f };

    const float a  = altM / float(kPlanetRadius);
    const float mu = dot(up, dir);
    if (hitsGround(a, mu)) return { 0.0f, 0.0f, 0.0f };

    return sampleLut(lut, a, mu) * (kSunDiscRadiance * edge);
}

}
}
