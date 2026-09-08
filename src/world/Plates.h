#pragma once

#include "math/Hash.h"
#include "math/SpaceMath.h"
#include "math/WorldMath.h"
#include "world/PlanetConfig.h"
#include "world/PlanetParams.h"
#include "world/Noise.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace planet {

using space::DVec3;

struct RangeSpec {
    bool  active = false;
    int   sign   = 1;

    float crestWidth  = 0.0f;
    float crestHeight = 0.0f;
    float sharpness   = 1.0f;
    float asymmetry   = 0.0f;
    float terrace     = 0.0f;

    int   ridges      = 1;
    float ridgeGap    = 1.5f;
};

struct PlateSystem {
    DVec3     seed[kPlateCount];
    RangeSpec range[kPlateCount * kPlateCount];

    const RangeSpec& at(int a, int b) const {
        return range[std::size_t(a) * kPlateCount + std::size_t(b)];
    }
};

struct PlateHit {
    int    a = 0, b = 1;
    double theta1 = 0.0;
    double theta2 = 0.0;
    double theta3 = 0.0;

    double toJunction() const { return 0.5 * (theta3 - theta2); }

    double toBoundary() const { return 0.5 * (theta2 - theta1); }
};

inline PlateHit plateAt(const PlateSystem& ps, const DVec3& dir) {

    double best = -2.0, second = -2.0, third = -2.0;
    int    bi = 0, si = 1;
    for (int i = 0; i < kPlateCount; ++i) {
        const double d = space::dot(dir, ps.seed[i]);
        if (d > best)        { third = second; second = best; si = bi; best = d; bi = i; }
        else if (d > second) { third = second; second = d;    si = i; }
        else if (d > third)  { third = d; }
    }
    PlateHit h;
    h.a = bi; h.b = si;
    h.theta1 = std::acos(std::max(-1.0, std::min(1.0, best)));
    h.theta2 = std::acos(std::max(-1.0, std::min(1.0, second)));
    h.theta3 = std::acos(std::max(-1.0, std::min(1.0, third)));
    return h;
}

inline bool plateIsLand(const Noise& n, const PlanetParams& p, const DVec3& dir) {
    const Noise::DualFbm f = n.fbmDual(dir, p.noiseOctaves, p.noiseFrequency,
                                       p.noiseLacunarity, p.noiseGain,
                                       p.rangeOctaves);
    return f.plain - p.seaLevelNoise >= 0.0f;
}

inline DVec3 plateSeedDir(unsigned seed, int i) {
    const double golden = space::kPi * (3.0 - std::sqrt(5.0));
    const double y  = 1.0 - 2.0 * (double(i) + 0.5) / double(kPlateCount);
    const double r  = std::sqrt(std::max(0.0, 1.0 - y * y));
    const double th = golden * double(i);
    DVec3 d{ std::cos(th) * r, y, std::sin(th) * r };

    uint32_t h = hashCombine(0x51ED270Bu ^ seed, uint32_t(i));
    const double jx = double(hashUnit(h = hashMix(h))) * 2.0 - 1.0;
    const double jy = double(hashUnit(h = hashMix(h))) * 2.0 - 1.0;
    const double jz = double(hashUnit(h = hashMix(h))) * 2.0 - 1.0;
    d = d + DVec3{ jx, jy, jz } * double(kPlateJitter);
    return space::normalize(d);
}

template <typename LandFn>
inline PlateSystem buildPlates(unsigned seed, LandFn isLand) {
    PlateSystem ps;
    for (int i = 0; i < kPlateCount; ++i) ps.seed[i] = plateSeedDir(seed, i);

    int hits[kPlateCount * kPlateCount] = {};
    int land[kPlateCount * kPlateCount] = {};
    const int    kProbes = 60000;
    const double kBand   = 0.02;
    const double golden  = space::kPi * (3.0 - std::sqrt(5.0));
    for (int i = 0; i < kProbes; ++i) {
        const double y  = 1.0 - 2.0 * (double(i) + 0.5) / double(kProbes);
        const double r  = std::sqrt(std::max(0.0, 1.0 - y * y));
        const double th = golden * double(i);
        const PlateHit h = plateAt(ps, DVec3{ std::cos(th) * r, y, std::sin(th) * r });
        if (h.toBoundary() > kBand) continue;
        const DVec3 d{ std::cos(th) * r, y, std::sin(th) * r };
        const bool dry = isLand(d);
        ++hits[std::size_t(h.a) * kPlateCount + std::size_t(h.b)];
        ++hits[std::size_t(h.b) * kPlateCount + std::size_t(h.a)];
        if (dry) {
            ++land[std::size_t(h.a) * kPlateCount + std::size_t(h.b)];
            ++land[std::size_t(h.b) * kPlateCount + std::size_t(h.a)];
        }
    }

    const double minArc    = double(kRangeMinLength) / kPlanetRadius;
    const int    minHits   = int(2.0 * kBand * minArc / (4.0 * space::kPi) *
                                 double(kProbes));

    struct Cand { int dry; uint32_t key; int a, b; };
    Cand cand[kPlateCount * kPlateCount];
    int  n = 0;
    for (int a = 0; a < kPlateCount; ++a)
        for (int b = a + 1; b < kPlateCount; ++b) {
            const std::size_t k = std::size_t(a) * kPlateCount + std::size_t(b);
            if (land[k] < minHits) continue;
            uint32_t h = hashCombine(0x2F1B3C5Du ^ seed, uint32_t(a));
            h = hashCombine(h, uint32_t(b));
            cand[n++] = { land[k], h, a, b };
        }

    std::sort(cand, cand + n, [](const Cand& x, const Cand& y) {
        return x.dry != y.dry ? x.dry > y.dry : x.key > y.key;
    });

    const int take = std::min(n, kRangeCount);
    for (int k = 0; k < take; ++k) {

        uint32_t h = cand[k].key;
        auto next = [&h]() { h = hashMix(h + 0x9E3779B9u); return hashUnit(h); };

        RangeSpec r;
        r.active      = true;

        r.sign        = next() < kRiftShare ? -1 : 1;

        r.ridges      = 1 + int(next() * float(kRangeRidgeMax));
        if (r.ridges > kRangeRidgeMax) r.ridges = kRangeRidgeMax;
        r.ridgeGap    = kRangeRidgeGapLo +
                        (kRangeRidgeGapHi - kRangeRidgeGapLo) * next();
        r.crestWidth  = kRangeCrestWidth  * (0.6f + 1.0f * next());
        r.crestHeight = kRangeCrestHeight * (0.6f + 0.9f * next());
        r.sharpness   = 1.0f + 3.0f * next();
        r.asymmetry   = 2.0f * next() - 1.0f;
        r.terrace     = next();

        ps.range[std::size_t(cand[k].a) * kPlateCount + std::size_t(cand[k].b)] = r;
        ps.range[std::size_t(cand[k].b) * kPlateCount + std::size_t(cand[k].a)] = r;
    }
    return ps;
}

struct Orogeny {
    float height  = 0.0f;
    float terrace = 0.0f;
};

inline Orogeny orogenyAt(const PlateSystem& ps, const DVec3& dir) {
    const PlateHit h = plateAt(ps, dir);
    const RangeSpec& r = ps.at(h.a, h.b);
    if (!r.active) return {};

    const double d = h.toBoundary();

    const bool lowSide = h.a < h.b;
    const float wide = 1.0f + 0.5f * (lowSide ? r.asymmetry : -r.asymmetry);
    const double w = double(r.crestWidth) * double(wide);
    if (w <= 0.0) return {};

    const double span = (double(r.ridges) - 1.0) * double(r.ridgeGap) * w + w;
    if (d >= span) return {};

    const float jf = smoothBand(0.0f, kRangeJunctionFade,
                                float(h.toJunction()));

    float height = 0.0f, terrace = 0.0f;
    for (int k = 0; k < r.ridges; ++k) {
        const double c  = double(k) * double(r.ridgeGap) * w;
        const double dk = d > c ? d - c : c - d;
        if (dk >= w) continue;
        const float prof = std::pow(float(1.0 - dk / w), r.sharpness);
        const float amp  = std::pow(kRangeRidgeFalloff, float(k));
        height  += r.crestHeight * amp * prof;
        terrace += r.terrace     * amp * prof;
    }

    Orogeny o;
    o.height  = height * jf * float(r.sign);
    o.terrace = terrace * jf;
    return o;
}

inline float plateauAt(const PlateSystem& ps, const DVec3& dir) {
    const PlateHit h = plateAt(ps, dir);
    const uint32_t k = hashMix(0x7E4A11C3u + uint32_t(h.a) * 0x9E3779B9u);
    if (hashUnit(k) >= kPlateauShare) return 0.0f;
    return smoothBand(0.0f, kPlateauEdgeFade, float(h.toBoundary()));
}

inline int activeRangeCount(const PlateSystem& ps) {
    int n = 0;
    for (int a = 0; a < kPlateCount; ++a)
        for (int b = a + 1; b < kPlateCount; ++b)
            if (ps.at(a, b).active) ++n;
    return n;
}

}
