#pragma once

#include "math/Hash.h"
#include "math/SpaceMath.h"
#include "math/WorldMath.h"
#include "world/PlanetConfig.h"
#include "world/CubeSphere.h"
#include "world/Noise.h"
#include "math/Quat.h"

#include <vector>
#include <cstdint>
#include <cmath>

namespace planet {

struct ScatterInstance {
    DVec3       pos;
    space::Quat orient;
    float       scale;
};

inline uint32_t scatterHash(int face, int ix, int iy, uint32_t salt) {
    uint32_t h = 0x9E3779B9u ^ salt;
    h = hashMix(h + static_cast<uint32_t>(face) * 0x9E3779B9u);
    h = hashMix(h + static_cast<uint32_t>(ix)   * 0x85EBCA6Bu);
    h = hashMix(h + static_cast<uint32_t>(iy)   * 0xC2B2AE35u);
    return h;
}

inline float scatterUnit(int face, int ix, int iy, uint32_t salt) {
    return hashUnit(scatterHash(face, ix, iy, salt));
}

inline float scatterDensity(const PlanetParams& p, float landE, float slope,
                            float moisture) {
    if (landE <= p.scatterMinElevation) return 0.0f;
    if (slope > p.scatterMaxSlope)      return 0.0f;

    const float dryness = 1.0f - moisture;
    const float rubble  = 0.55f + 0.45f * smoothBand(0.02f, p.scatterMaxSlope * 0.7f,
                                                     slope);
    return p.scatterDensity * rubble * (0.35f + 0.65f * dryness);
}

inline int scatterFaceOf(const DVec3& dir, double& u, double& v) {
    const double ax = std::fabs(dir.x), ay = std::fabs(dir.y), az = std::fabs(dir.z);
    int face;
    if (ax >= ay && ax >= az)      face = dir.x > 0 ? 0 : 1;
    else if (ay >= az)             face = dir.y > 0 ? 2 : 3;
    else                           face = dir.z > 0 ? 4 : 5;

    const Face& f = cubeFace(face);
    const double fwd = space::dot(dir, DVec3{ f.forward.x, f.forward.y, f.forward.z });
    const DVec3 q = dir * (1.0 / fwd);
    u = space::dot(q, DVec3{ f.right.x, f.right.y, f.right.z });
    v = space::dot(q, DVec3{ f.up.x,    f.up.y,    f.up.z    });
    return face;
}

inline std::vector<ScatterInstance> collectScatter(const World& w,
                                                   const DVec3& centre,
                                                   double range,
                                                   int detailOctaves,
                                                   int* cappedOut = nullptr) {
    std::vector<ScatterInstance> out;
    if (cappedOut) *cappedOut = 0;
    const double len = space::length(centre);
    if (len < 1.0) return out;
    const DVec3 dir = centre * (1.0 / len);

    double u0 = 0.0, v0 = 0.0;
    const int face = scatterFaceOf(dir, u0, v0);

    const double cell = 2.0 / static_cast<double>(1 << kScatterLevel);

    const double mPerU = space::length(faceToSphereD(face, u0 + cell, v0) * kPlanetRadius -
                                       faceToSphereD(face, u0,        v0) * kPlanetRadius) / cell;
    if (!(mPerU > 1.0)) return out;

    const int    reach = static_cast<int>(std::ceil(range / (mPerU * cell)));
    const int    ic    = static_cast<int>(std::floor((u0 + 1.0) / cell));
    const int    jc    = static_cast<int>(std::floor((v0 + 1.0) / cell));
    const int    last  = (1 << kScatterLevel) - 1;
    const double r2    = range * range;

    for (int j = jc - reach; j <= jc + reach; ++j) {
        if (j < 0 || j > last) continue;
        for (int i = ic - reach; i <= ic + reach; ++i) {
            if (i < 0 || i > last) continue;

            const float keep = scatterUnit(face, i, j, 0x1u);
            if (keep >= w.p.scatterDensity) continue;

            const double ju = 0.15 + 0.70 * scatterUnit(face, i, j, 0x2u);
            const double jv = 0.15 + 0.70 * scatterUnit(face, i, j, 0x3u);
            const double uu = -1.0 + cell * (i + ju);
            const double vv = -1.0 + cell * (j + jv);
            const DVec3  d  = faceToSphereD(face, uu, vv);

            {
                const DVec3 approx = d * kPlanetRadius - centre;
                const double slack = range + w.p.terrainAmplitude + w.p.detailAmplitude;
                if (space::dot(approx, approx) > slack * slack) continue;
            }

            const TerrainShape sh = sampleShape(w, d);
            const float landE = sh.landE;
            if (landE <= w.p.scatterMinElevation) continue;

            const double r = surfaceRadius(w, d, sh, detailOctaves);
            const DVec3  p = d * r;
            const DVec3  delta = p - centre;
            if (space::dot(delta, delta) > r2) continue;

            const DVec3 ref = (std::fabs(d.y) < 0.9) ? DVec3{ 0, 1, 0 }
                                                     : DVec3{ 1, 0, 0 };
            const DVec3 t1 = space::normalize(space::cross(ref, d));
            const DVec3 t2 = space::cross(d, t1);
            const double eps = 2.0;
            auto radAt = [&](const DVec3& q) {
                const DVec3 qn = space::normalize(q);
                return surfaceRadiusAt(w, qn, detailOctaves);
            };
            const double g1 = (radAt(p + t1 * eps) - r) / eps;
            const double g2 = (radAt(p + t2 * eps) - r) / eps;
            const DVec3  n  = space::normalize(d - t1 * g1 - t2 * g2);
            const float  slope = 1.0f - static_cast<float>(space::dot(n, d));

            if (keep >= scatterDensity(w.p, landE, slope, moistureAt(w, d))) continue;

            if (static_cast<int>(out.size()) >= kScatterMaxInstances) {
                if (cappedOut) ++*cappedOut;
                continue;
            }

            const float spin = scatterUnit(face, i, j, 0x4u) * 6.2831853f;
            const space::Quat up = space::fromTo(Vec3{ 0.0f, 1.0f, 0.0f },
                                                 space::toF(n));
            const space::Quat yaw = space::fromAxisAngle(Vec3{ 0.0f, 1.0f, 0.0f }, spin);

            ScatterInstance s;
            s.pos    = p;
            s.orient = up * yaw;
            s.scale  = w.p.scatterMinScale + (w.p.scatterMaxScale - w.p.scatterMinScale) *
                       scatterUnit(face, i, j, 0x5u);
            out.push_back(s);
        }
    }
    return out;
}

}
