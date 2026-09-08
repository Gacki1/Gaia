#pragma once

#include "world/CubeSphere.h"
#include "ship/Flight.h"
#include "ship/ShipConfig.h"
#include "math/WorldMath.h"
#include <cmath>

namespace planet {

class TerrainGround {
public:

    explicit TerrainGround(const PlanetParams& params = PlanetParams{}) : w_(params) {}

    const World& w() const { return w_; }

    GroundHit operator()(const space::DVec3& p) const;

private:
    World w_;
};

inline GroundHit TerrainGround::operator()(const space::DVec3& p) const {
    using space::DVec3;

    GroundHit h;
    const DVec3 dir = space::normalize(p);
    const double r0 = surfaceRadiusAt(w_, dir, kPhysicsDetailOctaves);
    h.radius = r0;

    const DVec3 ref = (std::fabs(dir.y) < 0.9) ? DVec3{ 0, 1, 0 } : DVec3{ 1, 0, 0 };
    const DVec3 t1  = space::normalize(space::cross(ref, dir));
    const DVec3 t2  = space::cross(dir, t1);
    const double eps = 0.5;

    auto radAt = [&](const DVec3& q) {
        const DVec3 d = space::normalize(q);
        return surfaceRadiusAt(w_, d, kPhysicsDetailOctaves);
    };
    const double d1 = (radAt(dir * r0 + t1 * eps) - r0) / eps;
    const double d2 = (radAt(dir * r0 + t2 * eps) - r0) / eps;

    h.normal = space::normalize(dir - t1 * d1 - t2 * d2);
    return h;
}

inline double footprintSlope(const World& w, const space::DVec3& dir) {
    using space::DVec3;
    DVec3 t = space::cross(dir, DVec3{ 0.0, 1.0, 0.0 });
    if (space::length(t) < 1e-6) t = space::cross(dir, DVec3{ 1.0, 0.0, 0.0 });
    t = space::normalize(t);
    const DVec3 b = space::normalize(space::cross(dir, t));
    const double off = kLandingFootprint / kPlanetRadius;

    const double r0 = surfaceRadiusAt(w, dir, kPhysicsDetailOctaves);
    double worst = 0.0;
    for (const DVec3& axis : { t, b }) {
        const DVec3 d1 = space::normalize(dir + axis * off);
        const DVec3 d2 = space::normalize(dir - axis * off);
        const double r1 = surfaceRadiusAt(w, d1, kPhysicsDetailOctaves);
        const double r2 = surfaceRadiusAt(w, d2, kPhysicsDetailOctaves);
        worst = std::max(worst, std::fabs(r1 - r0) / kLandingFootprint);
        worst = std::max(worst, std::fabs(r2 - r0) / kLandingFootprint);
    }
    return std::atan(worst);
}

inline space::DVec3 findSunlitLand(const World& w, int samples = 8192) {
    using space::DVec3;
    const DVec3 toSun = space::normalize(DVec3{ -kSunDirection.x,
                                                -kSunDirection.y,
                                                -kSunDirection.z });

    DVec3 best{ 0.0, 1.0, 0.0 };
    double bestR = -1e9;
    const double golden = space::kPi * (3.0 - std::sqrt(5.0));
    for (int i = 0; i < samples; ++i) {
        const double y  = 1.0 - 2.0 * (i + 0.5) / samples;
        const double r  = std::sqrt(std::max(0.0, 1.0 - y * y));
        const double th = golden * i;
        const DVec3 d{ std::cos(th) * r, y, std::sin(th) * r };
        if (space::dot(d, toSun) < 0.5) continue;

        const TerrainShape sh = sampleShape(w, d);
        if (sh.landE * sh.amp > w.p.bandHighland) continue;
        const double rad = terrainRadius(w.p, sh);
        if (rad <= bestR) continue;

        if (footprintSlope(w, d) > kMaxLandingSlope) continue;
        bestR = rad; best = d;
    }
    return best;
}

inline space::Quat uprightOn(const space::DVec3& up, const space::DVec3& desiredForward) {
    const space::DVec3 u = space::normalize(up);
    space::DVec3 f = desiredForward - u * space::dot(desiredForward, u);
    if (space::length(f) < 1e-6) {
        const space::DVec3 ref = (std::fabs(u.y) < 0.9) ? space::DVec3{ 0, 1, 0 }
                                                        : space::DVec3{ 1, 0, 0 };
        f = space::cross(ref, u);
    }
    f = space::normalize(f);
    const space::Quat level = space::fromTo(space::Vec3{ 0, 1, 0 }, space::toF(u));
    const space::Vec3 levelFwd = space::rotate(level, space::Vec3{ 0, 0, 1 });
    return space::fromTo(levelFwd, space::toF(f)) * level;
}

}
