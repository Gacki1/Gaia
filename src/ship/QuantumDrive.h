#pragma once

#include "ship/Flight.h"
#include "world/Atmosphere.h"
#include "ship/ShipConfig.h"
#include "math/WorldMath.h"

#include <cmath>
#include <algorithm>

namespace planet {

using space::DVec3;

struct Destination {
    DVec3       pos;
    const char* name;
};

template <typename GroundFn>
inline void buildDestinations(Destination* out, GroundFn&& ground) {
    static const char* kNames[kDestinationCount] = {
        "alpha", "beta", "gamma", "delta", "epsilon", "zeta"
    };
    const double golden = space::kPi * (3.0 - std::sqrt(5.0));

    auto landAt = [&](const DVec3& d) {
        return ground(d * kPlanetRadius).radius > kPlanetRadius + kSiteMinRelief;
    };

    for (int i = 0; i < kDestinationCount; ++i) {
        const double y  = 1.0 - 2.0 * (i + 0.5) / kDestinationCount;
        const double r  = std::sqrt(std::max(0.0, 1.0 - y * y));
        const double th = golden * i;
        const DVec3 seed{ std::cos(th) * r, y, std::sin(th) * r };

        DVec3 dir = seed;
        if (!landAt(seed)) {

            const DVec3 ref = (std::fabs(seed.y) < 0.9) ? DVec3{ 0, 1, 0 }
                                                        : DVec3{ 1, 0, 0 };
            const DVec3 t1 = space::normalize(space::cross(ref, seed));
            const DVec3 t2 = space::cross(seed, t1);
            for (int k = 1; k <= kSiteSearchSteps; ++k) {

                const double frac = std::sqrt(double(k) / kSiteSearchSteps);
                const double rad   = frac * kSiteSearchRadius;
                const double ang   = golden * k;
                const DVec3 cand = space::normalize(
                    seed + t1 * (std::cos(ang) * rad) + t2 * (std::sin(ang) * rad));
                if (landAt(cand)) { dir = cand; break; }
            }
        }
        out[i].pos  = dir * (ground(dir * kPlanetRadius).radius + kArrivalStandoff);
        out[i].name = kNames[i < kDestinationCount ? i : 0];
    }
}

inline bool routeIsClear(const DVec3& from, const DVec3& to) {
    const double fromR = space::length(from);
    if (fromR < kPlanetRadius) return false;

    const DVec3 delta = to - from;
    const double dist = space::length(delta);
    if (dist < 1.0) return true;
    const DVec3 dir = delta * (1.0 / dist);

    const float a  = float((fromR - kPlanetRadius) / kPlanetRadius);
    const float mu = float(space::dot(from * (1.0 / fromR), dir));
    const float hit = atmo::distanceToGround(a, mu);
    if (hit < 0.0f) return true;

    return double(hit) * kPlanetRadius >= dist;
}

inline float alignmentError(const ShipState& s, const DVec3& to) {
    const DVec3 delta = to - s.pos;
    const double d = space::length(delta);
    if (d < 1e-6) return 0.0f;
    const DVec3 want = delta * (1.0 / d);
    const Vec3  nose = space::rotate(s.orient, Vec3{ 0.0f, 0.0f, 1.0f });
    const double c = space::dot(want, space::toD(nose));
    return float(std::acos(std::max(-1.0, std::min(1.0, c))));
}

inline double transferRadiusFor(const DVec3& from, const DVec3& to) {
    const double rf = space::length(from), rt = space::length(to);
    if (rf < 1.0 || rt < 1.0) return kPlanetRadius + kTransferAltitude;
    const double c = std::max(-1.0, std::min(1.0,
                        space::dot(from * (1.0 / rf), to * (1.0 / rt))));
    const double half = 0.5 * std::acos(c);
    const double need = kPlanetRadius / std::max(std::cos(half), 1e-6);
    return std::min(std::max(need * kTransferMargin,
                             kPlanetRadius + kMinTransferAltitude),
                    kPlanetRadius + kTransferAltitude);
}

inline DVec3 transferPointOver(const DVec3& p, double radius) {
    const double r = space::length(p);
    if (r < 1.0) return DVec3{ 0.0, radius, 0.0 };
    return p * (radius / r);
}

enum class DriveState { Idle, Spooling, Ascending, Crossing, Descending, Arrived };

struct QuantumDrive {
    DriveState state  = DriveState::Idle;
    int        target = 0;
    double     spool  = 0.0;

    DVec3 wpUp{ 0, 0, 0 };
    DVec3 wpOver{ 0, 0, 0 };
    DVec3 wpDest{ 0, 0, 0 };

    DVec3  axis{ 0, 0, 0 };
    bool   braking = false;

    bool busy() const {
        return state != DriveState::Idle && state != DriveState::Arrived;
    }

    DVec3 legTarget() const {
        switch (state) {
        case DriveState::Ascending:  return wpUp;
        case DriveState::Crossing:   return wpOver;
        case DriveState::Descending: return wpDest;
        default:                     return wpDest;
        }
    }
};

enum class EngageResult { Ok, Misaligned, Blocked, TooClose, AlreadyBusy };

inline EngageResult engage(QuantumDrive& d, const ShipState& s, const DVec3& to) {
    if (d.busy()) return EngageResult::AlreadyBusy;

    const double dist = space::length(to - s.pos);
    if (dist < 2.0 * kArrivalStandoff) return EngageResult::TooClose;

    if (alignmentError(s, to) > kAlignTolerance) return EngageResult::Misaligned;

    const double rt = transferRadiusFor(s.pos, to);
    d.wpDest = to;
    d.wpUp   = transferPointOver(s.pos, rt);
    d.wpOver = transferPointOver(to,    rt);

    if (!routeIsClear(s.pos,  d.wpUp)   ||
        !routeIsClear(d.wpUp, d.wpOver) ||
        !routeIsClear(d.wpOver, d.wpDest)) return EngageResult::Blocked;

    d.state   = DriveState::Spooling;
    d.spool   = 0.0;
    d.braking = false;
    return EngageResult::Ok;
}

inline void abortDrive(QuantumDrive& d) {
    if (d.state == DriveState::Spooling) d.state = DriveState::Idle;
}

namespace detail {

inline bool beginLeg(QuantumDrive& d, const DVec3& from, const DVec3& to) {
    const DVec3 delta = to - from;
    const double dist = space::length(delta);
    if (dist < 1000.0) return false;
    d.axis    = delta * (1.0 / dist);
    d.braking = false;
    return true;
}

inline void advanceLeg(QuantumDrive& d, const ShipState& s) {
    for (;;) {
        switch (d.state) {
        case DriveState::Ascending: d.state = DriveState::Crossing;   break;
        case DriveState::Crossing:  d.state = DriveState::Descending; break;
        default:                    d.state = DriveState::Arrived;    return;
        }
        if (beginLeg(d, s.pos, d.legTarget())) return;
    }
}
}

inline DVec3 update(QuantumDrive& d, const ShipState& s, double dt) {
    const DVec3 zero{ 0.0, 0.0, 0.0 };

    if (d.state == DriveState::Spooling) {
        d.spool += dt;
        if (d.spool < kSpoolTime) return zero;

        d.state = DriveState::Ascending;
        if (!detail::beginLeg(d, s.pos, d.wpUp)) detail::advanceLeg(d, s);
        return zero;
    }

    if (d.state != DriveState::Ascending && d.state != DriveState::Crossing &&
        d.state != DriveState::Descending)
        return zero;

    const DVec3  to        = d.legTarget();
    const double remaining = space::dot(to - s.pos, d.axis);

    const double along = space::dot(s.vel, d.axis);

    if (remaining <= 0.0 || (d.braking && along < 1.0)) {
        detail::advanceLeg(d, s);
        return zero;
    }

    const double need = (remaining > 1e-6) ? (along * along) / (2.0 * remaining)
                                           : kQuantumAccel;

    if (!d.braking && need >= kQuantumBrakeMargin * kQuantumAccel) d.braking = true;

    if (d.braking) return d.axis * (-std::min(need, kQuantumAccel));
    return (along < kQuantumSpeed) ? d.axis * kQuantumAccel : zero;
}

}
