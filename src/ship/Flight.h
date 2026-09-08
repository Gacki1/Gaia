#pragma once

#include "math/SpaceMath.h"
#include "math/WorldMath.h"
#include "math/Quat.h"
#include "ship/ShipConfig.h"
#include <cmath>
#include <algorithm>

namespace planet {

using space::DVec3;
using space::Quat;
using space::Vec3;

struct GroundHit {
    double radius = 0.0;
    DVec3  normal{ 0, 1, 0 };
};

struct ShipControls {
    float pitch = 0.0f, yaw = 0.0f, roll = 0.0f;
    float strafeX = 0.0f, strafeY = 0.0f;
    float throttle = 0.0f;
    float thrustZ = 0.0f;
    bool  boost = false;
    bool  gearDown = false;
    bool  assist = true;

    DVec3 quantumAccel{ 0.0, 0.0, 0.0 };

    bool  quantumTravel = false;
};

struct ShipState {
    DVec3 pos{ 0, 0, 0 };
    DVec3 vel{ 0, 0, 0 };
    Quat  orient{ 0, 0, 0, 1 };
    Vec3  angVel{ 0, 0, 0 };

    float gearT = 0.0f;
    float legComp[4]{};
    bool  legContact[4]{};
    bool  landed = false;
    double landedFor = 0.0;

    double accum = 0.0;
};

inline DVec3 gravityAt(const DVec3& pos) {
    const double r2 = space::dot(pos, pos);
    if (r2 < 1.0) return DVec3{ 0, 0, 0 };
    const double r = std::sqrt(r2);
    return pos * (-kMu / (r2 * r));
}

inline double altitudeASL(const ShipState& s) {
    return space::length(s.pos) - kPlanetRadius;
}

inline Vec3 clampLength(const Vec3& v, float maxLen) {
    const float len = space::length(v);
    if (len <= maxLen || len < 1e-20f) return v;
    return v * (maxLen / len);
}

inline DVec3 clampLength(const DVec3& v, double maxLen) {
    const double len = space::length(v);
    if (len <= maxLen || len < 1e-300) return v;
    return v * (maxLen / len);
}

inline Vec3  toBody(const Quat& q, const Vec3&  v) { return space::rotate(space::conjugate(q), v); }
inline DVec3 toBody(const Quat& q, const DVec3& v) { return space::rotate(space::conjugate(q), v); }

template <class GroundFn>
void substep(ShipState& s, const ShipControls& in, GroundFn&& ground, double dt) {
    const DVec3 gravity = gravityAt(s.pos);

    DVec3 contactForce{ 0, 0, 0 };
    Vec3  contactTorque{ 0, 0, 0 };
    const bool gearActive = s.gearT > 0.98f;
    int  contacts = 0;

    const DVec3 angVelWorld =
        space::rotate(s.orient, DVec3{ s.angVel.x, s.angVel.y, s.angVel.z });

    for (int i = 0; i < 4; ++i) {
        s.legContact[i] = false;
        s.legComp[i]    = 0.0f;
        if (!gearActive) continue;

        const DVec3 footLocal{ kGearPivot[i].x,
                               kGearPivot[i].y - kGearLegLength,
                               kGearPivot[i].z };
        const DVec3 r    = space::rotate(s.orient, footLocal);
        const DVec3 foot = s.pos + r;

        const GroundHit g   = ground(foot);
        const double    pen = g.radius - space::length(foot);
        if (pen <= 0.0) continue;
        const double comp = std::min(pen, static_cast<double>(kGearTravel));

        const DVec3  vP = s.vel + space::cross(angVelWorld, r);
        const double vN = space::dot(vP, g.normal);

        const double fN = std::max(kGearK * comp - kGearC * vN, 0.0);

        const DVec3  vT  = vP - g.normal * vN;
        const double vTm = space::length(vT);
        DVec3 fT{ 0, 0, 0 };
        if (vTm > 1e-6) {
            const double fArrest = (kShipMass * 0.25) * vTm / dt;
            fT = vT * (-std::min(kGearFriction * fN, fArrest) / vTm);
        }

        const DVec3 F  = g.normal * fN + fT;
        const DVec3 tB = toBody(s.orient, space::cross(r, F));

        contactForce  = contactForce + F;
        contactTorque = contactTorque + Vec3{ static_cast<float>(tB.x),
                                              static_cast<float>(tB.y),
                                              static_cast<float>(tB.z) };
        s.legContact[i] = true;
        s.legComp[i]    = static_cast<float>(comp);
        ++contacts;
    }

    const float rateScale = in.boost ? kNavRateScale : 1.0f;
    const Vec3  rateMax{ kRateMaxPitch * rateScale,
                         kRateMaxYaw   * rateScale,
                         kRateMaxRoll  * rateScale };
    const Vec3  angAccelMax{ kAngAccelPitch, kAngAccelYaw, kAngAccelRoll };

    Vec3 angAccel;
    if (in.assist) {
        const Vec3 rateCmd{ in.pitch * rateMax.x, in.yaw * rateMax.y, in.roll * rateMax.z };
        angAccel = Vec3{ (rateCmd.x - s.angVel.x) / kTauAng,
                         (rateCmd.y - s.angVel.y) / kTauAng,
                         (rateCmd.z - s.angVel.z) / kTauAng };
    } else {
        angAccel = Vec3{ in.pitch * angAccelMax.x,
                         in.yaw   * angAccelMax.y,
                         in.roll  * angAccelMax.z };
    }

    angAccel = Vec3{ std::max(-angAccelMax.x, std::min(angAccelMax.x, angAccel.x)),
                     std::max(-angAccelMax.y, std::min(angAccelMax.y, angAccel.y)),
                     std::max(-angAccelMax.z, std::min(angAccelMax.z, angAccel.z)) };

    const double speedCap = in.boost ? kNavSpeed : kScmSpeed;
    const DVec3  velBody  = toBody(s.orient, s.vel);

    DVec3 accelBody;
    if (in.assist) {

        const DVec3 velCmd{ in.strafeX * 0.5 * speedCap,
                            in.strafeY * 0.5 * speedCap,
                            in.throttle * speedCap };
        accelBody = (velCmd - velBody) * (1.0 / kTauLin);

        bool onGround = false;
        for (int i = 0; i < 4; ++i) onGround = onGround || s.legContact[i];
        if (!onGround) accelBody = accelBody - toBody(s.orient, gravity);
    } else {
        accelBody = DVec3{ in.strafeX * kAccelLateral,
                           in.strafeY * kAccelVert,
                           in.thrustZ * (in.thrustZ >= 0.0f ? kAccelMain : kAccelRetro) };
    }

    if (in.quantumTravel) accelBody = DVec3{ 0.0, 0.0, 0.0 };

    accelBody.x = std::max(-kAccelLateral, std::min(kAccelLateral, accelBody.x));
    accelBody.y = std::max(-kAccelVert,    std::min(kAccelVert,    accelBody.y));
    accelBody.z = std::max(-kAccelRetro,   std::min(kAccelMain,    accelBody.z));

    const DVec3 accel = gravity + space::rotate(s.orient, accelBody) +
                        contactForce * (1.0 / kShipMass) + in.quantumAccel;

    s.vel = s.vel + accel * dt;

    if (in.assist && !in.quantumTravel) {
        const double sp = space::length(s.vel);
        if (sp > speedCap) s.vel = s.vel * (speedCap / sp);
    }

    s.pos = s.pos + s.vel * dt;

    Vec3 L{ static_cast<float>(s.angVel.x * kInertiaPitch),
            static_cast<float>(s.angVel.y * kInertiaYaw),
            static_cast<float>(s.angVel.z * kInertiaRoll) };

    const float spin = space::length(s.angVel);
    if (spin > 1e-9f)
        L = space::rotate(space::fromAxisAngle(s.angVel, -spin * static_cast<float>(dt)), L);

    L = L + Vec3{ static_cast<float>(angAccel.x * kInertiaPitch * dt),
                  static_cast<float>(angAccel.y * kInertiaYaw   * dt),
                  static_cast<float>(angAccel.z * kInertiaRoll  * dt) };

    s.angVel = Vec3{ static_cast<float>(L.x / kInertiaPitch),
                     static_cast<float>(L.y / kInertiaYaw),
                     static_cast<float>(L.z / kInertiaRoll) };

    if (in.assist) {
        s.angVel.x = std::max(-rateMax.x, std::min(rateMax.x, s.angVel.x));
        s.angVel.y = std::max(-rateMax.y, std::min(rateMax.y, s.angVel.y));
        s.angVel.z = std::max(-rateMax.z, std::min(rateMax.z, s.angVel.z));
    }

    s.angVel = s.angVel + Vec3{
        static_cast<float>(contactTorque.x / kInertiaPitch * dt),
        static_cast<float>(contactTorque.y / kInertiaYaw   * dt),
        static_cast<float>(contactTorque.z / kInertiaRoll  * dt) };

    s.orient = space::integrate(s.orient, s.angVel, static_cast<float>(dt));

    const bool allDown  = gearActive && contacts == 4;
    const bool touching = allDown &&
                          space::length(s.vel)    < kLandedSpeed &&
                          space::length(s.angVel) < kLandedSpin;
    s.landedFor = touching ? s.landedFor + dt : 0.0;
    s.landed    = s.landedFor >= kLandedHold;

    const bool piloting = in.pitch != 0.0f || in.yaw != 0.0f || in.roll != 0.0f ||
                          in.strafeX != 0.0f || in.strafeY != 0.0f ||
                          in.thrustZ != 0.0f || in.throttle != 0.0f;
    if (touching && !piloting) {

        const double decay = std::exp(-kLandedDamping * dt);
        s.vel    = s.vel * decay;
        s.angVel = s.angVel * static_cast<float>(decay);

        if (s.landed) {
            s.vel    = DVec3{ 0, 0, 0 };
            s.angVel = Vec3{ 0, 0, 0 };
        }
    }

    const float gearRate = static_cast<float>(dt) / kGearDeployTime;
    s.gearT = in.gearDown ? std::min(1.0f, s.gearT + gearRate)
                          : std::max(0.0f, s.gearT - gearRate);
}

template <class GroundFn>
int step(ShipState& s, const ShipControls& in, GroundFn&& ground, double frameDt) {
    s.accum += frameDt;
    int steps = 0;
    while (s.accum >= kFixedDt && steps < kMaxSubsteps) {
        substep(s, in, ground, kFixedDt);
        s.accum -= kFixedDt;
        ++steps;
    }

    if (steps == kMaxSubsteps) s.accum = 0.0;
    return steps;
}

inline GroundHit flatGround(const DVec3& p, double radius) {
    GroundHit h;
    h.radius = radius;
    h.normal = space::normalize(p);
    return h;
}

inline GroundHit planeGround(const DVec3& p, const DVec3& onPlane, const DVec3& normal) {
    GroundHit h;
    h.normal = space::normalize(normal);
    const DVec3  dir = space::normalize(p);
    const double den = space::dot(dir, h.normal);
    h.radius = (std::fabs(den) < 1e-12) ? space::length(p)
                                        : space::dot(onPlane, h.normal) / den;
    return h;
}

inline DVec3 shipUp(const ShipState& s) {
    return space::rotate(s.orient, DVec3{ 0, 1, 0 });
}

}
