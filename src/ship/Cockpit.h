#pragma once

#include "math/SpaceMath.h"
#include "math/Quat.h"
#include "ship/ShipConfig.h"

#include <cmath>
#include <algorithm>

namespace planet {

using space::Vec3;

enum class SwitchKind { Toggle, Momentary };

struct CockpitSwitch {
    const char* name;
    Vec3        centre;
    SwitchKind  kind;
};

enum SwitchIndex { kSwGear = 0, kSwAssist = 1, kSwCutThrottle = 2, kSwJump = 3,
                   kSwitchCount = 4 };

inline const CockpitSwitch kSwitches[kSwitchCount] = {
    { "gear",     Vec3{ -0.30f, kConsoleTopY + kSwitchHalf.y, 6.45f }, SwitchKind::Toggle    },
    { "assist",   Vec3{  0.00f, kConsoleTopY + kSwitchHalf.y, 6.45f }, SwitchKind::Toggle    },
    { "throttle", Vec3{  0.30f, kConsoleTopY + kSwitchHalf.y, 6.45f }, SwitchKind::Momentary },
    { "jump",     Vec3{ -0.60f, kConsoleTopY + kSwitchHalf.y, 6.45f }, SwitchKind::Momentary },
};

inline const Vec3 kLampCentre{ 0.60f, kConsoleTopY + 0.010f, 6.45f };
inline const Vec3 kLampHalf  { 0.055f, 0.010f, 0.030f };

enum class LampState { Cold = 0, Ready = 1, Busy = 2 };

struct ShipToggles {
    bool  gearDown = false;
    bool  assistOn = true;
    float throttle = 0.0f;

    bool  jumpRequested = false;
};

inline bool switchIsOn(const ShipToggles& t, int i) {
    switch (i) {
    case kSwGear:   return t.gearDown;
    case kSwAssist: return t.assistOn;
    default:        return false;
    }
}

inline void activateSwitch(ShipToggles& t, int i) {
    switch (i) {
    case kSwGear:         t.gearDown = !t.gearDown; break;
    case kSwAssist:       t.assistOn = !t.assistOn; break;
    case kSwCutThrottle:  t.throttle = 0.0f;        break;
    case kSwJump:         t.jumpRequested = true;   break;
    default: break;
    }
}

struct Pick {
    int   index    = -1;
    float distance = 0.0f;
    bool  hit() const { return index >= 0; }
};

inline bool rayAabb(const Vec3& o, const Vec3& d, const Vec3& lo, const Vec3& hi,
                    float& tHit) {
    const float od[3] = { o.x, o.y, o.z };
    const float dd[3] = { d.x, d.y, d.z };
    const float lod[3] = { lo.x, lo.y, lo.z };
    const float hid[3] = { hi.x, hi.y, hi.z };

    float tmin = 0.0f, tmax = 1e30f;
    for (int i = 0; i < 3; ++i) {
        if (std::fabs(dd[i]) < 1e-9f) {
            if (od[i] < lod[i] || od[i] > hid[i]) return false;
            continue;
        }
        const float inv = 1.0f / dd[i];
        float t1 = (lod[i] - od[i]) * inv;
        float t2 = (hid[i] - od[i]) * inv;
        if (t1 > t2) std::swap(t1, t2);
        tmin = std::max(tmin, t1);
        tmax = std::min(tmax, t2);
        if (tmin > tmax) return false;
    }
    tHit = tmin;
    return true;
}

inline Pick pickSwitch(const Vec3& eye, const Vec3& dir, float reach = kSwitchReach) {
    Pick best;
    for (int i = 0; i < kSwitchCount; ++i) {
        const Vec3& c = kSwitches[i].centre;
        const Vec3 lo{ c.x - kSwitchHalf.x, c.y - kSwitchHalf.y, c.z - kSwitchHalf.z };
        const Vec3 hi{ c.x + kSwitchHalf.x, c.y + kSwitchHalf.y, c.z + kSwitchHalf.z };
        float t = 0.0f;
        if (!rayAabb(eye, dir, lo, hi, t)) continue;
        if (t > reach) continue;
        if (!best.hit() || t < best.distance) { best.index = i; best.distance = t; }
    }
    return best;
}

inline space::Quat switchRotation(const ShipToggles& t, int i) {
    if (kSwitches[i].kind != SwitchKind::Toggle)
        return space::Quat{ 0.0f, 0.0f, 0.0f, 1.0f };
    const float angle = switchIsOn(t, i) ? -kSwitchTilt : kSwitchTilt;
    return space::fromAxisAngle(Vec3{ 1.0f, 0.0f, 0.0f }, angle);
}

}
