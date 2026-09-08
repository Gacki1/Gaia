#pragma once

#include "math/SpaceMath.h"
#include "math/WorldMath.h"
#include "world/PlanetConfig.h"
#include "ship/ShipConfig.h"
#include "ship/QuantumDrive.h"
#include "math/Quat.h"

#include <cmath>

namespace planet {

struct Outpost {
    DVec3       pos;
    space::Quat orient;
    const char* name;
};

template <typename GroundFn>
inline void buildOutposts(Outpost* out, GroundFn&& ground) {
    Destination dest[kDestinationCount];
    buildDestinations(dest, ground);
    for (int i = 0; i < kDestinationCount; ++i) {
        const DVec3 dir = space::normalize(dest[i].pos);
        out[i].pos    = dir * (space::length(dest[i].pos) - kArrivalStandoff);
        out[i].orient = space::fromTo(Vec3{ 0.0f, 1.0f, 0.0f }, space::toF(dir));
        out[i].name   = dest[i].name;
    }
}

}
