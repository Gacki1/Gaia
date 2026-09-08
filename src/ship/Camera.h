#pragma once

#include "math/SpaceMath.h"
#include "math/WorldMath.h"
#include "math/Quat.h"
#include "ship/Flight.h"
#include "ship/ShipConfig.h"
#include <cmath>
#include <algorithm>

namespace planet {

struct CameraState {
    bool  thirdPerson = false;
    float freeYaw   = 0.0f;
    float freePitch = 0.0f;
    space::Quat chaseRot{ 0, 0, 0, 1 };
    bool  chaseInit = false;
};

struct CameraView {
    space::DVec3 pos;
    space::Quat  orient;
};

inline CameraView updateCamera(CameraState& cam, const ShipState& ship,
                               float lookDx, float lookDy, bool freeLookHeld,
                               float dt) {

    const float yawMax   = cam.thirdPerson ? kFreeLookYawChase   : kFreeLookYawCockpit;
    const float pitchMax = cam.thirdPerson ? kFreeLookPitchChase : kFreeLookPitchCockpit;

    if (freeLookHeld) {

        cam.freeYaw   -= lookDx * kLookSensitivity;
        cam.freePitch -= lookDy * kLookSensitivity;
    } else {

        const float k = 1.0f - std::exp(-dt / kFreeLookReturnTau);
        cam.freeYaw   -= cam.freeYaw   * k;
        cam.freePitch -= cam.freePitch * k;
    }

    cam.freeYaw   = std::max(-yawMax,   std::min(yawMax,   cam.freeYaw));
    cam.freePitch = std::max(-pitchMax, std::min(pitchMax, cam.freePitch));

    const space::Quat lookOffset = space::fromYawPitch(cam.freeYaw, cam.freePitch);

    CameraView view;
    if (!cam.thirdPerson) {
        view.pos    = ship.pos + space::rotate(ship.orient,
                          space::DVec3{ kEyeOffset.x, kEyeOffset.y, kEyeOffset.z });
        view.orient = ship.orient * lookOffset;
        return view;
    }

    if (!cam.chaseInit) { cam.chaseRot = ship.orient; cam.chaseInit = true; }
    cam.chaseRot = space::nlerp(cam.chaseRot, ship.orient,
                                1.0f - std::exp(-dt / kChaseRotTau));

    const float lag = space::angleBetween(cam.chaseRot, ship.orient);
    if (lag > kChaseMaxLag)
        cam.chaseRot = space::nlerp(cam.chaseRot, ship.orient,
                                    1.0f - kChaseMaxLag / lag);

    const space::Quat base = cam.chaseRot * lookOffset;
    view.pos = ship.pos + space::rotate(base,
                   space::DVec3{ kChaseOffset.x, kChaseOffset.y, kChaseOffset.z });
    view.orient = base;
    return view;
}

}
