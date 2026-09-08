#pragma once

#include "math/SpaceMath.h"
#include "math/WorldMath.h"
#include "world/PlanetConfig.h"

namespace planet {

inline constexpr double kSurfaceGravity = 3.0;
inline constexpr double kMu             = kSurfaceGravity * kPlanetRadius * kPlanetRadius;

inline constexpr double kShipMass   = 50000.0;
inline constexpr float  kShipLength = 20.0f;
inline constexpr float  kShipSpan   = 12.0f;
inline constexpr float  kShipHeight = 5.0f;

inline constexpr double kInertiaPitch = kShipMass / 12.0 * (kShipHeight * kShipHeight + kShipLength * kShipLength);
inline constexpr double kInertiaYaw   = kShipMass / 12.0 * (kShipSpan   * kShipSpan   + kShipLength * kShipLength);
inline constexpr double kInertiaRoll  = kShipMass / 12.0 * (kShipSpan   * kShipSpan   + kShipHeight * kShipHeight);

inline constexpr double kAccelMain    = 40.0;
inline constexpr double kAccelRetro   = 20.0;
inline constexpr double kAccelLateral = 15.0;
inline constexpr double kAccelVert    = 15.0;

inline constexpr float kRateMaxPitch = 1.2f;
inline constexpr float kRateMaxYaw   = 1.0f;
inline constexpr float kRateMaxRoll  = 2.5f;
inline constexpr float kAngAccelPitch = 2.0f;
inline constexpr float kAngAccelYaw   = 1.6f;
inline constexpr float kAngAccelRoll  = 3.5f;

inline constexpr double kScmSpeed    = 250.0;
inline constexpr double kNavSpeed    = 1200.0;
inline constexpr float  kNavRateScale = 0.35f;

inline constexpr float kTauAng = 0.12f;
inline constexpr float kTauLin = 0.50f;

inline constexpr double kFixedDt     = 1.0 / 120.0;
inline constexpr int    kMaxSubsteps = 16;

inline constexpr float kGearLegLength  = 4.0f;
inline constexpr float kGearTravel     = 0.6f;
inline constexpr float kGearDeployTime = 1.5f;
inline const     space::Vec3 kGearPivot[4] = {
    { -5.0f, -2.5f, -4.0f }, {  5.0f, -2.5f, -4.0f },
    { -5.0f, -2.5f,  4.0f }, {  5.0f, -2.5f,  4.0f },
};

inline constexpr double kGearK = 1.07e5;

inline constexpr double kGearC = 1.4e5;

inline constexpr double kGearFriction = 0.9;

inline constexpr double kLandedSpeed   = 0.05;
inline constexpr double kLandedSpin    = 0.02;
inline constexpr double kLandedHold    = 0.5;

inline constexpr double kLandedDamping = 8.0;

inline constexpr double kShipSpawnAltitude = 5000.0;

inline constexpr float kStickGain      = 0.010f;
inline constexpr float kStickReturnTau = 0.35f;
inline constexpr float kThrottleRate   = 0.5f;
inline constexpr float kThrottleNotch  = 0.05f;

inline const space::Vec3 kEyeOffset   { 0.0f, 1.30f,  5.50f };

inline constexpr double kQuantumSpeed = 200000.0;
inline constexpr double kQuantumAccel =  40000.0;

inline constexpr double kQuantumBrakeMargin = 0.90;

inline constexpr double kSpoolTime       = 3.0;
inline constexpr float  kAlignTolerance  = 0.035f;
inline constexpr double kArrivalStandoff = 5000.0;
inline constexpr int    kDestinationCount = 6;

inline constexpr double kSiteSearchRadius = 0.30;
inline constexpr int    kSiteSearchSteps  = 512;

inline constexpr double kSiteMinRelief = 200.0;

inline constexpr double kMinTransferAltitude =  100000.0;
inline constexpr double kTransferAltitude    = 3600000.0;
inline constexpr double kTransferMargin      = 1.03;

inline constexpr float kConsoleTopY  = 0.85f;
inline constexpr float kConsoleZNear = 6.30f;
inline constexpr float kConsoleZFar  = 6.90f;
inline constexpr float kConsoleHalfX = 1.00f;
inline constexpr float kCoamingZNear = 6.60f;
inline constexpr float kCoamingTopY  = 0.98f;

inline const space::Vec3 kSwitchHalf { 0.055f, 0.045f, 0.022f };

inline constexpr float kSwitchReach = 1.60f;

inline constexpr float kCockpitAmbient = 0.45f;

inline constexpr float kSwitchAmbientOff = kCockpitAmbient;
inline constexpr float kSwitchAmbientOn  = 0.90f;

inline constexpr float kSwitchTilt = 0.35f;

inline const space::Vec3 kChaseOffset { 0.0f, 4.00f, -26.0f };

inline constexpr float kChaseRotTau = 0.18f;
inline constexpr float kChaseMaxLag = 0.44f;

inline constexpr float kFreeLookYawCockpit   = 1.92f;
inline constexpr float kFreeLookPitchCockpit = 1.31f;
inline constexpr float kFreeLookYawChase     = 3.14159f;
inline constexpr float kFreeLookPitchChase   = 1.40f;
inline constexpr float kFreeLookReturnTau    = 0.15f;
inline constexpr float kLookSensitivity      = 0.0022f;

inline constexpr int kPhysicsDetailOctaves = detailOctavesForLevel(kMaxDepth);

}
