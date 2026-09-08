#pragma once

#include "math/SpaceMath.h"
#include "math/WorldMath.h"

namespace planet {

using space::Vec3;

inline constexpr int   kWindowWidth  = 1280;
inline constexpr int   kWindowHeight = 720;

inline constexpr double kPlanetRadius = 1000000.0;

inline constexpr double kTerrainAmplitude = 11000.0;

inline constexpr unsigned kNoiseSeed   = 1337u;
inline constexpr float    kNoiseFrequency  = 1.35f;
inline constexpr float    kNoiseLacunarity = 2.0f;
inline constexpr float    kNoiseGain       = 0.5f;

inline constexpr int      kNoiseOctaves = 8;

inline constexpr double   kDetailAmplitude   = 2700.0;
inline constexpr float    kDetailGain        = 0.38f;
inline constexpr int      kMaxDetailOctaves  = 12;

inline constexpr float    kDetailFadeIn      = 0.02f;

inline constexpr int detailOctavesForLevel(int level) {
    const int n = level - 7;
    return n < 0 ? 0 : (n > kMaxDetailOctaves ? kMaxDetailOctaves : n);
}

inline constexpr float kSeaLevelNoise = 0.02f;

inline constexpr float  kDetailRidgeMix   = 0.92f;
inline constexpr int    kDetailRidgeFull  = 4;
inline constexpr int    kDetailRidgeFade  = 3;

inline constexpr float  kTerraceSteps = 7.0f;
inline constexpr float  kTerraceRiser = 0.35f;
inline constexpr float  kTerraceMix   = 0.35f;

inline constexpr float kBandShore    = 0.012f;
inline constexpr float kBandLowland  = 0.055f;
inline constexpr float kBandHighland = 0.130f;
inline constexpr float kBandPeak     = 0.230f;

inline constexpr float kSlopeSoft = 0.045f;
inline constexpr float kSlopeRock = 0.160f;

inline constexpr float kLatTemperate = 0.78f;
inline constexpr float kLatPolar     = 0.94f;

inline const Vec3 kPlanetAxis{ 0.0f, 1.0f, 0.0f };

inline constexpr int   kMoistureOctaves   = 3;
inline constexpr float kMoistureFrequency = 1.9f;

inline constexpr float kCanopyFrequency = 1500.0f;
inline constexpr int   kCanopyOctaves   = 3;

inline constexpr float kCanopyOpen   = 0.430f;
inline constexpr float kCanopyClosed = 0.470f;

inline constexpr int   kReliefOctaves   = 3;
inline constexpr float kReliefFrequency = 1.30f;

inline constexpr float kRangeSharpness = 6.0f;

inline constexpr float kRangeBoost = 3.2f;

inline constexpr float kRangeBlend = 0.75f;

inline constexpr int kRangeOctaves = 4;

inline constexpr float kRangeCoastFade = 0.06f;

enum BiomeId { kBiomeOcean = 0, kBiomePlains, kBiomeMesa, kBiomeMountains,
               kBiomeCount };

inline constexpr float kReliefPlains   = 0.450f;
inline constexpr float kReliefMesa     = 0.535f;
inline constexpr float kReliefMountain = 0.615f;

struct BiomeScales { float amp, detail, ridge, terrace; };

inline constexpr BiomeScales kBiomeScales[kBiomeCount] = {
     { 1.00f, 1.00f, 1.00f, 1.00f },
     { 0.30f, 0.45f, 0.25f, 0.20f },
     { 0.60f, 0.55f, 0.35f, 1.00f },
     { 1.00f, 1.00f, 1.00f, 0.10f },
};

inline const Vec3 kColDeepWater  {0.02f, 0.06f, 0.09f};
inline const Vec3 kColShallowWtr {0.04f, 0.20f, 0.20f};
inline const Vec3 kColShore      {0.38f, 0.30f, 0.20f};
inline const Vec3 kColLowlandDry {0.30f, 0.17f, 0.10f};
inline const Vec3 kColLowlandWet {0.22f, 0.10f, 0.26f};
inline const Vec3 kColHighland   {0.26f, 0.27f, 0.24f};
inline const Vec3 kColPeak       {0.62f, 0.65f, 0.68f};
inline const Vec3 kColRock       {0.22f, 0.20f, 0.19f};
inline const Vec3 kColPolar      {0.58f, 0.68f, 0.74f};

inline constexpr int kScatterLevel = 16;

inline constexpr float kScatterDensity = 0.78f;

inline constexpr float kScatterMinElevation = 0.01f;
inline constexpr float kScatterMaxSlope     = 0.55f;

inline constexpr float kScatterMinScale = 2.0f;
inline constexpr float kScatterMaxScale = 8.5f;

inline constexpr double kScatterRange        = 450.0;
inline constexpr int    kScatterMaxInstances = 1200;

inline constexpr double kScatterRebuild = 120.0;

inline constexpr double kScatterCeiling = 1500.0;

inline constexpr double kOutpostVisibleRange = 40000.0;

inline constexpr float kOutpostAmbient = 0.30f;

inline float smoothBand(float a, float b, float x) {
    if (b <= a) return x < a ? 0.0f : 1.0f;
    float t = (x - a) / (b - a);
    t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
    return t * t * (3.0f - 2.0f * t);
}

inline constexpr float kBandBeach = 0.004f;

inline constexpr float kMoistDry = 0.345f;
inline constexpr float kMoistWet = 0.455f;

inline constexpr float kGroundTileMetres = 2.0f;

inline constexpr float kGroundMacroMetres = 24.0f;

inline constexpr float kGroundPaletteMix = 0.0f;

inline constexpr float kGroundNormalStrength = 1.0f;

inline const     Vec3  kSunDirection {-0.40f, -0.75f, -0.52f};
inline constexpr float kAmbient = 0.12f;

inline constexpr double kAtmosphereTop        = 80000.0;
inline constexpr double kRayleighScaleHeight  = 15000.0;
inline constexpr double kMieScaleHeight       =  2500.0;

inline constexpr double kBetaRayleighR =  5.802e-6;
inline constexpr double kBetaRayleighG = 13.558e-6;
inline constexpr double kBetaRayleighB = 33.100e-6;

inline constexpr double kBetaMieScatter = 3.996e-6;
inline constexpr double kBetaMieExtinct = 4.440e-6;

inline constexpr float kMieG = 0.76f;

inline constexpr float kSunIrradiance = 20.0f;

inline constexpr float kExposure = 0.12f;

inline constexpr float kSunAngularRadius = 4.625e-3f;

inline constexpr float kSunDiscRadiance = 5000.0f;

inline constexpr int kLutWidth           = 256;
inline constexpr int kLutHeight          =  64;
inline constexpr int kTransmittanceSteps =  40;

inline constexpr int kSkySteps = 16;

inline constexpr int kPlateCount = 10;

inline constexpr float kPlateJitter = 0.18f;

inline constexpr int kRangeCount = 4;

inline constexpr double kRangeMinLength = 800000.0;

inline constexpr float kRangeCrestWidth  = 0.024f;

inline constexpr float kRangeCrestHeight = 0.48f;

inline constexpr float kRangeJunctionFade = 0.020f;

inline constexpr float kRangeWarpAmp   = 0.20f;

inline constexpr float kRangeWarpFreq  = 6.0f;
inline constexpr int   kRangeWarpOct   = 3;

inline constexpr int   kRangeRidgeMax     = 3;
inline constexpr float kRangeRidgeGapLo   = 1.35f;
inline constexpr float kRangeRidgeGapHi   = 2.10f;
inline constexpr float kRangeRidgeFalloff = 0.62f;

inline constexpr double kOrogenyAmplitude = 8200.0;

inline constexpr float kOrogenyCoastFade = 0.015f;

inline constexpr float kOrogenyColourWeight = 0.20f;

inline constexpr float kPlateauRiser = 0.22f;
inline constexpr float kPlateauMix   = 0.60f;

inline constexpr float kPlateauEdgeFade = 0.06f;

inline constexpr float kPlateauShare = 0.35f;

inline constexpr float kRiftShare = 0.25f;

inline constexpr double kSpireAmplitude = 900.0;

inline constexpr float kSpireFrequency = 620.0f;
inline constexpr int   kSpireOctaves   = 2;

inline constexpr float kSpireCrest = 0.640f;
inline constexpr float kSpireWall  = 0.045f;

inline constexpr int   kSpireProvinceOctaves   = 2;
inline constexpr float kSpireProvinceFrequency = 2.2f;
inline constexpr float kSpireProvinceLo = 0.520f;
inline constexpr float kSpireProvinceHi = 0.600f;

inline constexpr int kSpireOctaveIn   = 2;
inline constexpr int kSpireOctaveFull = 5;

inline constexpr double kLandingFootprint = 10.0;
inline constexpr double kMaxLandingSlope  = 0.14;

inline constexpr int kPatchResolution = 32;

inline constexpr int kMaxDepth = 18;

inline constexpr float kSplitFactor = 0.38f;

inline constexpr float kSplitFactorFar = 0.095f;
inline constexpr float kSplitAltNear   = 20000.0f;
inline constexpr float kSplitAltFar    = 100000.0f;

inline float splitFactorAt(double cameraRadius) {
    const float alt = static_cast<float>(cameraRadius - kPlanetRadius);
    const float t = smoothBand(kSplitAltNear, kSplitAltFar, alt);
    return kSplitFactor + (kSplitFactorFar - kSplitFactor) * t;
}

inline constexpr float kMorphBandStart = 0.65f;

inline constexpr double kPrefetchLead = 400.0;

inline constexpr int kFramesInFlight = 2;

inline constexpr double kUploadBudgetMs = 6.0;

inline constexpr int kMaxUploadsPerFrame = 128;

inline constexpr unsigned kStagingArenaBytes = 16u * 1024u * 1024u;

inline constexpr int kTransferBatches = 3;
inline constexpr int kMaxJobsPerFrame    = 128;
inline constexpr int kMaxJobsInFlight    = 512;

inline constexpr int kMeshCacheCapacity = 6000;

inline constexpr float kFovYDegrees = 60.0f;
inline constexpr float kNearPlane   = 0.05f;

}
