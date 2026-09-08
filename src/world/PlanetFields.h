#pragma once

#include "world/PlanetParams.h"

#include <cstddef>
#include <cstdint>

namespace planet {

struct ParamField {
    enum class Kind { Float, Int, Uint, Double };

    const char* key;
    const char* group;
    Kind        kind;
    std::size_t offset;
    double      lo, hi;
};

inline double fieldGet(const PlanetParams& p, const ParamField& f) {
    const char* base = reinterpret_cast<const char*>(&p);
    switch (f.kind) {
        case ParamField::Kind::Float:  return double(*reinterpret_cast<const float*>(base + f.offset));
        case ParamField::Kind::Int:    return double(*reinterpret_cast<const int*>(base + f.offset));
        case ParamField::Kind::Uint:   return double(*reinterpret_cast<const unsigned*>(base + f.offset));
        case ParamField::Kind::Double: return *reinterpret_cast<const double*>(base + f.offset);
    }
    return 0.0;
}

inline void fieldSet(PlanetParams& p, const ParamField& f, double v) {
    char* base = reinterpret_cast<char*>(&p);
    switch (f.kind) {
        case ParamField::Kind::Float:  *reinterpret_cast<float*>(base + f.offset) = float(v); break;
        case ParamField::Kind::Int:    *reinterpret_cast<int*>(base + f.offset) = int(v < 0 ? v - 0.5 : v + 0.5); break;
        case ParamField::Kind::Uint:   *reinterpret_cast<unsigned*>(base + f.offset) = unsigned(v < 0 ? 0 : v + 0.5); break;
        case ParamField::Kind::Double: *reinterpret_cast<double*>(base + f.offset) = v; break;
    }
}

#define PLANET_FIELD(member, kind, group, lo, hi) \
    ParamField{ #member, group, ParamField::Kind::kind, offsetof(PlanetParams, member), lo, hi }

inline const ParamField kParamFields[] = {
    PLANET_FIELD(seed,              Uint,   "identity",   0,     100000),

    PLANET_FIELD(noiseFrequency,    Float,  "continents", 0.2,   6.0),
    PLANET_FIELD(noiseLacunarity,   Float,  "continents", 1.5,   3.0),
    PLANET_FIELD(noiseGain,         Float,  "continents", 0.2,   0.8),
    PLANET_FIELD(noiseOctaves,      Int,    "continents", 1,     14),
    PLANET_FIELD(seaLevelNoise,     Float,  "continents", -0.5,  0.5),
    PLANET_FIELD(terrainAmplitude,  Double, "continents", 500,   20000),

    PLANET_FIELD(detailAmplitude,   Double, "detail",     0,     1500),
    PLANET_FIELD(detailGain,        Float,  "detail",     0.2,   0.8),
    PLANET_FIELD(detailFadeIn,      Float,  "detail",     0.001, 0.2),
    PLANET_FIELD(detailRidgeMix,    Float,  "detail",     0,     1),
    PLANET_FIELD(detailRidgeFull,   Int,    "detail",     0,     12),
    PLANET_FIELD(detailRidgeFade,   Int,    "detail",     0,     12),

    PLANET_FIELD(terraceSteps,      Float,  "terracing",  1,     20),
    PLANET_FIELD(terraceRiser,      Float,  "terracing",  0,     1),
    PLANET_FIELD(terraceMix,        Float,  "terracing",  0,     1),

    PLANET_FIELD(moistureOctaves,   Int,    "climate",    1,     8),
    PLANET_FIELD(moistureFrequency, Float,  "climate",    0.3,   6.0),
    PLANET_FIELD(reliefOctaves,     Int,    "climate",    1,     8),
    PLANET_FIELD(reliefFrequency,   Float,  "climate",    0.3,   6.0),

    PLANET_FIELD(reliefPlains,      Float,  "landforms",  0.2,   0.8),
    PLANET_FIELD(reliefMesa,        Float,  "landforms",  0.2,   0.8),
    PLANET_FIELD(reliefMountain,    Float,  "landforms",  0.2,   0.9),

    PLANET_FIELD(rangeSharpness,    Float,  "ranges",     1,     16),
    PLANET_FIELD(rangeBoost,        Float,  "ranges",     1,     8),
    PLANET_FIELD(rangeBlend,        Float,  "ranges",     0,     1),
    PLANET_FIELD(rangeOctaves,      Int,    "ranges",     1,     8),
    PLANET_FIELD(rangeCoastFade,    Float,  "ranges",     0.005, 0.3),

    PLANET_FIELD(spireAmplitude,        Double, "spires", 0,     3000),
    PLANET_FIELD(spireFrequency,        Float,  "spires", 100,   2000),
    PLANET_FIELD(spireOctaves,          Int,    "spires", 1,     5),
    PLANET_FIELD(spireCrest,            Float,  "spires", 0.3,   0.95),
    PLANET_FIELD(spireWall,             Float,  "spires", 0.01,  0.3),
    PLANET_FIELD(spireProvinceOctaves,  Int,    "spires", 1,     5),
    PLANET_FIELD(spireProvinceFrequency,Float,  "spires", 0.5,   8.0),
    PLANET_FIELD(spireProvinceLo,       Float,  "spires", 0.3,   0.8),
    PLANET_FIELD(spireProvinceHi,       Float,  "spires", 0.3,   0.9),
    PLANET_FIELD(spireOctaveIn,         Int,    "spires", 0,     10),
    PLANET_FIELD(spireOctaveFull,       Int,    "spires", 1,     12),

    PLANET_FIELD(bandBeach,         Float,  "bands",      0,     0.05),
    PLANET_FIELD(bandShore,         Float,  "bands",      0,     0.1),
    PLANET_FIELD(bandLowland,       Float,  "bands",      0,     0.3),
    PLANET_FIELD(bandHighland,      Float,  "bands",      0,     0.5),
    PLANET_FIELD(bandPeak,          Float,  "bands",      0,     0.8),
    PLANET_FIELD(slopeSoft,         Float,  "bands",      0,     0.5),
    PLANET_FIELD(slopeRock,         Float,  "bands",      0,     0.8),
    PLANET_FIELD(latTemperate,      Float,  "bands",      0,     1),
    PLANET_FIELD(latPolar,          Float,  "bands",      0,     1),
    PLANET_FIELD(moistDry,          Float,  "bands",      0.1,   0.9),
    PLANET_FIELD(moistWet,          Float,  "bands",      0.1,   0.9),

    PLANET_FIELD(groundTileMetres,     Float, "ground",   0.5,   8),
    PLANET_FIELD(groundMacroMetres,    Float, "ground",   2,     64),
    PLANET_FIELD(groundPaletteMix,     Float, "ground",   0,     1),
    PLANET_FIELD(groundNormalStrength, Float, "ground",   0,     3),

    PLANET_FIELD(scatterDensity,      Float,  "scatter",  0,     1),
    PLANET_FIELD(scatterMinElevation, Float,  "scatter",  0,     0.2),
    PLANET_FIELD(scatterMaxSlope,     Float,  "scatter",  0,     1),
    PLANET_FIELD(scatterMinScale,     Float,  "scatter",  0.2,   20),
    PLANET_FIELD(scatterMaxScale,     Float,  "scatter",  0.2,   30),
    PLANET_FIELD(scatterCeiling,      Double, "scatter",  100,   8000),
};

#undef PLANET_FIELD

inline constexpr int kParamFieldCount =
    int(sizeof(kParamFields) / sizeof(kParamFields[0]));

inline const char* const kBiomeNames[kBiomeCount] = {
    "ocean", "plains", "mesa", "mountains"
};
inline const char* const kBiomeFactorNames[4] = {
    "amp", "detail", "ridge", "terrace"
};

inline float& biomeFactor(PlanetParams& p, int biome, int factor) {
    BiomeScales& s = p.biomeScales[biome];
    switch (factor) {
        case 0:  return s.amp;
        case 1:  return s.detail;
        case 2:  return s.ridge;
        default: return s.terrace;
    }
}

inline const float& biomeFactor(const PlanetParams& p, int biome, int factor) {
    return biomeFactor(const_cast<PlanetParams&>(p), biome, factor);
}

}
