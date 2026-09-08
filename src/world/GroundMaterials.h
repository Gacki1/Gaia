#pragma once

#include "world/PlanetParams.h"
#include <cmath>

namespace planet {

enum GroundMat {
    kMatSand = 0,
    kMatDryLow,
    kMatWetLow,
    kMatUpland,
    kMatRock,
    kMatSnow,
    kGroundMatCount
};

struct GroundWeights { float w[kGroundMatCount]; };

inline GroundWeights groundWeightsAt(const PlanetParams& p, float e, float slope,
                                     float lat, float moisture) {
    GroundWeights g{};

    const float toLow    = smoothBand(p.bandBeach,    p.bandShore,    e);
    const float toUpland = smoothBand(p.bandLowland,  p.bandHighland, e);
    const float toSnow   = smoothBand(p.bandHighland, p.bandPeak,     e);

    const float wet = smoothBand(p.moistDry, p.moistWet, moisture);

    float sand   = 1.0f - toLow;
    float low    = toLow * (1.0f - toUpland);
    float upland = toUpland * (1.0f - toSnow);
    float snow   = toSnow;

    const float polar = smoothBand(p.latTemperate, p.latPolar, std::fabs(lat));
    const float keep  = 1.0f - polar;
    sand *= keep; low *= keep; upland *= keep;
    snow  = snow * keep + polar;

    const float rock = smoothBand(p.slopeSoft, p.slopeRock, slope);
    const float rest = 1.0f - rock;

    const float green = (low + upland) * wet;

    g.w[kMatSand]   = sand   * rest;
    g.w[kMatDryLow] = low    * (1.0f - wet) * rest;
    g.w[kMatWetLow] = green  * rest;
    g.w[kMatUpland] = upland * (1.0f - wet) * rest;
    g.w[kMatSnow]   = snow   * rest;
    g.w[kMatRock]   = rock;
    return g;
}

enum GroundLayer {
    kLayerSand = 0,
    kLayerDirt,
    kLayerForestFloor,
    kLayerGravel,
    kLayerRock,
    kLayerSnow,
    kGroundLayerCount
};

static_assert(kGroundLayerCount <= 8,
              "the vertex carries eight ground weights; a ninth layer needs a "
              "wider vertex, not a wider loop");

struct GroundLayerDesc {
    const char* token;
    const char* label;
};

inline const GroundLayerDesc kGroundLayers[kGroundLayerCount] = {
    { "sand",         "sand / shingle"    },
    { "dirt_ground",  "bare earth"        },
    { "forest_floor", "forest floor"      },
    { "gravel",       "gravel / scree"    },
    { "rock",         "cliff rock"        },
    { "snow",         "snow"              },
};

inline constexpr int kMaxVariants = 3;

struct GroundSlotDesc {
    int layer[kMaxVariants];
    int count;
};

inline constexpr GroundSlotDesc kGroundSlots[kGroundMatCount] = {
     { { kLayerSand,        0, 0 }, 1 },
     { { kLayerDirt,        0, 0 }, 1 },

     { { kLayerForestFloor, kLayerDirt, 0 }, 1 },
     { { kLayerGravel,      0, 0 }, 1 },
     { { kLayerRock,        0, 0 }, 1 },
     { { kLayerSnow,        0, 0 }, 1 },
};

inline void canopySplit(int n, float canopy, float* out) {
    if (n <= 1) { out[0] = 1.0f; return; }
    const float span = kCanopyClosed - kCanopyOpen;
    float prev = 1.0f;
    for (int i = 0; i < n - 1; ++i) {
        const float lo = kCanopyOpen + span * float(i)     / float(n - 1);
        const float hi = kCanopyOpen + span * float(i + 1) / float(n - 1);
        const float t  = smoothBand(lo, hi, canopy);
        out[i] = prev - t;
        prev = t;
    }
    out[n - 1] = prev;
}

struct GroundLayerWeights { float w[kGroundLayerCount]; };

inline GroundLayerWeights groundLayerWeights(const PlanetParams& p, float e,
                                             float slope, float lat,
                                             float moisture, float canopy) {
    const GroundWeights g = groundWeightsAt(p, e, slope, lat, moisture);
    GroundLayerWeights l{};
    for (int m = 0; m < kGroundMatCount; ++m) {
        if (g.w[m] <= 0.0f) continue;
        const GroundSlotDesc& sd = kGroundSlots[m];
        if (sd.count == 1) { l.w[sd.layer[0]] += g.w[m]; continue; }
        float v[kMaxVariants];
        canopySplit(sd.count, canopy, v);

        for (int k = 0; k < sd.count; ++k) l.w[sd.layer[k]] += g.w[m] * v[k];
    }
    return l;
}

}
