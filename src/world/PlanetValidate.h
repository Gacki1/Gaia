#pragma once

#include "world/PlanetParams.h"
#include "world/Noise.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace planet {

enum class Severity { Error, Warning };

struct Violation {
    Severity    sev;
    const char* field;
    std::string message;
};

inline constexpr double kHullClearance = 3.65;

inline constexpr int kGearOctave = 7;

inline std::vector<Violation> validate(const PlanetParams& p) {
    std::vector<Violation> v;
    auto err = [&](const char* f, std::string m) {
        v.push_back({ Severity::Error, f, std::move(m) });
    };
    auto warn = [&](const char* f, std::string m) {
        v.push_back({ Severity::Warning, f, std::move(m) });
    };

    auto ordered = [&](const char* f, float lo, float hi,
                       const char* loName, const char* hiName) {
        if (!(lo < hi))
            err(f, std::string(loName) + " must be below " + hiName +
                   " -- the ramp between them collapses to a hard step");
    };

    if (p.groundTileMetres <= 0.0f) {
        err("groundTileMetres", "the ground tile must have a positive size");
    } else {
        const float ratio = p.groundMacroMetres / p.groundTileMetres;
        const float whole = float(int(ratio + 0.5f));
        if (p.groundMacroMetres < p.groundTileMetres || std::fabs(ratio - whole) > 1e-4f) {
            char buf[192];
            std::snprintf(buf, sizeof(buf),
                "must be a whole multiple of the fine tile (%.3f m): %.3f / %.3f "
                "= %.4f. The wrapped texture origin does not cancel, and every "
                "patch boundary becomes a seam.",
                double(p.groundTileMetres), double(p.groundMacroMetres),
                double(p.groundTileMetres), double(ratio));
            err("groundMacroMetres", buf);
        }
    }

    for (int i = 0; i < kBiomeCount; ++i) {
        const BiomeScales& s = p.biomeScales[i];
        const float f[4] = { s.amp, s.detail, s.ridge, s.terrace };
        const char* n[4] = { "amp", "detail", "ridge", "terrace" };
        for (int k = 0; k < 4; ++k) {
            if (f[k] > 1.0f) {
                char buf[192];
                std::snprintf(buf, sizeof(buf),
                    "biome %d's %s factor is %.3f. Every factor must stay in "
                    "(0,1]: above 1 the terrain exceeds the amplitude the LOD "
                    "bounds and the landing gear were derived from.",
                    i, n[k], double(f[k]));
                err("biomeScales", buf);
            } else if (!(f[k] > 0.0f)) {
                char buf[160];
                std::snprintf(buf, sizeof(buf),
                    "biome %d's %s factor is %.3f -- at or below zero that "
                    "biome has no terrain at all.", i, n[k], double(f[k]));
                err("biomeScales", buf);
            }
        }
    }

    if (!(p.noiseLacunarity > 1.0f))
        err("noiseLacunarity", "must exceed 1, or every octave repeats the "
                               "one before it and the sum is a single octave");
    if (!(p.noiseGain > 0.0f && p.noiseGain < 1.0f))
        err("noiseGain", "must lie in (0,1): at or above 1 the octaves grow "
                          "without bound, at or below 0 there is one octave");
    if (p.noiseOctaves < 1)
        err("noiseOctaves", "at least one octave is needed for any terrain");
    if (p.noiseOctaves > 14)
        warn("noiseOctaves", "beyond about 14 the continental band's finest "
                             "feature is smaller than a patch vertex, so the "
                             "extra octaves alias rather than add detail");
    if (!(p.noiseFrequency > 0.0f))
        err("noiseFrequency", "must be positive");

    if (!(p.seaLevelNoise > -1.0f && p.seaLevelNoise < 1.0f)) {
        err("seaLevelNoise", "must lie strictly inside (-1,1): sampleShape "
                             "divides by 1 - seaLevelNoise on land and by "
                             "1 + seaLevelNoise at sea");
    }

    if (!(p.terrainAmplitude > 0.0))
        err("terrainAmplitude", "must be positive, or there is no land");
    if (p.detailAmplitude < 0.0)
        err("detailAmplitude", "cannot be negative");
    if (p.spireAmplitude < 0.0)
        err("spireAmplitude", "cannot be negative");

    {
        const double rough = p.detailAmplitude *
                             double(std::pow(p.detailGain, float(kGearOctave))) /
                             double(Noise::fixedNorm(p.detailGain, kMaxDetailOctaves));
        if (rough > kHullClearance) {
            char buf[224];
            std::snprintf(buf, sizeof(buf),
                "roughness over the landing-gear footprint is %.2f m, against "
                "%.2f m of hull clearance. The ship will rest inside the "
                "terrain it lands on. Lower detailAmplitude or detailGain.",
                rough, kHullClearance);
            err("detailAmplitude", buf);
        }
    }

    ordered("bandShore",    p.bandBeach,    p.bandShore,    "bandBeach", "bandShore");
    ordered("bandLowland",  p.bandShore,    p.bandLowland,  "bandShore", "bandLowland");
    ordered("bandHighland", p.bandLowland,  p.bandHighland, "bandLowland", "bandHighland");
    ordered("bandPeak",     p.bandHighland, p.bandPeak,     "bandHighland", "bandPeak");
    ordered("slopeRock",    p.slopeSoft,    p.slopeRock,    "slopeSoft", "slopeRock");
    ordered("latPolar",     p.latTemperate, p.latPolar,     "latTemperate", "latPolar");
    ordered("moistWet",     p.moistDry,     p.moistWet,     "moistDry", "moistWet");
    ordered("reliefMesa",   p.reliefPlains, p.reliefMesa,   "reliefPlains", "reliefMesa");
    ordered("reliefMountain", p.reliefMesa, p.reliefMountain,
            "reliefMesa", "reliefMountain");
    ordered("spireProvinceHi", p.spireProvinceLo, p.spireProvinceHi,
            "spireProvinceLo", "spireProvinceHi");
    if (p.spireOctaveIn >= p.spireOctaveFull)
        err("spireOctaveFull", "spires must fade in across at least one octave, "
                                "or they appear in a single LOD step and pop");

    if (p.bandPeak >= 1.0f)
        warn("bandPeak", "at or above 1 no ground reaches the peak band, so "
                         "the peak colour never appears (landE is capped at 1)");
    if (p.slopeSoft >= 1.0f)
        warn("slopeSoft", "slope is 1 - dot(normal, radial) and only reaches 1 "
                          "on a vertical face, so bare rock will never show");
    if (p.latPolar > 1.0f)
        warn("latPolar", "latitude is sin(lat) and never exceeds 1, so the "
                         "polar cap never fully forms");

    if (p.moistWet <= 0.25f)
        warn("moistWet", "measured moisture over land bottoms out near 0.25, "
                         "so a split below that leaves the planet entirely wet");
    if (p.moistDry >= 0.74f)
        warn("moistDry", "measured moisture over land tops out near 0.74, so a "
                         "split above that leaves the planet entirely dry");
    if (p.spireCrest >= 1.0f)
        warn("spireCrest", "the crest field is a normalised ridge sum and does "
                           "not reach 1, so no spire will ever stand");

    if (!(p.scatterMaxSlope > 0.0f))
        warn("scatterMaxSlope", "at or below zero nothing is ever scattered");
    if (p.scatterMinScale > p.scatterMaxScale)
        err("scatterMinScale", "the minimum scatter scale exceeds the maximum");

    return v;
}

inline bool hasErrors(const std::vector<Violation>& v) {
    for (const Violation& x : v) if (x.sev == Severity::Error) return true;
    return false;
}

}
