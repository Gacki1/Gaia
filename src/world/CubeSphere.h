#pragma once

#include "math/SpaceMath.h"
#include "world/PlanetConfig.h"
#include "world/World.h"
#include "world/Plates.h"
#include "world/GroundMaterials.h"
#include "render/Vertex.h"
#include <vector>
#include <cstdint>
#include <cmath>
#include <algorithm>

namespace planet {

using space::Vec3;

struct Face { Vec3 forward, right, up; };

inline const Face& cubeFace(int i) {
    static const Face kFaces[6] = {
        {{ 1, 0, 0}, { 0, 0,-1}, {0, 1, 0}},
        {{-1, 0, 0}, { 0, 0, 1}, {0, 1, 0}},
        {{ 0, 1, 0}, { 1, 0, 0}, {0, 0,-1}},
        {{ 0,-1, 0}, { 1, 0, 0}, {0, 0, 1}},
        {{ 0, 0, 1}, { 1, 0, 0}, {0, 1, 0}},
        {{ 0, 0,-1}, {-1, 0, 0}, {0, 1, 0}},
    };
    return kFaces[i];
}

inline DVec3 faceToSphereD(int face, double u, double v) {
    const Face& f = cubeFace(face);
    return space::normalize(DVec3{
        f.forward.x + f.right.x * u + f.up.x * v,
        f.forward.y + f.right.y * u + f.up.y * v,
        f.forward.z + f.right.z * u + f.up.z * v });
}

inline Vec3 faceToSphere(int face, float u, float v) {
    return space::toF(faceToSphereD(face, u, v));
}

inline float reliefAt(const World& w, const DVec3& dir) {
    const DVec3 q{ dir.x - 71.849, dir.y + 12.507, dir.z + 44.231 };
    const float r = w.noise.fbm(q, w.p.reliefOctaves, w.p.reliefFrequency,
                              w.p.noiseLacunarity, w.p.noiseGain);
    return 0.5f * (r + 1.0f);
}

struct BiomeWeights { float w[kBiomeCount]; };

inline BiomeWeights biomeWeightsAt(const PlanetParams& p, float eRaw,
                                   float relief, float lat) {
    BiomeWeights b{};
    if (eRaw <= 0.0f) { b.w[kBiomeOcean] = 1.0f; return b; }

    float mountains = smoothBand(p.reliefMesa,   p.reliefMountain, relief);
    const float mesaRaw = smoothBand(p.reliefPlains, p.reliefMesa, relief);

    mountains *= 1.0f - smoothBand(p.latTemperate, p.latPolar, std::fabs(lat));

    const float mesa   = mesaRaw * (1.0f - mountains);
    const float plains = 1.0f - mountains - mesa;

    b.w[kBiomeMountains] = mountains;
    b.w[kBiomeMesa]      = mesa;
    b.w[kBiomePlains]    = plains < 0.0f ? 0.0f : plains;
    return b;
}

inline BiomeScales blendScales(const PlanetParams& p, const BiomeWeights& b) {
    BiomeScales s{ 0.0f, 0.0f, 0.0f, 0.0f };
    for (int i = 0; i < kBiomeCount; ++i) {
        s.amp     += b.w[i] * p.biomeScales[i].amp;
        s.detail  += b.w[i] * p.biomeScales[i].detail;
        s.ridge   += b.w[i] * p.biomeScales[i].ridge;
        s.terrace += b.w[i] * p.biomeScales[i].terrace;
    }
    return s;
}

inline float terraceShape(float e, float steps, float riser) {
    const float x    = e * steps;
    const float base = std::floor(x);
    const float f    = x - base;
    const float t    = f <= 1.0f - riser ? 0.0f : (f - (1.0f - riser)) / riser;
    return (base + t * t * (3.0f - 2.0f * t)) / steps;
}

struct TerrainShape {
    float landE;
    float amp, detail, ridge;

    float orogeny  = 0.0f;
    float rangeTerrace = 0.0f;
};

inline TerrainShape sampleShape(const World& w, const DVec3& dir) {

    const Noise::DualFbm f = w.noise.fbmDual(dir, w.p.noiseOctaves, w.p.noiseFrequency,
                                           w.p.noiseLacunarity, w.p.noiseGain,
                                           w.p.rangeOctaves);
    const float land = f.plain - w.p.seaLevelNoise;

    if (land < 0.0f)
        return { land / (1.0f + w.p.seaLevelNoise), 1.0f, 1.0f, 1.0f };

    const float eRaw = land / (1.0f - w.p.seaLevelNoise);
    const float lat  = static_cast<float>(space::dot(dir, DVec3{ kPlanetAxis.x,
                                                                 kPlanetAxis.y,
                                                                 kPlanetAxis.z }));
    const BiomeWeights b = biomeWeightsAt(w.p, eRaw, reliefAt(w, dir), lat);
    const BiomeScales  s = blendScales(w.p, b);

    float ranged = std::pow(f.ridged, w.p.rangeSharpness) * w.p.rangeBoost;
    ranged = ranged > 1.0f ? 1.0f : ranged;

    const float blend = w.p.rangeBlend * b.w[kBiomeMountains] *
                        smoothBand(0.0f, w.p.rangeCoastFade, eRaw);
    float e = eRaw + blend * (ranged - eRaw);
    e = e < 0.0f ? 0.0f : (e > 1.0f ? 1.0f : e);

    const float plateau = plateauAt(w.plates, dir);
    const float riser   = w.p.terraceRiser +
                          plateau * (kPlateauRiser - w.p.terraceRiser);
    const float tmix    = w.p.terraceMix +
                          plateau * (kPlateauMix - w.p.terraceMix);
    const float landE = e + tmix * s.terrace *
                        (terraceShape(e, w.p.terraceSteps, riser) - e);

    const DVec3 wq{ dir.x + 31.709, dir.y - 17.311, dir.z + 53.137 };
    auto warpAxis = [&](double ox, double oy, double oz) {
        return double(w.noise.fbm(DVec3{ wq.x + ox, wq.y + oy, wq.z + oz },
                                  kRangeWarpOct, kRangeWarpFreq,
                                  w.p.noiseLacunarity, w.p.noiseGain));
    };
    DVec3 disp{ warpAxis(0.0, 0.0, 0.0),
                warpAxis(101.7, -63.1, 19.4),
                warpAxis(-47.3, 88.9, -12.6) };
    disp = disp - dir * space::dot(disp, dir);
    const DVec3 dirW = space::normalize(dir + disp * double(kRangeWarpAmp));
    const Orogeny oro = orogenyAt(w.plates, dirW);

    const float coast = smoothBand(0.0f, kOrogenyCoastFade, eRaw);

    const float floorLimit = -(landE * s.amp);
    float orogeny = oro.height * coast;
    if (orogeny < floorLimit) orogeny = floorLimit;

    return { landE, s.amp, s.detail, s.ridge, orogeny, oro.terrace * coast };
}

inline float sampleLandE(const World& w, const DVec3& dir) {
    return sampleShape(w, dir).landE;
}

inline double terrainRadius(const PlanetParams& p, const TerrainShape& s) {

    return kPlanetRadius + ((s.landE > 0.0f ? static_cast<double>(s.landE) : 0.0) *
                            static_cast<double>(s.amp) +
                            static_cast<double>(s.orogeny)) * p.terrainAmplitude;
}

inline float spireCrest(const World& w, const DVec3& p) {
    float sum = 0.0f, amp = 1.0f, norm = 0.0f, f = w.p.spireFrequency;
    for (int i = 0; i < w.p.spireOctaves; ++i) {
        sum  += amp * Noise::ridge(w.noise.fbm(p, 1, f, w.p.noiseLacunarity, w.p.noiseGain));
        norm += amp;
        amp  *= w.p.noiseGain;
        f    *= w.p.noiseLacunarity;
    }
    return norm > 0.0f ? sum / norm : 0.0f;
}

inline float spireField(const World& w, const DVec3& dir) {
    const DVec3 pm{ dir.x - 12.907, dir.y + 71.331, dir.z - 33.117 };
    const float pRaw = 0.5f * (w.noise.fbm(pm, w.p.spireProvinceOctaves,
                                         w.p.spireProvinceFrequency,
                                         w.p.noiseLacunarity, w.p.noiseGain) + 1.0f);
    const float province = smoothBand(w.p.spireProvinceLo, w.p.spireProvinceHi, pRaw);
    if (province <= 0.0f) return 0.0f;

    const DVec3 a{ dir.x + 5.113,  dir.y + 2.771,  dir.z - 9.317  };
    const DVec3 b{ dir.x - 61.703, dir.y - 4.139,  dir.z + 27.911 };
    const float ta = smoothBand(w.p.spireCrest, w.p.spireCrest + w.p.spireWall,
                                spireCrest(w, a));
    if (ta <= 0.0f) return 0.0f;
    const float tb = smoothBand(w.p.spireCrest, w.p.spireCrest + w.p.spireWall,
                                spireCrest(w, b));
    return province * ta * tb;
}

inline double spireHeight(const World& w, const DVec3& dir,
                          const TerrainShape& sh, int octaves) {
    if (sh.landE <= 0.0f) return 0.0;
    const float lodFade = smoothBand(float(w.p.spireOctaveIn), float(w.p.spireOctaveFull),
                                     float(octaves));
    if (lodFade <= 0.0f) return 0.0;

    const float e = sh.landE * sh.amp;
    const float base = smoothBand(w.p.bandShore, w.p.bandLowland, e) *
                       (1.0f - smoothBand(w.p.bandHighland, w.p.bandPeak, e));
    if (base <= 0.0f) return 0.0;

    return double(spireField(w, dir) * base * lodFade) * w.p.spireAmplitude;
}

inline double detailHeight(const World& w, const DVec3& dir,
                           const TerrainShape& sh, int octaves) {
    const float landE = sh.landE;
    if (octaves <= 0 || landE <= 0.0f) return 0.0;
    float mask = landE >= w.p.detailFadeIn ? 1.0f : landE / w.p.detailFadeIn;

    const float headroom = static_cast<float>(
        static_cast<double>(landE) * w.p.terrainAmplitude * sh.amp /
        (w.p.detailAmplitude * sh.detail));
    if (headroom < mask) mask = headroom;

    float baseFreq = w.p.noiseFrequency;
    for (int i = 0; i < w.p.noiseOctaves; ++i) baseFreq *= w.p.noiseLacunarity;

    const float d = w.noise.fbmPrefixRidged(dir, octaves, baseFreq, w.p.noiseLacunarity,
                                          w.p.detailGain, kMaxDetailOctaves,
                                          w.p.detailRidgeMix * sh.ridge,
                                          w.p.detailRidgeFull, w.p.detailRidgeFade);
    return static_cast<double>(d) * static_cast<double>(mask) *
           w.p.detailAmplitude * static_cast<double>(sh.detail);
}

inline double surfaceRadius(const World& w, const DVec3& dir,
                            const TerrainShape& sh, int octaves) {
    return terrainRadius(w.p, sh) + detailHeight(w, dir, sh, octaves) +
           spireHeight(w, dir, sh, octaves);
}

inline double surfaceRadiusAt(const World& w, const DVec3& dir, int octaves) {
    return surfaceRadius(w, dir, sampleShape(w, dir), octaves);
}

inline DVec3 terrainPos(const World& w, const DVec3& dir, int octaves) {
    return dir * surfaceRadiusAt(w, dir, octaves);
}

inline Vec3 lerpCol(const Vec3& a, const Vec3& b, float t) {
    t = t < 0 ? 0 : (t > 1 ? 1 : t);
    return a * (1.0f - t) + b * t;
}

inline float canopyAt(const World& w, const DVec3& dir) {
    const DVec3 q{ dir.x - 71.523, dir.y + 12.907, dir.z - 38.441 };
    const float c = w.noise.fbm(q, kCanopyOctaves, kCanopyFrequency,
                                w.p.noiseLacunarity, w.p.noiseGain);
    return 0.5f * (c + 1.0f);
}

inline float moistureAt(const World& w, const DVec3& dir) {
    const DVec3 q{ dir.x + 23.117, dir.y - 51.409, dir.z + 8.663 };
    const float m = w.noise.fbm(q, w.p.moistureOctaves, w.p.moistureFrequency,
                              w.p.noiseLacunarity, w.p.noiseGain);
    return 0.5f * (m + 1.0f);
}

inline Vec3 biomeColor(const PlanetParams& p, float landE, float slope,
                       float lat, float moisture) {
    if (landE <= 0.0f) {
        return lerpCol(kColShallowWtr, kColDeepWater, -landE);
    }
    const float e = landE > 1.0f ? 1.0f : landE;

    const Vec3 lowland = lerpCol(kColLowlandDry, kColLowlandWet, moisture);
    Vec3 col = lerpCol(kColShore,  lowland,      smoothBand(p.bandShore,   p.bandLowland,  e));
    col      = lerpCol(col,        kColHighland, smoothBand(p.bandLowland, p.bandHighland, e));
    col      = lerpCol(col,        kColPeak,     smoothBand(p.bandHighland, p.bandPeak,    e));

    col = lerpCol(col, kColRock, smoothBand(p.slopeSoft, p.slopeRock, slope));

    return lerpCol(col, kColPolar,
                   smoothBand(p.latTemperate, p.latPolar, std::fabs(lat)));
}

static_assert(kGroundMacroMetres >= kGroundTileMetres &&
              kGroundMacroMetres ==
                  kGroundTileMetres * float(int(kGroundMacroMetres / kGroundTileMetres)),
              "the macro ground tile must be a whole number of fine tiles, or "
              "the wrapped texture origin does not cancel at both scales");

inline DVec3 groundTexOrigin(const PlanetParams& p, const DVec3& patchOrigin) {
    const double macro = double(p.groundMacroMetres);
    auto wrap = [macro](double v) {
        const double m = std::fmod(v, macro);
        return m < 0.0 ? m + macro : m;
    };
    return { wrap(patchOrigin.x), wrap(patchOrigin.y), wrap(patchOrigin.z) };
}

inline DVec3 patchOrigin(int face, int level, int ix, int iy) {
    const double size = 2.0 / static_cast<double>(1 << level);
    const double u0 = -1.0 + size * static_cast<double>(ix);
    const double v0 = -1.0 + size * static_cast<double>(iy);
    return faceToSphereD(face, u0 + size * 0.5, v0 + size * 0.5) *
           static_cast<double>(kPlanetRadius);
}

struct SurfaceSample {
    DVec3 pos;
    Vec3  normal;
    Vec3  color;

    float mat[kGroundLayerCount];
};

inline SurfaceSample sampleSurface(const World& w, int face, double u, double v,
                                   double delta, int detailOctaves) {
    const DVec3 dir = faceToSphereD(face, u, v);
    const TerrainShape sh = sampleShape(w, dir);
    const float landE = sh.landE;

    SurfaceSample s;
    s.pos = dir * surfaceRadius(w, dir, sh, detailOctaves);

    const DVec3 pu1 = terrainPos(w, faceToSphereD(face, u + delta, v), detailOctaves);
    const DVec3 pu0 = terrainPos(w, faceToSphereD(face, u - delta, v), detailOctaves);
    const DVec3 pv1 = terrainPos(w, faceToSphereD(face, u, v + delta), detailOctaves);
    const DVec3 pv0 = terrainPos(w, faceToSphereD(face, u, v - delta), detailOctaves);
    DVec3 nd = space::normalize(space::cross(pv1 - pv0, pu1 - pu0));
    if (space::dot(nd, dir) < 0.0) nd = -nd;

    const float slope = 1.0f - static_cast<float>(space::dot(nd, dir));
    const float lat = static_cast<float>(space::dot(dir, DVec3{ kPlanetAxis.x,
                                                                kPlanetAxis.y,
                                                                kPlanetAxis.z }));
    s.normal = space::toF(nd);

    const float moisture = moistureAt(w, dir);
    const float colourE = landE * sh.amp + sh.orogeny * kOrogenyColourWeight;
    s.color  = biomeColor(w.p, colourE, slope, lat, moisture);

    if (landE > 0.0f) {
        const GroundLayerWeights gw = groundLayerWeights(w.p, colourE, slope, lat,
                                                         moisture, canopyAt(w, dir));
        for (int i = 0; i < kGroundLayerCount; ++i) s.mat[i] = gw.w[i];
    } else {
        for (int i = 0; i < kGroundLayerCount; ++i) s.mat[i] = 0.0f;
    }
    return s;
}

inline Vertex evalVertex(const World& w, int face, double u, double v, double delta,
                         const DVec3& origin, int detailOctaves,
                         const DVec3& coarsePos, const Vec3& coarseNormal) {
    const SurfaceSample s = sampleSurface(w, face, u, v, delta, detailOctaves);
    const Vec3 pf = space::toF(s.pos - origin);
    const Vec3 cf = space::toF(coarsePos - origin);

    auto q8 = [](float w) {
        const float c = w < 0.0f ? 0.0f : (w > 1.0f ? 1.0f : w);
        return static_cast<unsigned char>(c * 255.0f + 0.5f);
    };
    unsigned char mw[8] = {};
    for (int i = 0; i < kGroundLayerCount; ++i) mw[i] = q8(s.mat[i]);
    return makeVertex(pf.x, pf.y, pf.z, s.normal.x, s.normal.y, s.normal.z,
                      s.color.x, s.color.y, s.color.z, cf.x, cf.y, cf.z,
                      coarseNormal.x, coarseNormal.y, coarseNormal.z, mw);
}

struct GridSample {
    DVec3 pos;
    float landE = 0.0f;
    float amp   = 1.0f;

    float orogeny = 0.0f;
};

struct SurfaceGrid {
    int                      side = 0;
    std::vector<GridSample>  s;

    const GridSample& at(int i, int j) const { return s[size_t(j + 1) * side + (i + 1)]; }

    Vec3 normal(int i, int j, const DVec3& dir) const {
        const DVec3 du = at(i + 1, j).pos - at(i - 1, j).pos;
        const DVec3 dv = at(i, j + 1).pos - at(i, j - 1).pos;
        DVec3 n = space::normalize(space::cross(dv, du));
        if (space::dot(n, dir) < 0.0) n = -n;
        return space::toF(n);
    }
};

inline SurfaceGrid sampleSurfaceGrid(const World& w, int face, double u0, double v0,
                                     double step, int N, int detailOctaves) {
    SurfaceGrid g;
    g.side = N + 3;
    g.s.resize(size_t(g.side) * g.side);
    for (int j = -1; j <= N + 1; ++j)
        for (int i = -1; i <= N + 1; ++i) {
            const double u = u0 + step * double(i);
            const double v = v0 + step * double(j);
            const DVec3  dir = faceToSphereD(face, u, v);
            const TerrainShape sh = sampleShape(w, dir);
            GridSample gs;
            gs.pos     = dir * surfaceRadius(w, dir, sh, detailOctaves);
            gs.landE   = sh.landE;
            gs.amp     = sh.amp;
            gs.orogeny = sh.orogeny;
            g.s[size_t(j + 1) * g.side + (i + 1)] = gs;
        }
    return g;
}

inline Vertex gridVertex(const World& w, int face, double u, double v,
                         const GridSample& gs, const Vec3& nrm,
                         const DVec3& origin,
                         const DVec3& coarsePos, const Vec3& coarseNormal) {
    const DVec3 dir = faceToSphereD(face, u, v);
    const float slope = 1.0f - float(space::dot(space::toF(dir), nrm));
    const float lat   = float(space::dot(dir, DVec3{ kPlanetAxis.x, kPlanetAxis.y,
                                                     kPlanetAxis.z }));
    const float moisture = moistureAt(w, dir);

    const float e = gs.landE * gs.amp + gs.orogeny * kOrogenyColourWeight;

    const Vec3 color = biomeColor(w.p, e, slope, lat, moisture);
    float mat[kGroundLayerCount] = {};
    if (gs.landE > 0.0f) {
        const GroundLayerWeights gw = groundLayerWeights(w.p, e, slope, lat, moisture,
                                                         canopyAt(w, dir));
        for (int k = 0; k < kGroundLayerCount; ++k) mat[k] = gw.w[k];
    }

    const Vec3 pf = space::toF(gs.pos - origin);
    const Vec3 cf = space::toF(coarsePos - origin);
    auto q8 = [](float x) {
        const float c = x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x);
        return static_cast<unsigned char>(c * 255.0f + 0.5f);
    };
    unsigned char mw[8] = {};
    for (int k = 0; k < kGroundLayerCount; ++k) mw[k] = q8(mat[k]);
    return makeVertex(pf.x, pf.y, pf.z, nrm.x, nrm.y, nrm.z,
                      color.x, color.y, color.z, cf.x, cf.y, cf.z,
                      coarseNormal.x, coarseNormal.y, coarseNormal.z, mw);
}

struct ParentSurface {
    static constexpr int kSide = kPatchResolution / 2 + 1;
    DVec3 p[kSide * kSide];
    Vec3  n[kSide * kSide];
    bool  valid = false;

    void taps(int i, int j, int& a, int& b) const {
        const int pi = i >> 1, pj = j >> 1;
        const bool oi = (i & 1) != 0, oj = (j & 1) != 0;
        auto id = [](int x, int y) { return y * kSide + x; };
        if (!oi && !oj) { a = b = id(pi, pj);                     return; }
        if ( oi && !oj) { a = id(pi, pj);   b = id(pi + 1, pj);   return; }
        if (!oi &&  oj) { a = id(pi, pj);   b = id(pi, pj + 1);   return; }
        a = id(pi + 1, pj); b = id(pi, pj + 1);
    }

    DVec3 at(int i, int j) const {
        int a = 0, b = 0; taps(i, j, a, b);
        return a == b ? p[a] : (p[a] + p[b]) * 0.5;
    }

    Vec3 normalAt(int i, int j) const {
        int a = 0, b = 0; taps(i, j, a, b);
        return a == b ? n[a] : space::normalize(n[a] + n[b]);
    }
};

inline ParentSurface sampleParentSurface(const World& w, int face, int level,
                                         double u0, double v0, double step, int N) {
    ParentSurface ps;

    if (level <= 0 || N != kPatchResolution) return ps;

    const int    coarseDetail = detailOctavesForLevel(level - 1);
    const double coarseDelta  = step;

    const int side = ParentSurface::kSide;
    const SurfaceGrid pg = sampleSurfaceGrid(w, face, u0, v0, step * 2.0,
                                             side - 1, coarseDetail);
    (void)coarseDelta;
    for (int pj = 0; pj < side; ++pj)
        for (int pi = 0; pi < side; ++pi) {
            const double u = u0 + step * 2.0 * pi;
            const double v = v0 + step * 2.0 * pj;
            ps.p[pj * side + pi] = pg.at(pi, pj).pos;
            ps.n[pj * side + pi] = pg.normal(pi, pj, faceToSphereD(face, u, v));
        }
    ps.valid = true;
    return ps;
}

struct PatchMeshData {
    std::vector<Vertex>   vertices;

    std::vector<uint16_t> indices;
    DVec3                 origin;
};

inline PatchMeshData generatePatch(const World& w, int face, int level,
                                   int ix, int iy, int N) {
    PatchMeshData out;
    const int verts = N + 1;

    const double size  = 2.0 / static_cast<double>(1 << level);
    const double u0    = -1.0 + size * static_cast<double>(ix);
    const double v0    = -1.0 + size * static_cast<double>(iy);
    const double step  = size / static_cast<double>(N);
    const double delta = step * 0.5;
    const DVec3  origin = patchOrigin(face, level, ix, iy);
    const int    detail = detailOctavesForLevel(level);
    out.origin = origin;

    const ParentSurface parent = sampleParentSurface(w, face, level, u0, v0,
                                                     step, N);

    out.vertices.reserve(static_cast<size_t>(verts) * verts + 4u * verts);
    out.indices.reserve(static_cast<size_t>(N) * N * 6u + N * 4u * 6u);

    const SurfaceGrid grid = sampleSurfaceGrid(w, face, u0, v0, step, N, detail);
    (void)delta;

    for (int j = 0; j < verts; ++j)
        for (int i = 0; i < verts; ++i) {
            const double u = u0 + step * static_cast<double>(i);
            const double v = v0 + step * static_cast<double>(j);
            const DVec3  dir = faceToSphereD(face, u, v);
            const GridSample& gs = grid.at(i, j);
            const Vec3 nrm = grid.normal(i, j, dir);

            const DVec3 coarseP = parent.valid ? parent.at(i, j)       : gs.pos;
            const Vec3  coarseN = parent.valid ? parent.normalAt(i, j) : nrm;
            out.vertices.push_back(gridVertex(w, face, u, v, gs, nrm, origin,
                                              coarseP, coarseN));
        }
    auto gid = [verts](int i, int j) { return static_cast<uint16_t>(j * verts + i); };
    for (int j = 0; j < N; ++j)
        for (int i = 0; i < N; ++i) {
            const uint16_t a = gid(i, j),  b = gid(i + 1, j);
            const uint16_t c = gid(i, j+1), d = gid(i + 1, j + 1);
            out.indices.insert(out.indices.end(), { a, c, b,  b, c, d });
        }

    const DVec3 c00 = faceToSphereD(face, u0, v0) * kPlanetRadius;
    const DVec3 c10 = faceToSphereD(face, u0 + size, v0) * kPlanetRadius;
    const double worldEdge = space::length(c10 - c00);

    const double skirtDrop = std::min(0.5 * worldEdge,
                                      w.p.terrainAmplitude + w.p.detailAmplitude);

    auto addSkirt = [&](uint32_t edgeIndex) {
        const Vertex& e = out.vertices[edgeIndex];

        const DVec3 ep = origin + DVec3{ e.px, e.py, e.pz };
        const DVec3 dir = space::normalize(ep);
        const double rNew = space::length(ep) - skirtDrop;
        const Vec3 sp = space::toF(dir * rNew - origin);
        Vertex s = e;

        s.cx = e.cx + (sp.x - e.px);
        s.cy = e.cy + (sp.y - e.py);
        s.cz = e.cz + (sp.z - e.pz);
        s.px = sp.x; s.py = sp.y; s.pz = sp.z;
        out.vertices.push_back(s);
        return static_cast<uint32_t>(out.vertices.size() - 1);
    };

    auto buildEdge = [&](std::vector<uint16_t> border) {
        std::vector<uint16_t> skirt;
        skirt.reserve(border.size());
        for (uint32_t idx : border) skirt.push_back(addSkirt(idx));
        for (size_t k = 0; k + 1 < border.size(); ++k) {
            const uint16_t a = border[k],     b = border[k + 1];
            const uint16_t sa = skirt[k],     sb = skirt[k + 1];

            out.indices.insert(out.indices.end(), { a, sa, b,  b, sa, sb });
        }
    };
    { std::vector<uint16_t> e; for (int i = 0; i < verts; ++i) e.push_back(gid(i, 0));       buildEdge(e); }
    { std::vector<uint16_t> e; for (int i = 0; i < verts; ++i) e.push_back(gid(i, N));       buildEdge(e); }
    { std::vector<uint16_t> e; for (int j = 0; j < verts; ++j) e.push_back(gid(0, j));       buildEdge(e); }
    { std::vector<uint16_t> e; for (int j = 0; j < verts; ++j) e.push_back(gid(N, j));       buildEdge(e); }

    return out;
}

}
