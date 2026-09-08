#pragma once

#include "math/SpaceMath.h"
#include "world/PlanetConfig.h"
#include "world/CubeSphere.h"
#include "math/Mat4.h"
#include <vector>
#include <cstdint>
#include <cmath>
#include <algorithm>
#include <unordered_map>

namespace planet {

struct PatchKey {
    int face, level, ix, iy;
    bool operator==(const PatchKey& o) const {
        return face == o.face && level == o.level && ix == o.ix && iy == o.iy;
    }
};

struct PatchKeyHash {

    std::size_t operator()(const PatchKey& k) const {
        uint64_t h = (static_cast<uint64_t>(k.face)  << 58) |
                     (static_cast<uint64_t>(k.level) << 52) |
                     (static_cast<uint64_t>(static_cast<uint32_t>(k.ix) & 0x3FFFFFFu) << 26) |
                      static_cast<uint64_t>(static_cast<uint32_t>(k.iy) & 0x3FFFFFFu);
        h ^= h >> 30; h *= 0xBF58476D1CE4E5B9ull;
        h ^= h >> 27; h *= 0x94D049BB133111EBull;
        h ^= h >> 31;
        return static_cast<std::size_t>(h);
    }
};

struct PatchBounds {

    DAABB  aabb;
    DVec3  center;
    double worldEdge = 0.0;

    DVec3  surfaceCenter;
    double surfaceRadius = 0.0;
};

inline PatchBounds computePatchBounds(int face, int level, int ix, int iy,
                                      const World& w) {
    const double size = 2.0 / static_cast<double>(1 << level);
    const double u0 = -1.0 + size * static_cast<double>(ix);
    const double v0 = -1.0 + size * static_cast<double>(iy);

    PatchBounds pb;

    const DVec3 c00 = faceToSphereD(face, u0, v0) * kPlanetRadius;
    const DVec3 c10 = faceToSphereD(face, u0 + size, v0) * kPlanetRadius;
    pb.worldEdge = space::length(c10 - c00);

    double finestFeature = static_cast<double>(kPlanetRadius) / w.p.noiseFrequency;
    for (int i = 1; i < w.p.noiseOctaves; ++i) finestFeature *= 0.5;
    const double coarseness = std::min(1.0, pb.worldEdge / finestFeature);

    const double slack = w.p.terrainAmplitude * coarseness + w.p.detailAmplitude +
                         w.p.spireAmplitude + kOrogenyAmplitude;

    const int kS = 4;
    const int kN = (kS + 1) * (kS + 1);
    double hMax = 0.0;
    DVec3  dirs[kN];
    DVec3  surf[kN];
    int    n = 0;
    DAABB  surfBox;
    for (int j = 0; j <= kS; ++j)
        for (int i = 0; i <= kS; ++i) {
            const DVec3 d = faceToSphereD(face,
                                          u0 + size * (double(i) / kS),
                                          v0 + size * (double(j) / kS));

            const TerrainShape sh = sampleShape(w, d);
            const double r = terrainRadius(w.p, sh);
            hMax = std::max(hMax, r - kPlanetRadius);
            dirs[n] = d;

            surf[n] = d * surfaceRadius(w, d, sh, detailOctavesForLevel(level));
            surfBox.expand(surf[n]);
            ++n;
        }

    const double rLo = kPlanetRadius;
    const double rHi = kPlanetRadius + std::min(hMax + slack,
                                                w.p.terrainAmplitude + w.p.detailAmplitude +
                                                w.p.spireAmplitude + kOrogenyAmplitude);
    for (int i = 0; i < n; ++i) {
        pb.aabb.expand(dirs[i] * rLo);
        pb.aabb.expand(dirs[i] * rHi);
    }
    pb.center = pb.aabb.center();

    pb.surfaceCenter = surfBox.center();
    for (int i = 0; i < n; ++i)
        pb.surfaceRadius = std::max(pb.surfaceRadius,
                                    space::length(surf[i] - pb.surfaceCenter));
    return pb;
}

inline bool shouldSplit(const PatchBounds& pb, const DVec3& camera, int level) {
    if (level >= kMaxDepth) return false;
    const double dist = std::max(space::length(camera - pb.surfaceCenter) -
                                 pb.surfaceRadius, 1e-3);
    return pb.worldEdge > dist * double(splitFactorAt(space::length(camera)));
}

inline bool shouldPrefetch(const PatchBounds& pb, const DVec3& camera, int level) {
    if (level >= kMaxDepth) return false;
    const double dist = std::max(space::length(camera - pb.surfaceCenter) -
                                 pb.surfaceRadius - kPrefetchLead, 1e-3);
    return pb.worldEdge > dist * double(splitFactorAt(space::length(camera)));
}

struct SelectedPatch {
    PatchKey key;
    DAABB    aabb;
    double   worldEdge = 0.0;
};

template <class Bounds, class Resident>
void requestSubtree(int face, int level, int ix, int iy, const DVec3& camera,
                    const Bounds& bounds, const Resident& resident,
                    std::vector<PatchKey>& wanted) {
    if (level >= kMaxDepth) return;
    const PatchKey kids[4] = {
        { face, level + 1, ix * 2,     iy * 2     },
        { face, level + 1, ix * 2 + 1, iy * 2     },
        { face, level + 1, ix * 2,     iy * 2 + 1 },
        { face, level + 1, ix * 2 + 1, iy * 2 + 1 },
    };
    for (const PatchKey& k : kids) {
        if (!resident(k)) wanted.push_back(k);

        if (shouldSplit(bounds(k), camera, k.level))
            requestSubtree(k.face, k.level, k.ix, k.iy, camera, bounds,
                           resident, wanted);
    }
}

template <class Bounds, class Resident>
void selectFaceAdaptive(int face, int level, int ix, int iy, const DVec3& camera,
                        const Bounds& bounds, const Resident& resident,
                        std::vector<SelectedPatch>& out,
                        std::vector<PatchKey>& wanted) {
    const PatchBounds& pb = bounds(PatchKey{ face, level, ix, iy });
    const bool split = shouldSplit(pb, camera, level);

    const PatchKey kids[4] = {
        { face, level + 1, ix * 2,     iy * 2     },
        { face, level + 1, ix * 2 + 1, iy * 2     },
        { face, level + 1, ix * 2,     iy * 2 + 1 },
        { face, level + 1, ix * 2 + 1, iy * 2 + 1 },
    };

    if (!split) {
        out.push_back({ { face, level, ix, iy }, pb.aabb, pb.worldEdge });

        if (shouldPrefetch(pb, camera, level))
            for (const PatchKey& k : kids) if (!resident(k)) wanted.push_back(k);
        return;
    }

    bool allResident = true;
    for (const PatchKey& k : kids) if (!resident(k)) allResident = false;

    if (!allResident) {
        out.push_back({ { face, level, ix, iy }, pb.aabb, pb.worldEdge });
        requestSubtree(face, level, ix, iy, camera, bounds, resident, wanted);
        return;
    }

    for (const PatchKey& k : kids)
        selectFaceAdaptive(k.face, k.level, k.ix, k.iy, camera, bounds, resident,
                           out, wanted);
}

template <class Bounds, class Resident>
std::vector<SelectedPatch> selectLODAdaptive(const DVec3& camera,
                                             const Bounds& bounds,
                                             const Resident& resident,
                                             std::vector<PatchKey>& wanted) {
    std::vector<SelectedPatch> out;
    for (int f = 0; f < 6; ++f)
        selectFaceAdaptive(f, 0, 0, 0, camera, bounds, resident, out, wanted);
    return out;
}

class PatchBoundsCache {
public:
    explicit PatchBoundsCache(const World& w) : w_(&w) {}

    void reset(const World& w) { w_ = &w; map_.clear(); }

    void repoint(const World& w) { w_ = &w; }

    const PatchBounds& operator()(const PatchKey& k) const {
        auto it = map_.find(k);
        if (it != map_.end()) return it->second;

        if (map_.size() >= kCap) map_.clear();
        return map_.emplace(k, computePatchBounds(k.face, k.level, k.ix, k.iy, *w_))
                   .first->second;
    }

private:
    static constexpr std::size_t kCap = 200000;
    const World* w_;
    mutable std::unordered_map<PatchKey, PatchBounds, PatchKeyHash> map_;
};

inline std::vector<PatchKey> selectLOD(const DVec3& camera, const World& w) {
    PatchBoundsCache bounds(w);
    std::vector<PatchKey> wanted;
    const std::vector<SelectedPatch> sel = selectLODAdaptive(
        camera, bounds, [](const PatchKey&) { return true; }, wanted);
    std::vector<PatchKey> out;
    out.reserve(sel.size());
    for (const SelectedPatch& s : sel) out.push_back(s.key);
    return out;
}

}
