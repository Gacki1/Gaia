#pragma once

#include "platform/ImageLoad.h"
#include "world/GroundMaterials.h"
#include "world/GroundTextures.h"

#include <string>
#include <vector>
#include <cstdio>
#include <cmath>
#include <cstdint>

#ifndef PLANET_SOURCE_DIR
#define PLANET_SOURCE_DIR "."
#endif

namespace planet {

inline constexpr uint32_t kGroundTexLoadSize = 1024;

inline double srgbToLinear(uint8_t b) {
    const double s = double(b) / 255.0;
    return s <= 0.04045 ? s / 12.92 : std::pow((s + 0.055) / 1.055, 2.4);
}

struct GroundSet {
    std::vector<uint8_t> rgba;
    uint32_t size    = 0;
    uint32_t layers  = 0;
    int      loaded  = 0;

    std::vector<uint8_t> normalRG;
    int normalsLoaded = 0;

    float mean[kGroundLayerCount] = {};
};

namespace groundtex {

inline std::vector<std::string> searchRoots() {
    return { "textures", std::string(PLANET_SOURCE_DIR) + "/src/textures" };
}

std::string findFolder(const std::string& token);

std::string findMap(const std::string& dir, const char* token);

}

inline GroundSet loadGroundSet(uint32_t size = kGroundTexLoadSize) {
    GroundSet set;
    set.size   = size;
    set.layers = kGroundLayerCount;
    set.rgba.assign(size_t(size) * size * 4 * kGroundLayerCount, 0);

    set.normalRG.assign(size_t(size) * size * 2 * kGroundLayerCount, 128);

    const std::vector<uint8_t> fallback = buildGroundTextures();

    for (int m = 0; m < kGroundLayerCount; ++m) {
        uint8_t* dst = &set.rgba[size_t(m) * size * size * 4];
        const std::string dir = groundtex::findFolder(kGroundLayers[m].token);

        LoadedImage base, ao, nrm;
        if (!dir.empty()) {
            const std::string bp = groundtex::findMap(dir, "basecolor");
            if (!bp.empty()) base = loadImageRGBA(bp, size);
            const std::string ap = groundtex::findMap(dir, "_ao");
            if (!ap.empty()) ao = loadImageRGBA(ap, size);
            const std::string np = groundtex::findMap(dir, "_normal");
            if (!np.empty()) nrm = loadImageRGBA(np, size);
        }

        if (nrm.ok() && nrm.width == size && nrm.height == size) {
            uint8_t* dn = &set.normalRG[size_t(m) * size * size * 2];
            for (size_t i = 0; i < size_t(size) * size; ++i) {
                dn[i * 2 + 0] = nrm.rgba[i * 4 + 0];

                dn[i * 2 + 1] = nrm.rgba[i * 4 + 1];
            }
            ++set.normalsLoaded;
        }

        if (base.ok() && base.width == size && base.height == size) {
            const size_t px = size_t(size) * size;

            double lum = 0.0;
            for (size_t i = 0; i < px; ++i)
                lum += 0.2126 * srgbToLinear(base.rgba[i * 4 + 0]) +
                       0.7152 * srgbToLinear(base.rgba[i * 4 + 1]) +
                       0.0722 * srgbToLinear(base.rgba[i * 4 + 2]);
            lum /= double(px);

            set.mean[m] = float(lum > 1e-6 ? lum : 1.0);

            for (size_t i = 0; i < px; ++i) {
                dst[i * 4 + 0] = base.rgba[i * 4 + 0];
                dst[i * 4 + 1] = base.rgba[i * 4 + 1];
                dst[i * 4 + 2] = base.rgba[i * 4 + 2];

                dst[i * 4 + 3] = (ao.ok() && ao.width == size) ? ao.rgba[i * 4] : 255;
            }
            ++set.loaded;
            std::printf("  ground layer %d (%-18s) <- %s%s%s  (linear mean %.3f)\n",
                        m, kGroundLayers[m].label, dir.c_str(),
                        ao.ok() ? "  +AO" : "", nrm.ok() ? "  +N" : "", lum);
        } else {

            const int fs = kGroundTexSize;
            const uint8_t* src = &fallback[size_t(m % kGroundTexLayers) * fs * fs * 4];
            for (uint32_t y = 0; y < size; ++y)
                for (uint32_t x = 0; x < size; ++x) {
                    const uint32_t sx = x * fs / size, sy = y * fs / size;
                    const uint8_t* s = &src[(size_t(sy) * fs + sx) * 4];
                    uint8_t* d = &dst[(size_t(y) * size + x) * 4];
                    d[0] = s[0]; d[1] = s[1]; d[2] = s[2]; d[3] = 255;
                }

            {
                double lum = 0.0;
                for (size_t i = 0; i < size_t(size) * size; ++i)
                    lum += 0.2126 * srgbToLinear(dst[i * 4 + 0]) +
                           0.7152 * srgbToLinear(dst[i * 4 + 1]) +
                           0.0722 * srgbToLinear(dst[i * 4 + 2]);
                lum /= double(size) * size;
                set.mean[m] = float(lum > 1e-6 ? lum : 1.0);
            }
            std::printf("  ground layer %d (%-18s) <- generated placeholder\n",
                        m, kGroundLayers[m].label);
        }
    }
    return set;
}

}
