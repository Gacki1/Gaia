#pragma once

#include <cstdint>
#include <vector>

namespace planet {

inline constexpr int kFontFirstChar = 32;
inline constexpr int kFontLastChar  = 126;
inline constexpr int kFontCharCount = kFontLastChar - kFontFirstChar + 1;

struct FontAtlas {

    std::vector<uint8_t> pixels;
    uint32_t width  = 0;
    uint32_t height = 0;

    uint32_t cellW  = 0;
    uint32_t cellH  = 0;
    uint32_t cols   = 0;
    int      ascent = 0;

    float whiteU = 0.0f;
    float whiteV = 0.0f;

    bool ok() const { return width > 0 && height > 0 && !pixels.empty(); }
};

FontAtlas buildFontAtlas(const char* face = "Consolas", int pixelHeight = 16);

}
