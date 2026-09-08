#pragma once

#include "platform/FontAtlas.h"

#include <cstdint>
#include <string>
#include <vector>

namespace planet {

struct OverlayVertex {
    float x, y;
    float u, v;
    uint32_t rgba;
};

inline uint32_t overlayColor(uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255) {
    return uint32_t(r) | (uint32_t(g) << 8) | (uint32_t(b) << 16) | (uint32_t(a) << 24);
}

inline constexpr uint32_t kUiText    = 0xFFD6D6D6u;
inline constexpr uint32_t kUiDim     = 0xFF8C8C8Cu;
inline constexpr uint32_t kUiValue   = 0xFFDCDCDCu;
inline constexpr uint32_t kUiAccent  = 0xFFB08050u;
inline constexpr uint32_t kUiWarn    = 0xFF5C5CD0u;
inline constexpr uint32_t kUiPanel   = 0xF01F1F1Fu;
inline constexpr uint32_t kUiTrack   = 0xFF343434u;
inline constexpr uint32_t kUiFill    = 0xFF6E6E6Eu;
inline constexpr uint32_t kUiHilite  = 0xFF2F2F2Fu;
inline constexpr uint32_t kUiLine    = 0xFF2C2C2Cu;

class OverlayDraw {
public:
    explicit OverlayDraw(const FontAtlas& atlas) : atlas_(&atlas) {}

    void clear() { verts_.clear(); }

    void rect(float x, float y, float w, float h, uint32_t color) {
        quad(x, y, w, h, atlas_->whiteU, atlas_->whiteV, 0.0f, 0.0f, color);
    }

    float text(float x, float y, const std::string& s, uint32_t color) {
        const float cw = float(atlas_->cellW), ch = float(atlas_->cellH);
        const float du = cw / float(atlas_->width);
        const float dv = ch / float(atlas_->height);
        float px = x;
        for (char raw : s) {
            const int c = int(static_cast<unsigned char>(raw));

            if (c >= kFontFirstChar && c <= kFontLastChar && c != ' ') {
                const int idx = c - kFontFirstChar;
                const float u0 = float(uint32_t(idx) % atlas_->cols) * du;
                const float v0 = float(uint32_t(idx) / atlas_->cols) * dv;
                quad(px, y, cw, ch, u0, v0, du, dv, color);
            }
            px += cw;
        }
        return px;
    }

    void patchRectHeight(size_t mark, float h) {
        if (mark + 6 > verts_.size()) return;
        const float y0 = verts_[mark].y;
        for (size_t i = mark; i < mark + 6; ++i)
            if (verts_[i].y > y0) verts_[i].y = y0 + h;
    }

    float textWidth(size_t chars) const { return float(chars) * float(atlas_->cellW); }
    float lineHeight() const { return float(atlas_->cellH); }

    const std::vector<OverlayVertex>& vertices() const { return verts_; }
    bool empty() const { return verts_.empty(); }

private:

    void quad(float x, float y, float w, float h,
              float u, float v, float du, float dv, uint32_t c) {
        const OverlayVertex a{ x,     y,     u,      v,      c };
        const OverlayVertex b{ x + w, y,     u + du, v,      c };
        const OverlayVertex d{ x + w, y + h, u + du, v + dv, c };
        const OverlayVertex e{ x,     y + h, u,      v + dv, c };
        verts_.push_back(a); verts_.push_back(b); verts_.push_back(d);
        verts_.push_back(a); verts_.push_back(d); verts_.push_back(e);
    }

    const FontAtlas*           atlas_;
    std::vector<OverlayVertex> verts_;
};

}
