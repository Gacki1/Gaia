#pragma once

#include "render/OverlayDraw.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace planet {

struct UiInput {
    float mx = -1.0f, my = -1.0f;
    bool  down     = false;
    bool  pressed  = false;
    float wheel    = 0.0f;
    int   nudge    = 0;
    bool  coarse   = false;
};

inline constexpr float kUiRowPad    = 2.0f;
inline constexpr float kUiPanelPad  = 10.0f;
inline constexpr float kUiLabelCols = 23.0f;
inline constexpr float kUiValueCols = 11.0f;

struct UiState {
    const void* active = nullptr;
};

class Ui {
public:
    Ui(OverlayDraw& draw, const FontAtlas& font, UiState& state)
        : d_(&draw), f_(&font), s_(&state) {}

    static float trackStart(const FontAtlas& f, float panelX) {
        return panelX + kUiPanelPad + float(f.cellW) * kUiLabelCols;
    }
    static float trackWidth(const FontAtlas& f, float panelX, float panelW) {
        const float valueW = float(f.cellW) * kUiValueCols;
        return std::max(40.0f, panelX + panelW - kUiPanelPad - valueW -
                               trackStart(f, panelX) - 6.0f);
    }

    void begin(const UiInput& in) {
        in_ = in;

        if (!in_.down) s_->active = nullptr;
        hovered_ = nullptr;
    }

    void beginPanel(float x, float y, float width, const std::string& title) {
        px_ = x; py_ = y; pw_ = width;
        cursorY_ = y + kUiPanelPad;

        panelVertexMark_ = d_->vertices().size();
        d_->rect(x, y, width, 1.0f, kUiPanel);
        if (!title.empty()) {
            d_->text(x + kUiPanelPad, cursorY_, title, kUiText);
            cursorY_ += rowH();
        }
    }

    void beginPanelAt(float x, float width, float y) {
        px_ = x; py_ = y; pw_ = width;
        cursorY_ = y;
        panelVertexMark_ = kNoPanel;
    }
    void endPanelNoBackground() { panelVertexMark_ = kNoPanel; }

    void endPanel() {
        if (panelVertexMark_ == kNoPanel) return;
        const float h = cursorY_ + kUiPanelPad - py_;
        d_->patchRectHeight(panelVertexMark_, h);
    }

    void label(const std::string& text, uint32_t color = kUiDim) {
        d_->text(px_ + kUiPanelPad, cursorY_, text, color);
        cursorY_ += rowH();
    }

    void heading(const std::string& text) {
        cursorY_ += kUiRowPad * 2.0f;
        const float y = cursorY_ + float(f_->cellH) * 0.5f;
        d_->rect(px_ + kUiPanelPad, y, pw_ - kUiPanelPad * 2.0f, 1.0f, kUiLine);
        const float w = d_->textWidth(text.size() + 2);
        d_->rect(px_ + kUiPanelPad, cursorY_, w, float(f_->cellH), kUiPanel);
        d_->text(px_ + kUiPanelPad, cursorY_, text, kUiDim);
        cursorY_ += rowH() + kUiRowPad;
    }

    bool slider(const std::string& name, float* v, float lo, float hi,
                const char* fmt = "%.3f", const void* id = nullptr) {
        if (!id) id = v;
        const float y = cursorY_;
        cursorY_ += rowH();

        const float labelX = px_ + kUiPanelPad;
        const float trackX = trackStart(*f_, px_);
        const float trackW = trackWidth(*f_, px_, pw_);

        const float trackH = 3.0f;
        const float trackY = y + float(f_->cellH) * 0.5f - trackH * 0.5f;

        const bool over = hit(trackX, y, trackW + 1.0f, rowH());
        if (over) hovered_ = id;
        if (over && in_.pressed) s_->active = id;

        const float before = *v;
        if (s_->active == id && hi > lo) {

            float t = (in_.mx - trackX) / trackW;
            t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
            *v = lo + t * (hi - lo);
        }

        if (over && in_.nudge != 0) {
            const float step = (hi - lo) * (in_.coarse ? 0.05f : 0.005f);
            *v += float(in_.nudge) * step;
        }
        if (*v < lo) *v = lo;
        if (*v > hi) *v = hi;

        const float t = hi > lo ? (*v - lo) / (hi - lo) : 0.0f;

        if (over) d_->rect(px_, y, pw_, rowH(), kUiHilite);
        d_->text(labelX, y, name, kUiText);
        d_->rect(trackX, trackY, trackW, trackH, kUiTrack);
        d_->rect(trackX, trackY, trackW * t, trackH,
                 (s_->active == id) ? kUiAccent : kUiFill);

        char buf[64];
        std::snprintf(buf, sizeof(buf), fmt, double(*v));
        std::string sv(buf);
        if (sv.size() > size_t(kUiValueCols)) sv = sv.substr(0, size_t(kUiValueCols));
        const float valueX = px_ + pw_ - kUiPanelPad - d_->textWidth(sv.size());
        d_->text(valueX, y, sv, kUiValue);
        return *v != before;
    }

    bool sliderInt(const std::string& name, int* v, int lo, int hi) {
        float f = float(*v);
        const int before = *v;

        slider(name, &f, float(lo), float(hi), "%.0f", v);
        *v = int(std::lround(f));
        if (*v < lo) *v = lo;
        if (*v > hi) *v = hi;
        return *v != before;
    }

    bool checkbox(const std::string& name, bool* v) {
        const float y = cursorY_;
        cursorY_ += rowH();
        const float boxX = px_ + kUiPanelPad + d_->textWidth(size_t(kUiLabelCols));
        const float s = rowH() - 6.0f;
        const bool over = hit(boxX, y, s + d_->textWidth(4), rowH());
        if (over && in_.pressed) { *v = !*v; }
        d_->text(px_ + kUiPanelPad, y, name, over ? kUiText : kUiDim);
        d_->rect(boxX, y + 3.0f, s, s, kUiTrack);
        if (*v) d_->rect(boxX + 2.0f, y + 5.0f, s - 4.0f, s - 4.0f, kUiFill);
        return over && in_.pressed;
    }

    bool button(const std::string& text, bool newRow = true) {
        if (newRow) { buttonX_ = px_ + kUiPanelPad; buttonY_ = cursorY_; cursorY_ += rowH(); }
        const float w = d_->textWidth(text.size() + 2);
        const bool over = hit(buttonX_, buttonY_, w, rowH());
        d_->rect(buttonX_, buttonY_, w, float(f_->cellH), over ? kUiFill : kUiTrack);
        d_->text(buttonX_ + d_->textWidth(1), buttonY_, text, kUiText);
        buttonX_ += w + d_->textWidth(1);
        return over && in_.pressed;
    }

    float cursorY() const { return cursorY_; }
    float rowH() const { return float(f_->cellH) + kUiRowPad; }

    bool capturing() const { return s_->active != nullptr; }

private:
    bool hit(float x, float y, float w, float h) const {
        return in_.mx >= x && in_.mx < x + w && in_.my >= y && in_.my < y + h;
    }

    OverlayDraw*     d_;
    const FontAtlas* f_;
    UiState*         s_;
    UiInput          in_;
    const void*      hovered_ = nullptr;
    float px_ = 0, py_ = 0, pw_ = 0, cursorY_ = 0;
    float buttonX_ = 0, buttonY_ = 0;
    static constexpr size_t kNoPanel = ~size_t(0);
    size_t panelVertexMark_ = kNoPanel;
};

}
