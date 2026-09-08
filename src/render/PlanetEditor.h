#pragma once

#include "render/OverlayUi.h"
#include "world/PlanetFields.h"
#include "world/PlanetValidate.h"

#include <algorithm>
#include <cstring>
#include <string>
#include <vector>

namespace planet {

struct EditorActions {
    bool applyTerrain = false;
    bool save         = false;
    bool load         = false;
    bool reset        = false;
};

class PlanetEditor {
public:

    EditorActions draw(Ui& ui, OverlayDraw& d, const UiInput& in,
                       PlanetParams& edited, const PlanetParams& live,
                       float viewW, float viewH) {
        EditorActions act;
        const float rowH = ui.rowH();

        scroll_ -= in.wheel * rowH * 3.0f;

        const float panelW = 560.0f;
        const float panelX = 16.0f;
        const float panelY = 16.0f;
        const float bodyTop = panelY + rowH * 4.0f;
        const float bodyBottom = viewH - 16.0f - rowH * 2.0f;

        const float visibleRows = std::max(4.0f, (bodyBottom - bodyTop) / rowH - 2.0f);

        const std::vector<Violation> problems = validate(edited);
        const bool blocked = hasErrors(problems);
        const ParamChange change = classifyChange(live, edited);

        const float panelH = bodyBottom - panelY + rowH * 3.0f;
        d.rect(panelX - 1.0f, panelY - 1.0f, panelW + 2.0f, panelH + 2.0f, kUiLine);
        d.rect(panelX, panelY, panelW, panelH, kUiPanel);
        d.rect(panelX, panelY, panelW, rowH, kUiHilite);

        float y = panelY + kUiRowPad;
        d.text(panelX + kUiPanelPad, y, "gaia", kUiText);
        d.text(panelX + panelW - kUiPanelPad - d.textWidth(2), y, "F4", kUiDim);
        y += rowH;

        {
            std::string status;
            uint32_t col = kUiDim;
            if (blocked) {
                status = "invalid - see the marked rows";
                col = kUiWarn;
            } else if (change == ParamChange::Terrain) {
                status = dragging_ ? "rebuild on release (~80 ms)" : "rebuilding";
            } else if (change == ParamChange::ShadingOnly) {
                status = "shading - live";
            } else {
                status = "in sync";
            }
            d.text(panelX + kUiPanelPad, y, status, col);

            char pos[48];
            std::snprintf(pos, sizeof(pos), "%d-%d/%d", firstShown_ + 1,
                          lastShown_, totalRows_);
            d.text(panelX + panelW - kUiPanelPad - d.textWidth(std::strlen(pos)),
                   y, pos, kUiDim);
            y += rowH;
        }

        const int total = kParamFieldCount + kBiomeCount * 4;
        const float maxScroll = std::max(0.0f, (float(total) + 6.0f - visibleRows) * rowH);
        scroll_ = std::max(0.0f, std::min(scroll_, maxScroll));
        const int first = int(scroll_ / rowH);
        const int last  = std::min(total, first + int(visibleRows));
        firstShown_ = first; lastShown_ = last; totalRows_ = total;

        ui.beginPanelAt(panelX, panelW, bodyTop);
        const char* group = nullptr;
        for (int i = first; i < last; ++i) {
            if (i < kParamFieldCount) {
                const ParamField& f = kParamFields[i];
                if (!group || std::string(group) != f.group) {
                    group = f.group;
                    ui.heading(group);
                }

                float fv = float(fieldGet(edited, f));
                const void* id = reinterpret_cast<const char*>(&edited) + f.offset;
                const bool isInt = f.kind == ParamField::Kind::Int ||
                                   f.kind == ParamField::Kind::Uint;
                if (ui.slider(f.key, &fv, float(f.lo), float(f.hi),
                              isInt ? "%.0f" : "%.4g", id))
                    fieldSet(edited, f, double(fv));
                markIfBlamed(ui, d, problems, f.key);
            } else {
                const int bi = (i - kParamFieldCount) / 4;
                const int k  = (i - kParamFieldCount) % 4;
                if (k == 0) ui.heading(std::string("biome / ") + kBiomeNames[bi]);
                float& fv = biomeFactor(edited, bi, k);
                ui.slider(std::string(kBiomeNames[bi]) + "." + kBiomeFactorNames[k],
                          &fv, 0.0f, 1.0f, "%.3f");
                markIfBlamed(ui, d, problems, "biomeScales");
            }
        }

        int shown = 0;
        for (const Violation& p : problems) {
            if (shown++ >= 2) break;
            if (shown == 1) ui.heading(blocked ? "errors" : "warnings");
            const std::string line = std::string(p.field) + ": " + p.message;
            ui.label(line.substr(0, 64),
                     p.sev == Severity::Error ? kUiWarn : kUiDim);
            if (line.size() > 64) ui.label("  " + line.substr(64, 62), kUiDim);
        }

        ui.heading("file");
        act.save  = ui.button("save");
        act.load  = ui.button("load", false);
        act.reset = ui.button("reset", false);
        ui.label("wheel scrolls | drag, or hover and press LEFT/RIGHT", kUiDim);
        ui.endPanelNoBackground();

        dragging_ = ui.capturing();

        if (!dragging_ && change == ParamChange::Terrain && !blocked)
            act.applyTerrain = true;

        return act;
    }

    int firstRow() const { return firstShown_; }
    int rowCount() const { return totalRows_; }

private:

    void markIfBlamed(Ui& ui, OverlayDraw& d,
                      const std::vector<Violation>& problems, const char* key) {
        for (const Violation& p : problems)
            if (std::string(p.field) == key) {
                d.rect(10.0f, ui.cursorY() - ui.rowH(), 3.0f, ui.rowH() - 2.0f,
                       p.sev == Severity::Error ? kUiWarn : kUiValue);
                return;
            }
    }

    float scroll_   = 0.0f;
    bool  dragging_ = false;
    int   firstShown_ = 0, lastShown_ = 0, totalRows_ = 0;
};

}
