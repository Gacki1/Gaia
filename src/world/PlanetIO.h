#pragma once

#include "world/PlanetFields.h"

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace planet {

inline std::string writePlanet(const PlanetParams& p, const std::string& name = "") {
    std::string out;
    out += "# planet";
    if (!name.empty()) { out += " \""; out += name; out += "\""; }
    out += "\n# written by the in-game editor -- see src/world/PlanetFields.h\n\n";

    const char* group = nullptr;
    char line[192];
    for (const ParamField& f : kParamFields) {
        if (!group || std::string(group) != f.group) {
            group = f.group;
            out += "\n# --- ";
            out += group;
            out += "\n";
        }
        const double v = fieldGet(p, f);

        std::snprintf(line, sizeof(line), "%-24s %.9g\n", f.key, v);
        out += line;
    }

    out += "\n# --- biome factors (amp / detail / ridge / terrace, each in (0,1])\n";
    for (int b = 0; b < kBiomeCount; ++b)
        for (int k = 0; k < 4; ++k) {
            std::snprintf(line, sizeof(line), "%-24s %.9g\n",
                          (std::string("biome.") + kBiomeNames[b] + "." +
                           kBiomeFactorNames[k]).c_str(),
                          double(biomeFactor(p, b, k)));
            out += line;
        }
    return out;
}

struct ReadReport {
    int                      applied = 0;
    std::vector<std::string> unknown;
};

inline ReadReport readPlanet(const std::string& text, PlanetParams& out) {
    out = PlanetParams{};
    ReadReport rep;

    size_t i = 0;
    while (i < text.size()) {
        size_t e = text.find('\n', i);
        if (e == std::string::npos) e = text.size();
        std::string line = text.substr(i, e - i);
        i = e + 1;

        const size_t hash = line.find('#');
        if (hash != std::string::npos) line = line.substr(0, hash);

        size_t a = line.find_first_not_of(" \t\r");
        if (a == std::string::npos) continue;
        size_t b = line.find_last_not_of(" \t\r");
        line = line.substr(a, b - a + 1);
        if (line.empty()) continue;

        const size_t sp = line.find_first_of(" \t");
        if (sp == std::string::npos) { rep.unknown.push_back(line); continue; }
        const std::string key = line.substr(0, sp);
        const std::string val = line.substr(line.find_first_not_of(" \t", sp));
        const double v = std::strtod(val.c_str(), nullptr);

        bool hit = false;
        for (const ParamField& f : kParamFields)
            if (key == f.key) { fieldSet(out, f, v); ++rep.applied; hit = true; break; }
        if (hit) continue;

        if (key.compare(0, 6, "biome.") == 0) {
            for (int bi = 0; bi < kBiomeCount && !hit; ++bi)
                for (int k = 0; k < 4 && !hit; ++k) {
                    const std::string want = std::string("biome.") + kBiomeNames[bi] +
                                             "." + kBiomeFactorNames[k];
                    if (key == want) {
                        biomeFactor(out, bi, k) = float(v);
                        ++rep.applied;
                        hit = true;
                    }
                }
        }
        if (!hit) rep.unknown.push_back(key);
    }
    return rep;
}

}
