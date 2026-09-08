#pragma once

#include "math/SpaceMath.h"
#include "render/Vertex.h"
#include "ship/ShipConfig.h"
#include "ship/Cockpit.h"
#include <vector>
#include <cstdint>
#include <cmath>

namespace planet {

struct ShipMeshData {
    std::vector<Vertex>   vertices;
    std::vector<uint16_t> indices;
};

namespace shipmesh {

using space::Vec3;

inline const Vec3 kColHull    { 0.45f, 0.47f, 0.50f };
inline const Vec3 kColAccent  { 0.60f, 0.25f, 0.15f };
inline const Vec3 kColCanopy  { 0.08f, 0.12f, 0.20f };
inline const Vec3 kColNozzle  { 0.15f, 0.15f, 0.16f };
inline const Vec3 kColGlow    { 1.00f, 0.45f, 0.15f };
inline const Vec3 kColGear    { 0.30f, 0.31f, 0.33f };

inline const Vec3 kColPanel   { 0.34f, 0.36f, 0.40f };
inline const Vec3 kColConsole { 0.22f, 0.25f, 0.30f };
inline const Vec3 kColSeat    { 0.26f, 0.20f, 0.18f };
inline const Vec3 kColSwitch  { 0.18f, 0.19f, 0.22f };
inline const Vec3 kColSwitchCap{ 0.72f, 0.66f, 0.30f };
inline const Vec3 kColLamp    { 0.55f, 0.80f, 0.60f };

inline const Vec3 kColCrystal    { 0.30f, 0.44f, 0.50f };
inline const Vec3 kColCrystalTip { 0.46f, 0.62f, 0.66f };

inline const Vec3 kColOutpost { 0.52f, 0.46f, 0.36f };
inline const Vec3 kColPad     { 0.24f, 0.25f, 0.27f };
inline const Vec3 kColBeacon  { 1.90f, 1.20f, 0.35f };

inline void addTri(ShipMeshData& m, const Vec3& a, const Vec3& b, const Vec3& c,
                   const Vec3& col) {
    Vec3 n = space::cross(b - a, c - a);
    const float len = space::length(n);
    if (len < 1e-12f) return;
    n = n * (1.0f / len);
    const uint32_t base = static_cast<uint32_t>(m.vertices.size());

    static const unsigned char kNoGroundMaterial[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    for (const Vec3& p : { a, b, c })

        m.vertices.push_back(makeVertex(p.x, p.y, p.z, n.x, n.y, n.z,
                                        col.x, col.y, col.z, p.x, p.y, p.z,
                                        n.x, n.y, n.z, kNoGroundMaterial));
    m.indices.push_back(static_cast<uint16_t>(base));
    m.indices.push_back(static_cast<uint16_t>(base + 1));
    m.indices.push_back(static_cast<uint16_t>(base + 2));
}

inline void addQuad(ShipMeshData& m, const Vec3& a, const Vec3& b, const Vec3& c,
                    const Vec3& d, const Vec3& col) {
    addTri(m, a, b, c, col);
    addTri(m, a, c, d, col);
}

inline void addBox(ShipMeshData& m, const Vec3& lo, const Vec3& hi, const Vec3& col) {
    const Vec3 a{ lo.x, lo.y, lo.z }, b{ hi.x, lo.y, lo.z };
    const Vec3 c{ hi.x, hi.y, lo.z }, d{ lo.x, hi.y, lo.z };
    const Vec3 e{ lo.x, lo.y, hi.z }, f{ hi.x, lo.y, hi.z };
    const Vec3 g{ hi.x, hi.y, hi.z }, h{ lo.x, hi.y, hi.z };
    addQuad(m, b, a, d, c, col);
    addQuad(m, e, f, g, h, col);
    addQuad(m, a, e, h, d, col);
    addQuad(m, f, b, c, g, col);
    addQuad(m, a, b, f, e, col);
    addQuad(m, d, h, g, c, col);
}

struct Ring { float z, halfW, halfH; };

inline void ringPoints(const Ring& r, Vec3 out[8]) {
    for (int i = 0; i < 8; ++i) {
        const float a = (static_cast<float>(i) + 0.5f) * (2.0f * space::kPi / 8.0f);
        out[i] = Vec3{ std::cos(a) * r.halfW, std::sin(a) * r.halfH, r.z };
    }
}

inline void addLoft(ShipMeshData& m, const Ring& a, const Ring& b, const Vec3& col) {
    Vec3 pa[8], pb[8];
    ringPoints(a, pa);
    ringPoints(b, pb);
    for (int i = 0; i < 8; ++i) {
        const int j = (i + 1) & 7;
        addQuad(m, pa[i], pa[j], pb[j], pb[i], col);
    }
}

inline void addCap(ShipMeshData& m, const Ring& r, bool towardPlusZ, const Vec3& col) {
    Vec3 p[8];
    ringPoints(r, p);
    const Vec3 centre{ 0.0f, 0.0f, r.z };
    for (int i = 0; i < 8; ++i) {
        const int j = (i + 1) & 7;
        if (towardPlusZ) addTri(m, p[i], p[j], centre, col);
        else             addTri(m, p[j], p[i], centre, col);
    }
}

}

inline ShipMeshData buildShipHull() {
    using namespace shipmesh;
    ShipMeshData m;

    const Ring rings[] = {
        { -10.0f, 1.6f, 1.3f },
        {  -6.0f, 2.4f, 2.0f },
        {  -1.0f, 3.0f, 2.4f },
        {   3.0f, 2.8f, 2.1f },
        {   7.0f, 1.8f, 1.3f },
        {  10.0f, 0.7f, 0.6f },
    };
    const int nRings = static_cast<int>(sizeof(rings) / sizeof(rings[0]));
    for (int i = 0; i + 1 < nRings; ++i) addLoft(m, rings[i], rings[i + 1], kColHull);
    addCap(m, rings[0], false, kColHull);
    addCap(m, rings[nRings - 1], true, kColHull);

    for (int side = -1; side <= 1; side += 2) {
        const float xi = 2.6f * side, xo = 6.0f * side;
        const Vec3 ri{ xi, 0.3f, 3.0f },  rif{ xi, 0.3f, -4.0f };
        const Vec3 ro{ xo, 0.1f, 0.5f },  rof{ xo, 0.1f, -3.0f };
        const Vec3 bi{ xi, -0.5f, 3.0f }, bif{ xi, -0.5f, -4.0f };
        const Vec3 bo{ xo, -0.1f, 0.5f }, bof{ xo, -0.1f, -3.0f };
        if (side > 0) {
            addQuad(m, ri, ro, rof, rif, kColHull);
            addQuad(m, bi, bif, bof, bo, kColHull);
            addQuad(m, ro, bo, bof, rof, kColAccent);
            addQuad(m, ri, rif, bif, bi, kColHull);
            addQuad(m, ri, bi, bo, ro, kColAccent);
            addQuad(m, rif, rof, bof, bif, kColHull);
        } else {
            addQuad(m, ri, rif, rof, ro, kColHull);
            addQuad(m, bi, bo, bof, bif, kColHull);
            addQuad(m, ro, rof, bof, bo, kColAccent);
            addQuad(m, ri, bi, bif, rif, kColHull);
            addQuad(m, ri, ro, bo, bi, kColAccent);
            addQuad(m, rif, bif, bof, rof, kColHull);
        }
    }

    {
        const float z0 = 3.4f, z1 = 7.6f, hw = 1.35f, y0 = 1.1f, y1 = 2.35f;
        const Vec3 fl{ -0.7f, y0 + 0.15f, z1 }, fr{ 0.7f, y0 + 0.15f, z1 };
        const Vec3 bl{ -hw,   y0,         z0 }, br{ hw,   y0,         z0 };
        const Vec3 tl{ -0.8f, y1,         z0 + 1.1f }, tr{ 0.8f, y1, z0 + 1.1f };
        addQuad(m, fl, fr, tr, tl, kColCanopy);
        addQuad(m, tl, tr, br, bl, kColCanopy);
        addTri(m, fl, tl, bl, kColCanopy);
        addTri(m, fr, br, tr, kColCanopy);
    }

    for (int side = -1; side <= 1; side += 2) {
        const float x = 3.4f * side;
        addBox(m, Vec3{ x - 1.1f, -1.0f, -10.5f }, Vec3{ x + 1.1f, 1.0f, -4.5f },
               kColNozzle);

        addQuad(m, Vec3{ x - 0.8f, -0.7f, -10.6f }, Vec3{ x + 0.8f, -0.7f, -10.6f },
                   Vec3{ x + 0.8f,  0.7f, -10.6f }, Vec3{ x - 0.8f,  0.7f, -10.6f },
                kColGlow);
    }

    return m;
}

inline ShipMeshData buildCockpitInterior() {
    using namespace shipmesh;
    ShipMeshData m;

    addBox(m, Vec3{ -1.05f, 0.05f, 3.20f }, Vec3{ 1.05f, 0.25f, 6.90f }, kColPanel);
    addBox(m, Vec3{ -1.05f, 0.05f, 3.90f }, Vec3{ 1.05f, 1.45f, 4.10f }, kColPanel);

    addBox(m, Vec3{ -kConsoleHalfX, 0.30f,        kConsoleZNear },
              Vec3{  kConsoleHalfX, kConsoleTopY, kConsoleZFar  }, kColConsole);
    addBox(m, Vec3{ -kConsoleHalfX, kConsoleTopY, kCoamingZNear },
              Vec3{  kConsoleHalfX, kCoamingTopY, kConsoleZFar  }, kColHull);

    for (int side = -1; side <= 1; side += 2) {
        const float xi = 0.80f * side, xo = 1.10f * side;
        addBox(m, Vec3{ std::min(xi, xo), 0.25f, 4.80f },
                  Vec3{ std::max(xi, xo), 0.85f, 6.30f }, kColConsole);
    }

    addBox(m, Vec3{ -0.42f, 0.25f, 5.05f }, Vec3{ 0.42f, 0.50f, 5.90f }, kColSeat);
    addBox(m, Vec3{ -0.42f, 0.50f, 5.05f }, Vec3{ 0.42f, 1.55f, 5.30f }, kColSeat);

    for (int side = -1; side <= 1; side += 2) {
        const float x0 = 0.74f * side, x1 = 0.90f * side;
        addBox(m, Vec3{ std::min(x0, x1), 0.85f, 6.30f },
                  Vec3{ std::max(x0, x1), 1.55f, 6.50f }, kColHull);
    }

    return m;
}

inline ShipMeshData buildSwitchLever() {
    using namespace shipmesh;
    ShipMeshData m;
    const float hx = kSwitchHalf.x, hz = kSwitchHalf.z;
    const float h  = 2.0f * kSwitchHalf.y;

    addBox(m, Vec3{ -hx, 0.0f, -hz }, Vec3{ hx, h - 0.010f, hz }, kColSwitch);

    addBox(m, Vec3{ -hx, h - 0.010f, -hz }, Vec3{ hx, h, hz }, kColSwitchCap);
    return m;
}

inline ShipMeshData buildLamp() {
    using namespace shipmesh;
    ShipMeshData m;
    addBox(m, Vec3{ -kLampHalf.x, 0.0f,             -kLampHalf.z },
              Vec3{  kLampHalf.x, 2.0f * kLampHalf.y, kLampHalf.z }, kColLamp);
    return m;
}

inline ShipMeshData buildGearLeg() {
    using namespace shipmesh;
    ShipMeshData m;
    const float L = kGearLegLength;
    addBox(m, Vec3{ -0.22f, -L + 0.25f, -0.22f }, Vec3{ 0.22f, 0.0f, 0.22f }, kColGear);
    addBox(m, Vec3{ -0.55f, -L,        -0.55f }, Vec3{ 0.55f, -L + 0.25f, 0.55f },
           kColHull);
    return m;
}

inline ShipMeshData buildCrystalShard() {
    using namespace shipmesh;
    ShipMeshData m;
    const int   kSides = 5;
    const float kBase  = 0.16f;
    const float kWaist = 0.11f;
    const float kShldr = 0.55f;

    const Vec3  apex{ 0.05f, 1.0f, -0.03f };

    Vec3 lo[kSides], hi[kSides];
    for (int i = 0; i < kSides; ++i) {
        const float a = 6.2831853f * i / kSides;
        lo[i] = Vec3{ std::cos(a) * kBase,  0.0f,   std::sin(a) * kBase  };
        hi[i] = Vec3{ std::cos(a) * kWaist, kShldr, std::sin(a) * kWaist };
    }
    for (int i = 0; i < kSides; ++i) {
        const int j = (i + 1) % kSides;
        addQuad(m, lo[i], lo[j], hi[j], hi[i], kColCrystal);
        addTri (m, hi[i], hi[j], apex,          kColCrystalTip);
    }
    return m;
}

inline ShipMeshData buildOutpost() {
    using namespace shipmesh;
    ShipMeshData m;

    const float padR = 34.0f, padY = 14.0f, padT = 2.0f;
    Vec3 rim[6], rimT[6];
    for (int i = 0; i < 6; ++i) {
        const float a = 6.2831853f * i / 6.0f;
        rim [i] = Vec3{ std::cos(a) * padR, padY,        std::sin(a) * padR };
        rimT[i] = Vec3{ std::cos(a) * padR, padY + padT, std::sin(a) * padR };
    }
    const Vec3 hubB{ 0.0f, padY, 0.0f }, hubT{ 0.0f, padY + padT, 0.0f };
    for (int i = 0; i < 6; ++i) {
        const int j = (i + 1) % 6;
        addTri (m, rimT[i], rimT[j], hubT, kColPad);
        addTri (m, rim [j], rim [i], hubB, kColHull);
        addQuad(m, rim[i], rim[j], rimT[j], rimT[i], kColHull);
    }

    for (int i = 0; i < 6; ++i) {
        const float a = 6.2831853f * i / 6.0f;
        const float x = std::cos(a) * (padR - 5.0f), z = std::sin(a) * (padR - 5.0f);
        addBox(m, Vec3{ x - 1.6f, -30.0f, z - 1.6f },
                  Vec3{ x + 1.6f,  padY,  z + 1.6f }, kColHull);
    }

    for (int i = 0; i < 6; ++i) {
        const float a = 6.2831853f * (i + 0.5f) / 6.0f;
        const float x = std::cos(a) * (padR - 3.0f), z = std::sin(a) * (padR - 3.0f);
        addBox(m, Vec3{ x - 0.8f, padY + padT,        z - 0.8f },
                  Vec3{ x + 0.8f, padY + padT + 4.0f, z + 0.8f }, kColBeacon);
    }

    const float mastH = 130.0f;
    addBox(m, Vec3{ -3.0f, 0.0f, -3.0f }, Vec3{ 3.0f, mastH, 3.0f }, kColHull);

    for (int i = 0; i < 3; ++i) {
        const float a = 6.2831853f * i / 3.0f;
        const float x = std::cos(a) * 26.0f, z = std::sin(a) * 26.0f;
        const Vec3 lo{ x, 2.0f, z }, hi{ 0.0f, mastH * 0.62f, 0.0f };
        addQuad(m, Vec3{ lo.x - 1.0f, lo.y, lo.z }, Vec3{ lo.x + 1.0f, lo.y, lo.z },
                   Vec3{ hi.x + 1.0f, hi.y, hi.z }, Vec3{ hi.x - 1.0f, hi.y, hi.z },
                   kColHull);
    }

    addBox(m, Vec3{ -4.5f, mastH,        -4.5f },
              Vec3{  4.5f, mastH + 7.0f,  4.5f }, kColBeacon);

    for (int side = -1; side <= 1; side += 2) {
        const float x = 78.0f * side;
        addBox(m, Vec3{ x - 16.0f, 0.0f, -70.0f * side - 12.0f },
                  Vec3{ x + 16.0f, 17.0f, -70.0f * side + 12.0f }, kColOutpost);
        addBox(m, Vec3{ x - 17.0f, 17.0f, -70.0f * side - 13.0f },
                  Vec3{ x + 17.0f, 19.0f, -70.0f * side + 13.0f }, kColHull);
    }

    {
        const Ring lo{ 0.0f, 11.0f, 11.0f }, hi{ 0.0f, 11.0f, 11.0f };
        (void)lo; (void)hi;
        Vec3 base[8], top[8];
        for (int i = 0; i < 8; ++i) {
            const float a = 6.2831853f * (i + 0.5f) / 8.0f;
            base[i] = Vec3{ -95.0f + std::cos(a) * 12.0f, 0.0f,  60.0f + std::sin(a) * 12.0f };
            top [i] = Vec3{ -95.0f + std::cos(a) * 12.0f, 26.0f, 60.0f + std::sin(a) * 12.0f };
        }
        for (int i = 0; i < 8; ++i) {
            const int j = (i + 1) & 7;
            addQuad(m, base[i], base[j], top[j], top[i], kColOutpost);
            addTri (m, top[i], top[j], Vec3{ -95.0f, 30.0f, 60.0f }, kColHull);
        }
    }
    return m;
}

}
