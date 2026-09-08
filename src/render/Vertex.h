#pragma once

#include <cmath>

namespace planet {

inline void octEncode(float x, float y, float z, short& ox, short& oy) {
    const float inv = 1.0f / (std::fabs(x) + std::fabs(y) + std::fabs(z) + 1e-20f);
    float u = x * inv, v = y * inv;
    if (z < 0.0f) {
        const float su = u >= 0.0f ? 1.0f : -1.0f;
        const float sv = v >= 0.0f ? 1.0f : -1.0f;
        const float nu = (1.0f - std::fabs(v)) * su;
        const float nv = (1.0f - std::fabs(u)) * sv;
        u = nu; v = nv;
    }
    auto q = [](float t) {
        t = t < -1.0f ? -1.0f : (t > 1.0f ? 1.0f : t);
        return static_cast<short>(t * 32767.0f + (t >= 0.0f ? 0.5f : -0.5f));
    };
    ox = q(u); oy = q(v);
}

struct Vertex {

    float px, py, pz;

    short nx, ny;

    unsigned char r, g, b, a;

    float cx, cy, cz;

    short cnx, cny;

    unsigned char m0, m1, m2, m3;
    unsigned char m4, m5, m6, m7;
};

inline Vertex makeVertex(float px, float py, float pz,
                         float nx, float ny, float nz,
                         float r,  float g,  float b,
                         float cx, float cy, float cz,
                         float cnx, float cny, float cnz,
                         const unsigned char m[8]) {
    Vertex v{};
    v.px = px; v.py = py; v.pz = pz;
    octEncode(nx, ny, nz, v.nx, v.ny);
    auto q8 = [](float t) {
        t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
        return static_cast<unsigned char>(t * 255.0f + 0.5f);
    };
    v.r = q8(r); v.g = q8(g); v.b = q8(b); v.a = 255;
    v.cx = cx; v.cy = cy; v.cz = cz;
    octEncode(cnx, cny, cnz, v.cnx, v.cny);
    v.m0 = m[0]; v.m1 = m[1]; v.m2 = m[2]; v.m3 = m[3];
    v.m4 = m[4]; v.m5 = m[5]; v.m6 = m[6]; v.m7 = m[7];
    return v;
}

inline void octDecode(short ox, short oy, float& x, float& y, float& z) {
    float u = float(ox) / 32767.0f, v = float(oy) / 32767.0f;
    z = 1.0f - std::fabs(u) - std::fabs(v);
    if (z < 0.0f) {
        const float su = u >= 0.0f ? 1.0f : -1.0f;
        const float sv = v >= 0.0f ? 1.0f : -1.0f;
        const float nu = (1.0f - std::fabs(v)) * su;
        const float nv = (1.0f - std::fabs(u)) * sv;
        u = nu; v = nv;
    }
    const float len = std::sqrt(u * u + v * v + z * z);
    const float inv = len > 0.0f ? 1.0f / len : 0.0f;
    x = u * inv; y = v * inv; z *= inv;
}

struct UnpackedNormal { float x, y, z; };

inline UnpackedNormal vertexNormal(const Vertex& v) {
    UnpackedNormal n{};
    octDecode(v.nx, v.ny, n.x, n.y, n.z);
    return n;
}

inline UnpackedNormal vertexCoarseNormal(const Vertex& v) {
    UnpackedNormal n{};
    octDecode(v.cnx, v.cny, n.x, n.y, n.z);
    return n;
}

static_assert(sizeof(Vertex) == 44, "the packed vertex must stay 44 bytes");

}
