#pragma once

#include "math/SpaceMath.h"
#include "math/WorldMath.h"
#include <array>
#include <cmath>
#include <algorithm>

namespace planet {

using space::Vec3;

struct Mat4 {

    std::array<float, 16> m{};

    static Mat4 identity() {
        Mat4 r;
        r.m[0] = r.m[5] = r.m[10] = r.m[15] = 1.0f;
        return r;
    }

    float  operator()(int row, int col) const { return m[col * 4 + row]; }
    float& operator()(int row, int col)       { return m[col * 4 + row]; }
};

inline Mat4 operator*(const Mat4& a, const Mat4& b) {
    Mat4 r;
    for (int c = 0; c < 4; ++c)
        for (int row = 0; row < 4; ++row) {
            float s = 0.0f;
            for (int k = 0; k < 4; ++k) s += a.m[k * 4 + row] * b.m[c * 4 + k];
            r.m[c * 4 + row] = s;
        }
    return r;
}

struct Vec4 { float x, y, z, w; };
inline Vec4 transform(const Mat4& a, const Vec3& v) {
    return {
        a.m[0] * v.x + a.m[4] * v.y + a.m[8]  * v.z + a.m[12],
        a.m[1] * v.x + a.m[5] * v.y + a.m[9]  * v.z + a.m[13],
        a.m[2] * v.x + a.m[6] * v.y + a.m[10] * v.z + a.m[14],
        a.m[3] * v.x + a.m[7] * v.y + a.m[11] * v.z + a.m[15],
    };
}

inline Mat4 lookAt(const Vec3& eye, const Vec3& forward, const Vec3& up) {
    Vec3 f = space::normalize(forward);
    Vec3 s = space::normalize(space::cross(f, up));
    Vec3 u = space::cross(s, f);

    Mat4 r = Mat4::identity();
    r.m[0] = s.x;  r.m[4] = s.y;  r.m[8]  = s.z;  r.m[12] = -space::dot(s, eye);
    r.m[1] = u.x;  r.m[5] = u.y;  r.m[9]  = u.z;  r.m[13] = -space::dot(u, eye);
    r.m[2] = -f.x; r.m[6] = -f.y; r.m[10] = -f.z; r.m[14] =  space::dot(f, eye);
    r.m[3] = 0.0f; r.m[7] = 0.0f; r.m[11] = 0.0f; r.m[15] = 1.0f;
    return r;
}

inline Mat4 perspective(float fovYRadians, float aspect, float nearZ, float farZ) {
    const float t = std::tan(fovYRadians * 0.5f);
    Mat4 r;
    r.m[0]  = 1.0f / (aspect * t);
    r.m[5]  = -1.0f / t;
    r.m[10] = farZ / (nearZ - farZ);
    r.m[11] = -1.0f;
    r.m[14] = -(farZ * nearZ) / (farZ - nearZ);
    return r;
}

inline Mat4 perspectiveInfReverseZ(float fovYRadians, float aspect, float nearZ) {
    const float t = std::tan(fovYRadians * 0.5f);
    Mat4 r;
    r.m[0]  = 1.0f / (aspect * t);
    r.m[5]  = -1.0f / t;
    r.m[10] = 0.0f;
    r.m[11] = -1.0f;
    r.m[14] = nearZ;
    return r;
}

struct AABB {
    Vec3 mn{ 1e30f,  1e30f,  1e30f};
    Vec3 mx{-1e30f, -1e30f, -1e30f};
    void expand(const Vec3& p) {
        mn.x = std::min(mn.x, p.x); mn.y = std::min(mn.y, p.y); mn.z = std::min(mn.z, p.z);
        mx.x = std::max(mx.x, p.x); mx.y = std::max(mx.y, p.y); mx.z = std::max(mx.z, p.z);
    }
    Vec3 center() const { return {(mn.x+mx.x)*0.5f, (mn.y+mx.y)*0.5f, (mn.z+mx.z)*0.5f}; }
};

inline AABB relativeAABB(const DAABB& b, const space::DVec3& camera) {
    AABB out;
    out.expand(space::toF(b.mn - camera));
    out.expand(space::toF(b.mx - camera));
    return out;
}

struct Frustum {
    std::array<std::array<float, 4>, 6> planes{};

    static Frustum fromViewProj(const Mat4& vp) {

        auto row = [&](int R) {
            return std::array<float, 4>{ vp(R,0), vp(R,1), vp(R,2), vp(R,3) };
        };
        const auto r0 = row(0), r1 = row(1), r2 = row(2), r3 = row(3);
        auto add = [](const std::array<float,4>& a, const std::array<float,4>& b) {
            return std::array<float,4>{a[0]+b[0], a[1]+b[1], a[2]+b[2], a[3]+b[3]};
        };
        auto sub = [](const std::array<float,4>& a, const std::array<float,4>& b) {
            return std::array<float,4>{a[0]-b[0], a[1]-b[1], a[2]-b[2], a[3]-b[3]};
        };
        Frustum fr;
        fr.planes[0] = add(r3, r0);
        fr.planes[1] = sub(r3, r0);
        fr.planes[2] = add(r3, r1);
        fr.planes[3] = sub(r3, r1);

        fr.planes[4] = r2;
        fr.planes[5] = sub(r3, r2);
        for (auto& p : fr.planes) {
            float len = std::sqrt(p[0]*p[0] + p[1]*p[1] + p[2]*p[2]);
            if (len > 1e-8f) {
                p[0]/=len; p[1]/=len; p[2]/=len; p[3]/=len;
            } else {

                p = { 0.0f, 0.0f, 0.0f, 1.0f };
            }
        }
        return fr;
    }

    bool intersectsAABB(const AABB& b) const {
        for (const auto& p : planes) {
            Vec3 pv{
                p[0] >= 0.0f ? b.mx.x : b.mn.x,
                p[1] >= 0.0f ? b.mx.y : b.mn.y,
                p[2] >= 0.0f ? b.mx.z : b.mn.z,
            };
            if (p[0]*pv.x + p[1]*pv.y + p[2]*pv.z + p[3] < 0.0f)
                return false;
        }
        return true;
    }
};

}
