#pragma once

#include "math/SpaceMath.h"
#include "math/WorldMath.h"
#include <cmath>

namespace space {

struct Quat {
    float x = 0.0f, y = 0.0f, z = 0.0f, w = 1.0f;
};

inline Quat quatIdentity() { return Quat{ 0.0f, 0.0f, 0.0f, 1.0f }; }

inline Quat operator*(const Quat& a, const Quat& b) {
    return {
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    };
}

inline float dot(const Quat& a, const Quat& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

inline Quat normalize(const Quat& q) {
    const float len = std::sqrt(dot(q, q));
    if (len < 1e-20f) return quatIdentity();
    const float inv = 1.0f / len;
    return { q.x * inv, q.y * inv, q.z * inv, q.w * inv };
}

inline Quat conjugate(const Quat& q) { return { -q.x, -q.y, -q.z, q.w }; }

inline Quat fromAxisAngle(const Vec3& axis, float radians) {
    const float len = length(axis);
    if (len < 1e-20f) return quatIdentity();
    const float h = radians * 0.5f;
    const float s = std::sin(h) / len;
    return { axis.x * s, axis.y * s, axis.z * s, std::cos(h) };
}

inline Vec3 rotate(const Quat& q, const Vec3& v) {
    const Vec3 u{ q.x, q.y, q.z };
    const Vec3 t = cross(u, v) * 2.0f;
    return v + t * q.w + cross(u, t);
}

inline DVec3 rotate(const Quat& q, const DVec3& v) {
    const DVec3 u{ q.x, q.y, q.z };
    const DVec3 t = cross(u, v) * 2.0;
    return v + t * static_cast<double>(q.w) + cross(u, t);
}

inline Basis toBasis(const Quat& q) {
    Basis b;
    b.forward = rotate(q, Vec3{ 0.0f, 0.0f, 1.0f });
    b.right   = rotate(q, Vec3{ 1.0f, 0.0f, 0.0f });
    b.up      = rotate(q, Vec3{ 0.0f, 1.0f, 0.0f });
    return b;
}

inline Quat fromYawPitch(float yaw, float pitch) {
    return fromAxisAngle(Vec3{ 0.0f, 1.0f, 0.0f }, yaw) *
           fromAxisAngle(Vec3{ 1.0f, 0.0f, 0.0f }, -pitch);
}

inline Quat fromTo(const Vec3& from, const Vec3& to) {
    const Vec3  f = normalize(from), t = normalize(to);
    const float d = dot(f, t);
    if (d >= 1.0f - 1e-6f) return quatIdentity();
    if (d <= -1.0f + 1e-6f) {
        Vec3 axis = cross(Vec3{ 1.0f, 0.0f, 0.0f }, f);
        if (length(axis) < 1e-6f) axis = cross(Vec3{ 0.0f, 1.0f, 0.0f }, f);
        return fromAxisAngle(normalize(axis), kPi);
    }
    const Vec3  c = cross(f, t);
    const float s = std::sqrt((1.0f + d) * 2.0f);
    return normalize(Quat{ c.x / s, c.y / s, c.z / s, s * 0.5f });
}

inline Quat nlerp(const Quat& a, const Quat& b, float t) {

    const float s = dot(a, b) < 0.0f ? -1.0f : 1.0f;
    return normalize(Quat{
        a.x + (b.x * s - a.x) * t,
        a.y + (b.y * s - a.y) * t,
        a.z + (b.z * s - a.z) * t,
        a.w + (b.w * s - a.w) * t,
    });
}

inline float angleBetween(const Quat& a, const Quat& b) {
    float d = std::fabs(dot(a, b));
    if (d > 1.0f) d = 1.0f;
    return 2.0f * std::acos(d);
}

inline Quat integrate(const Quat& q, const Vec3& angVelBody, float dt) {
    const float w = length(angVelBody);
    if (w < 1e-9f) return q;
    const float h = w * dt * 0.5f;
    const float s = std::sin(h) / w;
    return normalize(q * Quat{ angVelBody.x * s,
                               angVelBody.y * s,
                               angVelBody.z * s,
                               std::cos(h) });
}

}
