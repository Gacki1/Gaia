#pragma once

#include <cmath>
#include <algorithm>

namespace space {

    constexpr float kPi = 3.14159265358979323846f;

    inline float deg2rad(float deg) { return deg * (kPi / 180.0f); }

    struct Vec3 {
        float x = 0.0f, y = 0.0f, z = 0.0f;
        Vec3() = default;
        Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    };

    inline Vec3 operator+(const Vec3& a, const Vec3& b) { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
    inline Vec3 operator-(const Vec3& a, const Vec3& b) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
    inline Vec3 operator*(const Vec3& a, float s)       { return { a.x * s,  a.y * s,  a.z * s }; }

    inline float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
    inline Vec3  cross(const Vec3& a, const Vec3& b) {
        return { a.y * b.z - a.z * b.y,
                 a.z * b.x - a.x * b.z,
                 a.x * b.y - a.y * b.x };
    }
    inline float length(const Vec3& a) { return std::sqrt(dot(a, a)); }
    inline Vec3  normalize(const Vec3& a) {
        const float len = length(a);
        return (len > 1e-8f) ? a * (1.0f / len) : Vec3{ 0.0f, 0.0f, 1.0f };
    }

    struct Basis {
        Vec3 forward{ 0.0f, 0.0f, 1.0f };
        Vec3 right{ 1.0f, 0.0f, 0.0f };
        Vec3 up{ 0.0f, 1.0f, 0.0f };
    };

    inline Basis makeBasis(float yaw, float pitch) {
        const float cy = std::cos(yaw),   sy = std::sin(yaw);
        const float cp = std::cos(pitch), sp = std::sin(pitch);

        Basis b;
        b.forward = normalize(Vec3{ sy * cp, sp, cy * cp });

        b.right   = normalize(cross(Vec3{ 0.0f, 1.0f, 0.0f }, b.forward));
        b.up      = normalize(cross(b.forward, b.right));
        return b;
    }

    inline float clampPitch(float pitch, float maxPitch) {
        return std::max(-maxPitch, std::min(maxPitch, pitch));
    }

    inline float focalLength(float vFovRadians) {
        return 1.0f / std::tan(vFovRadians * 0.5f);
    }

    struct Projected {
        bool  visible = false;
        float ndcX = 0.0f;
        float ndcY = 0.0f;
        float depth = 0.0f;
    };

    inline Projected projectPoint(const Vec3& world, const Vec3& camPos,
                                  const Basis& basis, float focal,
                                  float aspect, float nearPlane) {
        const Vec3 rel = world - camPos;
        const float depth = dot(rel, basis.forward);

        Projected out;
        out.depth = depth;
        if (depth <= nearPlane) {
            out.visible = false;
            return out;
        }

        const float camX = dot(rel, basis.right);
        const float camY = dot(rel, basis.up);

        out.ndcX =  (focal * camX) / (depth * aspect);
        out.ndcY = -(focal * camY) / depth;
        out.visible = true;
        return out;
    }

    inline float projectedRadiusNDC(float worldRadius, float depth, float focal) {
        return (focal * worldRadius) / depth;
    }

    inline float wrapAxis(float v, float half) {
        const float size = 2.0f * half;
        float r = std::fmod(v + half, size);
        if (r < 0.0f) r += size;
        return r - half;
    }

}
