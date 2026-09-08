#pragma once

#include "math/SpaceMath.h"
#include <cmath>
#include <algorithm>

namespace space {

    struct DVec3 {
        double x = 0.0, y = 0.0, z = 0.0;
        DVec3() = default;
        DVec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

        DVec3(const Vec3& v) : x(v.x), y(v.y), z(v.z) {}
    };

    inline DVec3 operator+(const DVec3& a, const DVec3& b) { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
    inline DVec3 operator-(const DVec3& a, const DVec3& b) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
    inline DVec3 operator*(const DVec3& a, double s)       { return { a.x * s,  a.y * s,  a.z * s  }; }
    inline DVec3 operator-(const DVec3& a)                 { return { -a.x, -a.y, -a.z }; }

    inline double dot(const DVec3& a, const DVec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
    inline DVec3  cross(const DVec3& a, const DVec3& b) {
        return { a.y * b.z - a.z * b.y,
                 a.z * b.x - a.x * b.z,
                 a.x * b.y - a.y * b.x };
    }
    inline double length(const DVec3& a) { return std::sqrt(dot(a, a)); }
    inline DVec3  normalize(const DVec3& a) {
        const double len = length(a);
        return (len > 1e-300) ? a * (1.0 / len) : DVec3{ 0.0, 0.0, 1.0 };
    }

    inline Vec3 toF(const DVec3& a) {
        return { static_cast<float>(a.x), static_cast<float>(a.y), static_cast<float>(a.z) };
    }
    inline DVec3 toD(const Vec3& a) { return { a.x, a.y, a.z }; }

}

namespace planet {

using space::DVec3;

struct DAABB {
    DVec3 mn{ 1e300,  1e300,  1e300};
    DVec3 mx{-1e300, -1e300, -1e300};

    void expand(const DVec3& p) {
        mn.x = std::min(mn.x, p.x); mn.y = std::min(mn.y, p.y); mn.z = std::min(mn.z, p.z);
        mx.x = std::max(mx.x, p.x); mx.y = std::max(mx.y, p.y); mx.z = std::max(mx.z, p.z);
    }
    DVec3 center() const {
        return { (mn.x + mx.x) * 0.5, (mn.y + mx.y) * 0.5, (mn.z + mx.z) * 0.5 };
    }
};

inline double distanceToDAABB(const DAABB& b, const DVec3& p) {
    const double dx = std::max({ b.mn.x - p.x, 0.0, p.x - b.mx.x });
    const double dy = std::max({ b.mn.y - p.y, 0.0, p.y - b.mx.y });
    const double dz = std::max({ b.mn.z - p.z, 0.0, p.z - b.mx.z });
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}
