#pragma once
//
// gmk::Vec -- vector types used throughout the kernel.
//
//   Vec3i   3D point/vector in kernel integer units (length_t).
//   Vec3d   3D point/vector in double precision (the working type inside
//           numerical routines).
//   Vec3a   3D triple of fixed-point angles (used for Euler rotations).
//
// Conversions between Vec3i and Vec3d are explicit so it's obvious where
// precision is being traded.
//

#include <array>
#include <cmath>
#include <cstdint>

#include "gmk/units.hpp"

namespace gmk {

struct Vec3d {
    double x{0}, y{0}, z{0};

    constexpr Vec3d() = default;
    constexpr Vec3d(double X, double Y, double Z) : x{X}, y{Y}, z{Z} {}

    constexpr double  operator[](int i) const { return i==0?x:i==1?y:z; }
    constexpr double& operator[](int i)       { return i==0?x:i==1?y:z; }

    constexpr Vec3d operator+(Vec3d o) const { return {x+o.x, y+o.y, z+o.z}; }
    constexpr Vec3d operator-(Vec3d o) const { return {x-o.x, y-o.y, z-o.z}; }
    constexpr Vec3d operator-()         const { return {-x, -y, -z}; }
    constexpr Vec3d operator*(double s) const { return {x*s, y*s, z*s}; }
    constexpr Vec3d operator/(double s) const { return {x/s, y/s, z/s}; }
    constexpr Vec3d& operator+=(Vec3d o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
    constexpr Vec3d& operator-=(Vec3d o) { x-=o.x; y-=o.y; z-=o.z; return *this; }
    constexpr Vec3d& operator*=(double s){ x*=s;   y*=s;   z*=s;   return *this; }

    constexpr double dot(Vec3d o) const { return x*o.x + y*o.y + z*o.z; }
    constexpr Vec3d  cross(Vec3d o) const {
        return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x};
    }
    double           norm()    const { return std::sqrt(dot(*this)); }
    constexpr double norm_sq() const { return dot(*this); }
    Vec3d            normalized() const {
        double n = norm();
        return (n > 0.0) ? Vec3d{x/n, y/n, z/n} : Vec3d{};
    }
};

constexpr Vec3d operator*(double s, Vec3d v) { return v * s; }

struct Vec3i {
    length_t x{0}, y{0}, z{0};

    constexpr Vec3i() = default;
    constexpr Vec3i(length_t X, length_t Y, length_t Z) : x{X}, y{Y}, z{Z} {}

    constexpr bool operator==(const Vec3i& o) const noexcept {
        return x == o.x && y == o.y && z == o.z;
    }
    constexpr bool operator!=(const Vec3i& o) const noexcept { return !(*this == o); }

    constexpr Vec3i operator+(const Vec3i& o) const noexcept {
        return { sat_add(x, o.x), sat_add(y, o.y), sat_add(z, o.z) };
    }
    constexpr Vec3i operator-(const Vec3i& o) const noexcept {
        return { sat_sub(x, o.x), sat_sub(y, o.y), sat_sub(z, o.z) };
    }
    constexpr Vec3i operator-() const noexcept {
        return { -x, -y, -z };
    }
};

struct Vec3a {
    angle_t x{0}, y{0}, z{0};  // canonical: rotate about X, then Y, then Z.

    constexpr Vec3a() = default;
    constexpr Vec3a(angle_t X, angle_t Y, angle_t Z) : x{X}, y{Y}, z{Z} {}
};

// ---------------------------------------------------------------------------
// Conversions between fixed-point and double-precision space.
// ---------------------------------------------------------------------------
inline Vec3d to_vec3d_m(Vec3i v) noexcept {
    return Vec3d{ length_to_m(v.x), length_to_m(v.y), length_to_m(v.z) };
}
inline Vec3d to_vec3d_mm(Vec3i v) noexcept {
    return Vec3d{ length_to_mm(v.x), length_to_mm(v.y), length_to_mm(v.z) };
}
inline Vec3i from_vec3d_m(Vec3d v) noexcept {
    return Vec3i{ m_to_length(v.x), m_to_length(v.y), m_to_length(v.z) };
}
inline Vec3i from_vec3d_mm(Vec3d v) noexcept {
    return Vec3i{ mm_to_length(v.x), mm_to_length(v.y), mm_to_length(v.z) };
}

// Distance helpers that take the integer path to avoid double overflow on
// kilometre-scale models. Result is in metres.
inline double distance_m(Vec3i a, Vec3i b) noexcept {
    // Cast to double *after* subtraction to keep precision around the
    // origin, and *before* the squaring to keep dynamic range.
    double dx = length_to_m(b.x - a.x);
    double dy = length_to_m(b.y - a.y);
    double dz = length_to_m(b.z - a.z);
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

}  // namespace gmk
