#pragma once
//
// Rigid + uniform-scale transforms. The kernel stores rigid transforms in
// double precision because that's the natural type for rotation matrices.
// Coordinates are converted to metres for the math and back to fm afterwards.
//
// We deliberately do not support sheared or non-uniform-scale transforms
// at this level; freeform deformations live higher in the modeller, not in
// the kernel.
//

#include "gmk/math/vec.hpp"
#include "gmk/units.hpp"

namespace gmk {

struct Mat3d {
    // Column-major 3x3.
    double m[9]{1,0,0, 0,1,0, 0,0,1};

    static constexpr Mat3d identity() { return Mat3d{}; }
    static Mat3d         rotation_x(double rad);
    static Mat3d         rotation_y(double rad);
    static Mat3d         rotation_z(double rad);
    static Mat3d         rotation_axis(Vec3d axis_unit, double rad);
    static Mat3d         from_euler(angle_t rx, angle_t ry, angle_t rz);

    Vec3d operator*(Vec3d v) const {
        return Vec3d{
            m[0]*v.x + m[3]*v.y + m[6]*v.z,
            m[1]*v.x + m[4]*v.y + m[7]*v.z,
            m[2]*v.x + m[5]*v.y + m[8]*v.z
        };
    }
    Mat3d operator*(const Mat3d& o) const;
    Mat3d transpose() const;
};

struct Transform {
    Mat3d    rotation{};
    Vec3i    translation{0,0,0};
    double   uniform_scale = 1.0;

    static Transform identity() { return Transform{}; }

    // Apply transform to a kernel integer point. Conversion goes through
    // double space; any saturation on the way back is reported via a
    // caller-supplied flag, allowing detection in tight loops.
    Vec3i apply(Vec3i p, bool* saturated = nullptr) const;
    Vec3d apply(Vec3d p_metres) const;

    Transform inverse() const;
    Transform operator*(const Transform& o) const;
};

}  // namespace gmk
