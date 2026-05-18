#include "gmk/math/transform.hpp"

#include <cmath>

namespace gmk {

Mat3d Mat3d::rotation_x(double r) {
    double c = std::cos(r), s = std::sin(r);
    Mat3d M{};
    M.m[0] = 1; M.m[3] = 0; M.m[6] = 0;
    M.m[1] = 0; M.m[4] = c; M.m[7] = -s;
    M.m[2] = 0; M.m[5] = s; M.m[8] =  c;
    return M;
}
Mat3d Mat3d::rotation_y(double r) {
    double c = std::cos(r), s = std::sin(r);
    Mat3d M{};
    M.m[0] = c;  M.m[3] = 0; M.m[6] = s;
    M.m[1] = 0;  M.m[4] = 1; M.m[7] = 0;
    M.m[2] = -s; M.m[5] = 0; M.m[8] = c;
    return M;
}
Mat3d Mat3d::rotation_z(double r) {
    double c = std::cos(r), s = std::sin(r);
    Mat3d M{};
    M.m[0] = c; M.m[3] = -s; M.m[6] = 0;
    M.m[1] = s; M.m[4] =  c; M.m[7] = 0;
    M.m[2] = 0; M.m[5] =  0; M.m[8] = 1;
    return M;
}
Mat3d Mat3d::rotation_axis(Vec3d a, double r) {
    // Rodrigues' formula.
    double c = std::cos(r), s = std::sin(r), C = 1.0 - c;
    double x = a.x, y = a.y, z = a.z;
    Mat3d M{};
    M.m[0] = c + x*x*C;     M.m[3] = x*y*C - z*s;   M.m[6] = x*z*C + y*s;
    M.m[1] = y*x*C + z*s;   M.m[4] = c + y*y*C;     M.m[7] = y*z*C - x*s;
    M.m[2] = z*x*C - y*s;   M.m[5] = z*y*C + x*s;   M.m[8] = c + z*z*C;
    return M;
}
Mat3d Mat3d::from_euler(angle_t rx, angle_t ry, angle_t rz) {
    return rotation_z(angle_to_rad(rz)) *
           rotation_y(angle_to_rad(ry)) *
           rotation_x(angle_to_rad(rx));
}

Mat3d Mat3d::operator*(const Mat3d& o) const {
    Mat3d R{};
    for (int col = 0; col < 3; ++col) {
        for (int row = 0; row < 3; ++row) {
            double s = 0;
            for (int k = 0; k < 3; ++k) {
                s += m[k*3 + row] * o.m[col*3 + k];
            }
            R.m[col*3 + row] = s;
        }
    }
    return R;
}
Mat3d Mat3d::transpose() const {
    Mat3d R{};
    R.m[0] = m[0]; R.m[3] = m[1]; R.m[6] = m[2];
    R.m[1] = m[3]; R.m[4] = m[4]; R.m[7] = m[5];
    R.m[2] = m[6]; R.m[5] = m[7]; R.m[8] = m[8];
    return R;
}

// ---------------------------------------------------------------------------
// Transform implementation.
// ---------------------------------------------------------------------------
Vec3d Transform::apply(Vec3d p) const {
    Vec3d r = rotation * p;
    r = r * uniform_scale;
    return Vec3d{
        r.x + length_to_m(translation.x),
        r.y + length_to_m(translation.y),
        r.z + length_to_m(translation.z),
    };
}

Vec3i Transform::apply(Vec3i p, bool* sat_flag) const {
    Vec3d pm = to_vec3d_m(p);
    Vec3d rm = rotation * pm;
    rm = rm * uniform_scale;
    rm.x += length_to_m(translation.x);
    rm.y += length_to_m(translation.y);
    rm.z += length_to_m(translation.z);
    bool sat = false;
    auto map = [&](double v) -> length_t {
        double fm = v * 1e15;
        if (fm >  static_cast<double>(LENGTH_MAX)) { sat = true; return LENGTH_MAX; }
        if (fm <  static_cast<double>(LENGTH_MIN)) { sat = true; return LENGTH_MIN; }
        return static_cast<length_t>(std::llround(fm));
    };
    Vec3i out{ map(rm.x), map(rm.y), map(rm.z) };
    if (sat_flag) *sat_flag = sat;
    return out;
}

Transform Transform::inverse() const {
    // For a rigid+uniform-scale transform: R^-1 = R^T, scale^-1 = 1/scale,
    // translation^-1 = -R^T * (translation / scale).
    Transform inv;
    inv.rotation      = rotation.transpose();
    inv.uniform_scale = (uniform_scale != 0.0) ? (1.0 / uniform_scale) : 1.0;

    Vec3d t_m{ length_to_m(translation.x),
               length_to_m(translation.y),
               length_to_m(translation.z) };
    Vec3d tinv = inv.rotation * (t_m * -inv.uniform_scale);
    inv.translation = from_vec3d_m(tinv);
    return inv;
}

Transform Transform::operator*(const Transform& o) const {
    Transform r;
    r.rotation      = rotation * o.rotation;
    r.uniform_scale = uniform_scale * o.uniform_scale;
    // Apply *this* to o's translation expressed in metres, then add ours.
    Vec3d ot_m{ length_to_m(o.translation.x),
                length_to_m(o.translation.y),
                length_to_m(o.translation.z) };
    Vec3d rt = rotation * (ot_m * uniform_scale);
    Vec3d total{ rt.x + length_to_m(translation.x),
                 rt.y + length_to_m(translation.y),
                 rt.z + length_to_m(translation.z) };
    r.translation = from_vec3d_m(total);
    return r;
}

}  // namespace gmk
