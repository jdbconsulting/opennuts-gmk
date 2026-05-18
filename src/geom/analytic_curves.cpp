#include "gmk/geom/analytic_curves.hpp"

#include <cmath>

namespace gmk {

// ---------------------------------------------------------------------------
// LineCurve.
// ---------------------------------------------------------------------------
Status LineCurve::init(Vec3d o, Vec3d d, double a, double b) {
    if (d.norm_sq() < 1e-30) return Status::DegenerateInput;
    if (a >= b)              return Status::InvalidArgument;
    o_ = o; d_ = d; u0_ = a; u1_ = b;
    return Status::Ok;
}
Status LineCurve::point(double u, Vec3d& out) const {
    out = Vec3d{ o_.x + u*d_.x, o_.y + u*d_.y, o_.z + u*d_.z };
    return Status::Ok;
}
Status LineCurve::point_and_tangent(double u, Vec3d& p, Vec3d& t) const {
    point(u, p);
    t = d_;
    return Status::Ok;
}
Status LineCurve::to_nurbs(NurbsCurve& out) const {
    Vec3d p0{ o_.x + u0_*d_.x, o_.y + u0_*d_.y, o_.z + u0_*d_.z };
    Vec3d p1{ o_.x + u1_*d_.x, o_.y + u1_*d_.y, o_.z + u1_*d_.z };
    double knots[4] = { u0_, u0_, u1_, u1_ };
    Vec3d cps[2] = { p0, p1 };
    return out.init(1, knots, 4, cps, nullptr, 2);
}

// ---------------------------------------------------------------------------
// CircleCurve. Frame is orthonormalised against the supplied axes; if the
// caller passes near-degenerate inputs we report DegenerateInput.
// ---------------------------------------------------------------------------
Status CircleCurve::init(Vec3d o, Vec3d n, Vec3d x, double r) {
    if (!(r > 0.0)) return Status::DegenerateInput;
    double nn = n.norm();
    if (nn < 1e-15) return Status::DegenerateInput;
    n = n * (1.0 / nn);
    // Project x onto the plane perpendicular to n.
    double dxn = x.dot(n);
    Vec3d xp{ x.x - dxn*n.x, x.y - dxn*n.y, x.z - dxn*n.z };
    double xn = xp.norm();
    if (xn < 1e-15) return Status::DegenerateInput;
    xp = xp * (1.0 / xn);
    Vec3d yp = n.cross(xp);
    o_ = o; n_ = n; x_ = xp; y_ = yp; r_ = r;
    return Status::Ok;
}
Status CircleCurve::point(double u, Vec3d& out) const {
    double c = std::cos(u), s = std::sin(u);
    out = Vec3d{ o_.x + r_*(c*x_.x + s*y_.x),
                 o_.y + r_*(c*x_.y + s*y_.y),
                 o_.z + r_*(c*x_.z + s*y_.z) };
    return Status::Ok;
}
Status CircleCurve::point_and_tangent(double u, Vec3d& p, Vec3d& t) const {
    double c = std::cos(u), s = std::sin(u);
    p = Vec3d{ o_.x + r_*(c*x_.x + s*y_.x),
               o_.y + r_*(c*x_.y + s*y_.y),
               o_.z + r_*(c*x_.z + s*y_.z) };
    t = Vec3d{ r_*(-s*x_.x + c*y_.x),
               r_*(-s*x_.y + c*y_.y),
               r_*(-s*x_.z + c*y_.z) };
    return Status::Ok;
}

// Exact rational quadratic representation of a circle: 4 quadrant arcs
// share a degree-2 NURBS with 9 control points, weights (1, √2/2)
// alternating, and a knot vector with multiplicity 3 at the ends and
// 2 at the three internal joints.
//
// Reference: P&T "The NURBS Book", §7.5, eq. (7.32).
Status CircleCurve::to_nurbs(NurbsCurve& out) const {
    constexpr double SQ2_2 = 0.7071067811865476;  // √2/2
    const double w[9] = {1.0, SQ2_2, 1.0, SQ2_2, 1.0, SQ2_2, 1.0, SQ2_2, 1.0};
    // Control points (in the local frame, before centering and orientation).
    double r = r_;
    Vec3d local[9] = {
        Vec3d{ r,  0, 0},
        Vec3d{ r,  r, 0},
        Vec3d{ 0,  r, 0},
        Vec3d{-r,  r, 0},
        Vec3d{-r,  0, 0},
        Vec3d{-r, -r, 0},
        Vec3d{ 0, -r, 0},
        Vec3d{ r, -r, 0},
        Vec3d{ r,  0, 0},
    };
    Vec3d cps[9];
    for (int i = 0; i < 9; ++i) {
        Vec3d L = local[i];
        cps[i] = Vec3d{
            o_.x + L.x*x_.x + L.y*y_.x,
            o_.y + L.x*x_.y + L.y*y_.y,
            o_.z + L.x*x_.z + L.y*y_.z,
        };
    }
    const double knots[12] = {
        0,         0,         0,
        0.25*2*PI, 0.25*2*PI,
        0.50*2*PI, 0.50*2*PI,
        0.75*2*PI, 0.75*2*PI,
        2*PI,      2*PI,      2*PI
    };
    return out.init(2, knots, 12, cps, w, 9);
}

// ---------------------------------------------------------------------------
// EllipseCurve.
// ---------------------------------------------------------------------------
Status EllipseCurve::init(Vec3d o, Vec3d n, Vec3d x, double ra, double rb) {
    if (!(ra > 0.0) || !(rb > 0.0)) return Status::DegenerateInput;
    double nn = n.norm(); if (nn < 1e-15) return Status::DegenerateInput;
    n = n * (1.0 / nn);
    double dxn = x.dot(n);
    Vec3d xp{ x.x - dxn*n.x, x.y - dxn*n.y, x.z - dxn*n.z };
    double xn = xp.norm(); if (xn < 1e-15) return Status::DegenerateInput;
    xp = xp * (1.0 / xn);
    Vec3d yp = n.cross(xp);
    o_ = o; n_ = n; x_ = xp; y_ = yp; ra_ = ra; rb_ = rb;
    return Status::Ok;
}
Status EllipseCurve::point(double u, Vec3d& out) const {
    double c = std::cos(u), s = std::sin(u);
    out = Vec3d{ o_.x + ra_*c*x_.x + rb_*s*y_.x,
                 o_.y + ra_*c*x_.y + rb_*s*y_.y,
                 o_.z + ra_*c*x_.z + rb_*s*y_.z };
    return Status::Ok;
}
Status EllipseCurve::point_and_tangent(double u, Vec3d& p, Vec3d& t) const {
    point(u, p);
    double c = std::cos(u), s = std::sin(u);
    t = Vec3d{ -ra_*s*x_.x + rb_*c*y_.x,
               -ra_*s*x_.y + rb_*c*y_.y,
               -ra_*s*x_.z + rb_*c*y_.z };
    return Status::Ok;
}

}  // namespace gmk
