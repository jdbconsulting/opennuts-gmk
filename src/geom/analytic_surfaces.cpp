#include "gmk/geom/analytic_surfaces.hpp"

#include <cmath>

namespace gmk {

// Project x_ref onto the plane perpendicular to axis, then orthonormalise.
// Returns Ok and (xp, yp = axis × xp). Returns DegenerateInput on failure.
static Status build_orthonormal_frame(Vec3d axis, Vec3d x_ref,
                                      Vec3d& axis_unit, Vec3d& xp, Vec3d& yp) {
    double an = axis.norm();
    if (an < 1e-15) return Status::DegenerateInput;
    axis_unit = axis * (1.0 / an);
    double dxn = x_ref.dot(axis_unit);
    xp = Vec3d{ x_ref.x - dxn*axis_unit.x,
                x_ref.y - dxn*axis_unit.y,
                x_ref.z - dxn*axis_unit.z };
    double xn = xp.norm();
    if (xn < 1e-15) return Status::DegenerateInput;
    xp = xp * (1.0 / xn);
    yp = axis_unit.cross(xp);
    return Status::Ok;
}

// ---------------------------------------------------------------------------
// Plane.
// ---------------------------------------------------------------------------
Status PlaneSurface::init(Vec3d o, Vec3d xa, Vec3d ya,
                          double u0, double u1, double v0, double v1) {
    if (u0 >= u1 || v0 >= v1) return Status::InvalidArgument;
    double xn = xa.norm(); if (xn < 1e-15) return Status::DegenerateInput;
    Vec3d xu = xa * (1.0 / xn);
    // Project y_axis perpendicular to xu.
    double dxy = ya.dot(xu);
    Vec3d yp{ ya.x - dxy*xu.x, ya.y - dxy*xu.y, ya.z - dxy*xu.z };
    double yn = yp.norm(); if (yn < 1e-15) return Status::DegenerateInput;
    Vec3d yu = yp * (1.0 / yn);
    o_ = o; x_ = xu; y_ = yu; n_ = xu.cross(yu);
    u0_ = u0; u1_ = u1; v0_ = v0; v1_ = v1;
    return Status::Ok;
}
Status PlaneSurface::point(double u, double v, Vec3d& out) const {
    out = Vec3d{ o_.x + u*x_.x + v*y_.x,
                 o_.y + u*x_.y + v*y_.y,
                 o_.z + u*x_.z + v*y_.z };
    return Status::Ok;
}
Status PlaneSurface::point_and_partials(double u, double v,
                                        Vec3d& p, Vec3d& du, Vec3d& dv) const {
    point(u, v, p);
    du = x_;
    dv = y_;
    return Status::Ok;
}

// ---------------------------------------------------------------------------
// Sphere.  axis_z is "north pole" direction.  axis_x is the reference for u=0.
// ---------------------------------------------------------------------------
Status SphereSurface::init(Vec3d o, Vec3d az_in, Vec3d ax_in, double r) {
    if (!(r > 0.0)) return Status::DegenerateInput;
    Vec3d az, axx, ayy;
    Status s = build_orthonormal_frame(az_in, ax_in, az, axx, ayy);
    if (s != Status::Ok) return s;
    o_ = o; ax_ = az; ay_ = ayy; az_ = axx; r_ = r;
    // Naming convention here: ax_ is the *axis of revolution* ("z"),
    // ay_ is the y-axis of the equator, az_ is the x-axis of the equator.
    return Status::Ok;
}
Status SphereSurface::point(double u, double v, Vec3d& out) const {
    double cu = std::cos(u), su = std::sin(u);
    double cv = std::cos(v), sv = std::sin(v);
    double x = r_ * cv * cu;
    double y = r_ * cv * su;
    double z = r_ * sv;
    out = Vec3d{ o_.x + x*az_.x + y*ay_.x + z*ax_.x,
                 o_.y + x*az_.y + y*ay_.y + z*ax_.y,
                 o_.z + x*az_.z + y*ay_.z + z*ax_.z };
    return Status::Ok;
}
Status SphereSurface::point_and_partials(double u, double v,
                                         Vec3d& p, Vec3d& du, Vec3d& dv) const {
    double cu = std::cos(u), su = std::sin(u);
    double cv = std::cos(v), sv = std::sin(v);
    double r  = r_;
    p = Vec3d{ o_.x + r*cv*cu*az_.x + r*cv*su*ay_.x + r*sv*ax_.x,
               o_.y + r*cv*cu*az_.y + r*cv*su*ay_.y + r*sv*ax_.y,
               o_.z + r*cv*cu*az_.z + r*cv*su*ay_.z + r*sv*ax_.z };
    // ∂/∂u
    double dxu = -r*cv*su, dyu = r*cv*cu, dzu = 0;
    du = Vec3d{ dxu*az_.x + dyu*ay_.x + dzu*ax_.x,
                dxu*az_.y + dyu*ay_.y + dzu*ax_.y,
                dxu*az_.z + dyu*ay_.z + dzu*ax_.z };
    // ∂/∂v
    double dxv = -r*sv*cu, dyv = -r*sv*su, dzv = r*cv;
    dv = Vec3d{ dxv*az_.x + dyv*ay_.x + dzv*ax_.x,
                dxv*az_.y + dyv*ay_.y + dzv*ax_.y,
                dxv*az_.z + dyv*ay_.z + dzv*ax_.z };
    return Status::Ok;
}

// ---------------------------------------------------------------------------
// Cylinder.
// ---------------------------------------------------------------------------
Status CylinderSurface::init(Vec3d o, Vec3d ax, Vec3d xref, double r,
                             double v0, double v1) {
    if (!(r > 0.0)) return Status::DegenerateInput;
    if (v0 >= v1)   return Status::InvalidArgument;
    Vec3d axu, xp, yp;
    Status s = build_orthonormal_frame(ax, xref, axu, xp, yp);
    if (s != Status::Ok) return s;
    o_ = o; ax_ = axu; x_ = xp; y_ = yp; r_ = r; v0_ = v0; v1_ = v1;
    return Status::Ok;
}
Status CylinderSurface::point(double u, double v, Vec3d& out) const {
    double c = std::cos(u), s = std::sin(u);
    out = Vec3d{ o_.x + r_*c*x_.x + r_*s*y_.x + v*ax_.x,
                 o_.y + r_*c*x_.y + r_*s*y_.y + v*ax_.y,
                 o_.z + r_*c*x_.z + r_*s*y_.z + v*ax_.z };
    return Status::Ok;
}
Status CylinderSurface::point_and_partials(double u, double v,
                                           Vec3d& p, Vec3d& du, Vec3d& dv) const {
    double c = std::cos(u), s = std::sin(u);
    p = Vec3d{ o_.x + r_*c*x_.x + r_*s*y_.x + v*ax_.x,
               o_.y + r_*c*x_.y + r_*s*y_.y + v*ax_.y,
               o_.z + r_*c*x_.z + r_*s*y_.z + v*ax_.z };
    du = Vec3d{ -r_*s*x_.x + r_*c*y_.x,
                -r_*s*x_.y + r_*c*y_.y,
                -r_*s*x_.z + r_*c*y_.z };
    dv = ax_;
    return Status::Ok;
}

// ---------------------------------------------------------------------------
// Cone.  Radius increases linearly with v; for a tip (apex) cone start at
// some v0 < 0 such that r_at(v0) = 0.
// ---------------------------------------------------------------------------
Status ConeSurface::init(Vec3d o, Vec3d ax, Vec3d xref,
                         double r_base, double half_angle,
                         double v0, double v1) {
    if (v0 >= v1)               return Status::InvalidArgument;
    if (!(r_base >= 0.0))       return Status::DegenerateInput;
    if (half_angle < -PI/2 || half_angle > PI/2)
                                return Status::OutOfRange;
    Vec3d axu, xp, yp;
    Status s = build_orthonormal_frame(ax, xref, axu, xp, yp);
    if (s != Status::Ok) return s;
    o_ = o; ax_ = axu; x_ = xp; y_ = yp;
    r_base_ = r_base; half_angle_ = half_angle;
    v0_ = v0; v1_ = v1;
    return Status::Ok;
}
Status ConeSurface::point(double u, double v, Vec3d& out) const {
    double r_at = r_base_ + v * std::tan(half_angle_);
    if (r_at < 0.0) r_at = 0.0;
    double c = std::cos(u), s = std::sin(u);
    out = Vec3d{ o_.x + r_at*c*x_.x + r_at*s*y_.x + v*ax_.x,
                 o_.y + r_at*c*x_.y + r_at*s*y_.y + v*ax_.y,
                 o_.z + r_at*c*x_.z + r_at*s*y_.z + v*ax_.z };
    return Status::Ok;
}
Status ConeSurface::point_and_partials(double u, double v,
                                       Vec3d& p, Vec3d& du, Vec3d& dv) const {
    double tan_h = std::tan(half_angle_);
    double r_at = r_base_ + v * tan_h;
    if (r_at < 0.0) r_at = 0.0;
    double c = std::cos(u), s = std::sin(u);
    p  = Vec3d{ o_.x + r_at*c*x_.x + r_at*s*y_.x + v*ax_.x,
                o_.y + r_at*c*x_.y + r_at*s*y_.y + v*ax_.y,
                o_.z + r_at*c*x_.z + r_at*s*y_.z + v*ax_.z };
    du = Vec3d{ -r_at*s*x_.x + r_at*c*y_.x,
                -r_at*s*x_.y + r_at*c*y_.y,
                -r_at*s*x_.z + r_at*c*y_.z };
    dv = Vec3d{ tan_h*c*x_.x + tan_h*s*y_.x + ax_.x,
                tan_h*c*x_.y + tan_h*s*y_.y + ax_.y,
                tan_h*c*x_.z + tan_h*s*y_.z + ax_.z };
    return Status::Ok;
}

// ---------------------------------------------------------------------------
// Torus.
// ---------------------------------------------------------------------------
Status TorusSurface::init(Vec3d o, Vec3d ax, Vec3d xref, double R, double r) {
    if (!(R > 0.0) || !(r > 0.0)) return Status::DegenerateInput;
    Vec3d axu, xp, yp;
    Status s = build_orthonormal_frame(ax, xref, axu, xp, yp);
    if (s != Status::Ok) return s;
    o_ = o; ax_ = axu; x_ = xp; y_ = yp; R_ = R; r_ = r;
    return Status::Ok;
}
Status TorusSurface::point(double u, double v, Vec3d& out) const {
    double cu = std::cos(u), su = std::sin(u);
    double cv = std::cos(v), sv = std::sin(v);
    double ring_r = R_ + r_ * cv;
    out = Vec3d{ o_.x + ring_r*cu*x_.x + ring_r*su*y_.x + r_*sv*ax_.x,
                 o_.y + ring_r*cu*x_.y + ring_r*su*y_.y + r_*sv*ax_.y,
                 o_.z + ring_r*cu*x_.z + ring_r*su*y_.z + r_*sv*ax_.z };
    return Status::Ok;
}
Status TorusSurface::point_and_partials(double u, double v,
                                        Vec3d& p, Vec3d& du, Vec3d& dv) const {
    double cu = std::cos(u), su = std::sin(u);
    double cv = std::cos(v), sv = std::sin(v);
    double ring_r = R_ + r_ * cv;
    p = Vec3d{ o_.x + ring_r*cu*x_.x + ring_r*su*y_.x + r_*sv*ax_.x,
               o_.y + ring_r*cu*x_.y + ring_r*su*y_.y + r_*sv*ax_.y,
               o_.z + ring_r*cu*x_.z + ring_r*su*y_.z + r_*sv*ax_.z };
    du = Vec3d{ -ring_r*su*x_.x + ring_r*cu*y_.x,
                -ring_r*su*x_.y + ring_r*cu*y_.y,
                -ring_r*su*x_.z + ring_r*cu*y_.z };
    dv = Vec3d{ -r_*sv*cu*x_.x - r_*sv*su*y_.x + r_*cv*ax_.x,
                -r_*sv*cu*x_.y - r_*sv*su*y_.y + r_*cv*ax_.y,
                -r_*sv*cu*x_.z - r_*sv*su*y_.z + r_*cv*ax_.z };
    return Status::Ok;
}

}  // namespace gmk
