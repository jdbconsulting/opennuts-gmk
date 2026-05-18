#pragma once
//
// Analytic curves -- lines, circles and ellipses. Each one is its own
// closed-form Curve implementation (faster than NURBS for everyday use)
// and can be promoted to a NURBS curve via ``to_nurbs()`` for B-rep edge
// storage and intersection algorithms that only know about NURBS.
//
// All maths in metres (double); the fixed-point side converts at the
// boundary.
//

#include "gmk/geom/curve.hpp"
#include "gmk/geom/nurbs_curve.hpp"
#include "gmk/math/vec.hpp"
#include "gmk/result.hpp"

namespace gmk {

// ---------------------------------------------------------------------------
// Line: C(u) = origin + u * direction, with explicit u_min..u_max as a
// finite trim. The direction does not need to be unit length; the
// parameter is metric only if it is.
// ---------------------------------------------------------------------------
class LineCurve final : public Curve {
public:
    LineCurve() = default;
    Status init(Vec3d origin, Vec3d direction, double u_min_, double u_max_);

    CurveKind kind()  const noexcept override { return CurveKind::Line; }
    double    u_min() const noexcept override { return u0_; }
    double    u_max() const noexcept override { return u1_; }
    std::unique_ptr<Curve> clone() const override {
        return std::unique_ptr<Curve>(new LineCurve(*this));
    }

    Status point(double u, Vec3d& out)                              const override;
    Status point_and_tangent(double u, Vec3d& p, Vec3d& t)          const override;

    Vec3d origin()    const noexcept { return o_; }
    Vec3d direction() const noexcept { return d_; }

    Status to_nurbs(NurbsCurve& out) const;

private:
    Vec3d o_{}, d_{};
    double u0_{0}, u1_{1};
};

// ---------------------------------------------------------------------------
// Circle: parametrised by angle in radians 0..2π in a local frame
// (origin, x_axis, y_axis), radius r.
// ---------------------------------------------------------------------------
class CircleCurve final : public Curve {
public:
    CircleCurve() = default;
    Status init(Vec3d origin, Vec3d normal, Vec3d x_axis, double radius);

    CurveKind kind()      const noexcept override { return CurveKind::Circle; }
    double    u_min()     const noexcept override { return 0.0; }
    double    u_max()     const noexcept override { return 2.0 * PI; }
    bool      is_closed() const noexcept override { return true; }
    std::unique_ptr<Curve> clone() const override {
        return std::unique_ptr<Curve>(new CircleCurve(*this));
    }

    Status point(double u, Vec3d& out)                              const override;
    Status point_and_tangent(double u, Vec3d& p, Vec3d& t)          const override;

    double radius() const noexcept { return r_; }
    Vec3d  origin() const noexcept { return o_; }
    Vec3d  normal() const noexcept { return n_; }
    Vec3d  xaxis()  const noexcept { return x_; }
    Vec3d  yaxis()  const noexcept { return y_; }

    // Exact NURBS representation of a full circle using 9 control points
    // (quadratic, 4-segment rational) -- Piegl & Tiller §7.5.
    Status to_nurbs(NurbsCurve& out) const;

private:
    Vec3d o_{}, n_{0,0,1}, x_{1,0,0}, y_{0,1,0};
    double r_{1.0};
};

// ---------------------------------------------------------------------------
// Ellipse: same local frame as circle, with major axis along x_axis and
// minor axis along y_axis. ra is semi-major, rb is semi-minor.
// ---------------------------------------------------------------------------
class EllipseCurve final : public Curve {
public:
    EllipseCurve() = default;
    Status init(Vec3d origin, Vec3d normal, Vec3d x_axis, double ra, double rb);

    CurveKind kind()      const noexcept override { return CurveKind::Ellipse; }
    double    u_min()     const noexcept override { return 0.0; }
    double    u_max()     const noexcept override { return 2.0 * PI; }
    bool      is_closed() const noexcept override { return true; }
    std::unique_ptr<Curve> clone() const override {
        return std::unique_ptr<Curve>(new EllipseCurve(*this));
    }

    Status point(double u, Vec3d& out)                              const override;
    Status point_and_tangent(double u, Vec3d& p, Vec3d& t)          const override;

private:
    Vec3d o_{}, n_{0,0,1}, x_{1,0,0}, y_{0,1,0};
    double ra_{1.0}, rb_{1.0};
};

}  // namespace gmk
