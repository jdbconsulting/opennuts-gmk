#pragma once
//
// gmk::NurbsCurve -- a non-uniform rational B-spline curve.
//
//   degree p, n+1 control points P_i with weights w_i, knot vector
//   U = (u_0, ..., u_{n+p+1}). The curve is
//
//       C(u) = sum_i  w_i P_i N_{i,p}(u)  /  sum_i w_i N_{i,p}(u)
//
//   The implementation follows Piegl & Tiller, "The NURBS Book", §2.5
//   (algorithm A2.1 for FindSpan, A2.2 for BasisFuns, A4.1 for evaluation
//   of a rational curve).
//
//   All maths is done in metres (double). The fixed-point side of the
//   kernel converts to metres before calling and back after.
//
//   Storage layout: a single contiguous std::vector of homogeneous
//   coordinates (x*w, y*w, z*w, w), so each control point is 4 doubles.
//   Knot vector is a separate std::vector<double> of length n+p+2.
//

#include <cstddef>
#include <cstdint>
#include <vector>

#include "gmk/geom/curve.hpp"
#include "gmk/math/vec.hpp"
#include "gmk/result.hpp"

namespace gmk {

struct ControlPointH {
    double x{0}, y{0}, z{0}, w{1};  // homogeneous: stored as (xw, yw, zw, w)
};

class NurbsCurve final : public Curve {
public:
    NurbsCurve() = default;

    // Initialise. ``knots`` must be non-decreasing and have length
    // ``control_points.size() + degree + 1``. Weights must be strictly
    // positive. Returns InvalidArgument on any violation.
    Status init(int                    degree,
                const double*          knots, std::size_t n_knots,
                const Vec3d*           control_points,
                const double*          weights /* may be nullptr */,
                std::size_t            n_cp);

    int           degree()         const noexcept { return p_; }
    std::size_t   control_count()  const noexcept { return cps_.size(); }
    std::size_t   knot_count()     const noexcept { return knots_.size(); }
    const double* knots()          const noexcept { return knots_.data(); }
    const ControlPointH* cps()     const noexcept { return cps_.data(); }

    CurveKind kind()      const noexcept override { return CurveKind::Nurbs; }
    std::unique_ptr<Curve> clone() const override {
        return std::unique_ptr<Curve>(new NurbsCurve(*this));
    }
    double    u_min()     const noexcept override {
        return knots_.empty() ? 0.0 : knots_[static_cast<std::size_t>(p_)];
    }
    double    u_max()     const noexcept override {
        return knots_.empty() ? 0.0 : knots_[knots_.size() - 1 - static_cast<std::size_t>(p_)];
    }
    bool      is_closed() const noexcept override;

    Status    point(double u, Vec3d& out)                       const override;
    Status    point_and_tangent(double u, Vec3d&, Vec3d&)       const override;
    Status    derivatives(double u, int max_order, Vec3d* out, int n_out) const;

    // Knot insertion / refinement.
    Status insert_knot(double u_new, int multiplicity = 1);

private:
    // FindSpan (P&T A2.1).
    int find_span(double u) const noexcept;
    // BasisFuns (P&T A2.2). Writes ``p_+1`` basis values into ``N``.
    void basis_funs(int span, double u, double* N) const noexcept;
    // Derivatives of basis functions up to and including order ``d``.
    // Writes a (d+1) * (p_+1) row-major table into ``ders``.
    void basis_funs_derivs(int span, double u, int d, double* ders) const noexcept;

    int                          p_{0};
    std::vector<double>          knots_;
    std::vector<ControlPointH>   cps_;  // pre-multiplied by w
};

}  // namespace gmk
