#pragma once
//
// gmk::Curve -- abstract 3D curve.
//
// Every curve has a parametric domain ``[u0, u1]`` (in dimensionless
// parameter space) and exposes three queries:
//
//   evaluate(u, out_p)             -- point on the curve at u
//   evaluate_d(u, out_p, out_t)    -- point + first derivative
//   evaluate_dd(u, out_p, out_t, out_n) -- point + first + second derivatives
//
// Concrete implementations may overload the most efficient form and the
// base class fills in the others via finite differences if necessary.
//
// All coordinates are returned in metres (double). The reason we don't
// return Vec3i directly is that curves are used inside numerical routines
// (closest-point, intersection, tessellation) that need real-valued maths;
// callers convert back to fm at the boundary.
//

#include <cstdint>
#include <memory>

#include "gmk/math/vec.hpp"
#include "gmk/result.hpp"

namespace gmk {

enum class CurveKind : std::uint8_t {
    Line,
    Circle,
    Ellipse,
    Nurbs,
};

class Curve {
public:
    virtual ~Curve() = default;

    virtual CurveKind kind()      const noexcept = 0;
    virtual double    u_min()     const noexcept = 0;
    virtual double    u_max()     const noexcept = 0;
    virtual bool      is_closed() const noexcept { return false; }

    // Deep copy of this curve. Concrete subclasses implement by returning
    // ``std::make_unique<Subclass>(*this)``.
    virtual std::unique_ptr<Curve> clone() const = 0;

    // Point at parameter u (metres).
    virtual Status point(double u, Vec3d& out) const = 0;

    // Point + tangent (not normalised).
    virtual Status point_and_tangent(double u, Vec3d& out_p, Vec3d& out_t) const {
        // Default: forward/backward differencing.
        Status s = point(u, out_p);
        if (s != Status::Ok) return s;
        double h = 1e-7 * (u_max() - u_min());
        Vec3d p1{}, p0{};
        double um = u_max(), un = u_min();
        double uf = (u + h <= um) ? u + h : u;
        double ub = (u - h >= un) ? u - h : u;
        if (point(uf, p1) != Status::Ok || point(ub, p0) != Status::Ok) {
            return Status::DidNotConverge;
        }
        double inv = 1.0 / (uf - ub);
        out_t = Vec3d{(p1.x-p0.x)*inv, (p1.y-p0.y)*inv, (p1.z-p0.z)*inv};
        return Status::Ok;
    }
};

}  // namespace gmk
