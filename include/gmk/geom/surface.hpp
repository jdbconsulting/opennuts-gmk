#pragma once
//
// gmk::Surface -- abstract bivariate parametric surface.
//
// Surfaces are parameterised by (u, v) on a rectangular domain
//      [u_min, u_max] x [v_min, v_max].
//
// Periodic surfaces (cylinder, sphere wrap, torus) report u_period() and/or
// v_period() so that tessellators and intersection algorithms know to stitch
// across the seam. The kernel does not collapse the parametrisation; a
// sphere has a singular pole at v_min and v_max instead.
//

#include <cstdint>
#include <memory>

#include "gmk/math/vec.hpp"
#include "gmk/result.hpp"

namespace gmk {

enum class SurfaceKind : std::uint8_t {
    Plane,
    Sphere,
    Cylinder,
    Cone,
    Torus,
    Nurbs,
};

class Surface {
public:
    virtual ~Surface() = default;

    virtual SurfaceKind kind() const noexcept = 0;
    virtual double u_min()     const noexcept = 0;
    virtual double u_max()     const noexcept = 0;
    virtual double v_min()     const noexcept = 0;
    virtual double v_max()     const noexcept = 0;
    virtual bool   u_periodic() const noexcept { return false; }
    virtual bool   v_periodic() const noexcept { return false; }

    virtual std::unique_ptr<Surface> clone() const = 0;

    // Surface point (metres).
    virtual Status point(double u, double v, Vec3d& out) const = 0;

    // Point + partial derivatives ∂S/∂u and ∂S/∂v. Default implementation
    // does central differencing; concrete surfaces override with the
    // analytic formula.
    virtual Status point_and_partials(double u, double v,
                                      Vec3d& p, Vec3d& du, Vec3d& dv) const {
        Status s = point(u, v, p);
        if (s != Status::Ok) return s;
        double hu = 1e-6 * (u_max() - u_min());
        double hv = 1e-6 * (v_max() - v_min());
        Vec3d a{}, b{};
        if (point(u + hu, v, a) != Status::Ok || point(u - hu, v, b) != Status::Ok)
            return Status::DidNotConverge;
        du = (a - b) * (0.5 / hu);
        if (point(u, v + hv, a) != Status::Ok || point(u, v - hv, b) != Status::Ok)
            return Status::DidNotConverge;
        dv = (a - b) * (0.5 / hv);
        return Status::Ok;
    }

    // Outward unit normal at (u, v). The orientation comes from
    // normalize(du × dv); subclasses can override if the underlying
    // parameterisation has a different convention.
    virtual Status normal(double u, double v, Vec3d& out) const {
        Vec3d p, du, dv;
        Status s = point_and_partials(u, v, p, du, dv);
        if (s != Status::Ok) return s;
        Vec3d n = du.cross(dv);
        double nn = n.norm();
        if (nn < 1e-30) return Status::Singular;
        out = n * (1.0 / nn);
        return Status::Ok;
    }
};

}  // namespace gmk
