#pragma once
//
// Analytic surfaces -- plane, sphere, cylinder, cone, torus. Each one has
// a closed-form Surface implementation and can be promoted to a NURBS
// surface for B-rep storage / algorithms that only understand NURBS.
//
// Parameterisations:
//
//   PlaneSurface     S(u, v) = origin + u*x + v*y       (no domain restriction)
//   SphereSurface    S(u, v) = origin + r*(cos v)*(cos u, sin u, 0)
//                              + r*(sin v)*axis
//                                 u in [0, 2π], v in [-π/2, π/2]
//   CylinderSurface  S(u, v) = origin + r*(cos u)*x + r*(sin u)*y + v*axis
//                                 u in [0, 2π], v in [v_min, v_max]
//   ConeSurface      S(u, v) = origin + (r + v*tan(half_angle))*(cos u)*x
//                              + (r + v*tan(half_angle))*(sin u)*y + v*axis
//                                 u in [0, 2π], v in [v_min, v_max]
//   TorusSurface     S(u, v) = origin + (R + r*cos v)*((cos u)*x + (sin u)*y)
//                              + r*(sin v)*axis
//                                 u, v in [0, 2π]
//

#include "gmk/geom/surface.hpp"
#include "gmk/math/vec.hpp"
#include "gmk/result.hpp"

namespace gmk {

class PlaneSurface final : public Surface {
public:
    PlaneSurface() = default;
    Status init(Vec3d origin, Vec3d x_axis, Vec3d y_axis,
                double u_min_, double u_max_, double v_min_, double v_max_);

    SurfaceKind kind() const noexcept override { return SurfaceKind::Plane; }
    double u_min() const noexcept override { return u0_; }
    double u_max() const noexcept override { return u1_; }
    double v_min() const noexcept override { return v0_; }
    double v_max() const noexcept override { return v1_; }
    std::unique_ptr<Surface> clone() const override {
        return std::unique_ptr<Surface>(new PlaneSurface(*this));
    }

    Status point(double u, double v, Vec3d& out) const override;
    Status point_and_partials(double u, double v, Vec3d&, Vec3d&, Vec3d&) const override;
    Status normal(double, double, Vec3d& out) const override { out = n_; return Status::Ok; }

    Vec3d origin() const noexcept { return o_; }
    Vec3d xaxis()  const noexcept { return x_; }
    Vec3d yaxis()  const noexcept { return y_; }
    Vec3d normal_vec() const noexcept { return n_; }

private:
    Vec3d o_{}, x_{1,0,0}, y_{0,1,0}, n_{0,0,1};
    double u0_{-1}, u1_{1}, v0_{-1}, v1_{1};
};

class SphereSurface final : public Surface {
public:
    SphereSurface() = default;
    Status init(Vec3d origin, Vec3d axis_z, Vec3d axis_x, double radius);

    SurfaceKind kind()    const noexcept override { return SurfaceKind::Sphere; }
    std::unique_ptr<Surface> clone() const override {
        return std::unique_ptr<Surface>(new SphereSurface(*this));
    }
    double u_min()        const noexcept override { return 0.0; }
    double u_max()        const noexcept override { return 2.0 * PI; }
    double v_min()        const noexcept override { return -PI/2.0; }
    double v_max()        const noexcept override { return  PI/2.0; }
    bool   u_periodic()   const noexcept override { return true; }

    Status point(double u, double v, Vec3d& out) const override;
    Status point_and_partials(double u, double v, Vec3d&, Vec3d&, Vec3d&) const override;

    double radius() const noexcept { return r_; }
    Vec3d origin() const noexcept { return o_; }

private:
    Vec3d o_{}, ax_{0,0,1}, ay_{0,1,0}, az_{1,0,0};
    double r_{1.0};
};

class CylinderSurface final : public Surface {
public:
    CylinderSurface() = default;
    Status init(Vec3d origin, Vec3d axis, Vec3d x_ref, double radius,
                double v_min_, double v_max_);

    SurfaceKind kind() const noexcept override { return SurfaceKind::Cylinder; }
    std::unique_ptr<Surface> clone() const override {
        return std::unique_ptr<Surface>(new CylinderSurface(*this));
    }
    double u_min() const noexcept override { return 0.0; }
    double u_max() const noexcept override { return 2.0 * PI; }
    double v_min() const noexcept override { return v0_; }
    double v_max() const noexcept override { return v1_; }
    bool   u_periodic() const noexcept override { return true; }

    Status point(double u, double v, Vec3d& out) const override;
    Status point_and_partials(double u, double v, Vec3d&, Vec3d&, Vec3d&) const override;

    double radius() const noexcept { return r_; }
    Vec3d origin()  const noexcept { return o_; }
    Vec3d axis()    const noexcept { return ax_; }

private:
    Vec3d o_{}, ax_{0,0,1}, x_{1,0,0}, y_{0,1,0};
    double r_{1.0};
    double v0_{0}, v1_{1};
};

class ConeSurface final : public Surface {
public:
    ConeSurface() = default;
    // half_angle is the half-angle of the cone in radians. Radius is the
    // radius at the base ring (v = 0).
    Status init(Vec3d origin, Vec3d axis, Vec3d x_ref,
                double radius_at_base, double half_angle,
                double v_min_, double v_max_);

    SurfaceKind kind() const noexcept override { return SurfaceKind::Cone; }
    std::unique_ptr<Surface> clone() const override {
        return std::unique_ptr<Surface>(new ConeSurface(*this));
    }
    double u_min() const noexcept override { return 0.0; }
    double u_max() const noexcept override { return 2.0 * PI; }
    double v_min() const noexcept override { return v0_; }
    double v_max() const noexcept override { return v1_; }
    bool   u_periodic() const noexcept override { return true; }

    Status point(double u, double v, Vec3d& out) const override;
    Status point_and_partials(double u, double v, Vec3d&, Vec3d&, Vec3d&) const override;

private:
    Vec3d o_{}, ax_{0,0,1}, x_{1,0,0}, y_{0,1,0};
    double r_base_{1.0}, half_angle_{0.0};
    double v0_{0}, v1_{1};
};

class TorusSurface final : public Surface {
public:
    TorusSurface() = default;
    // R is the major radius (centre of tube to centre of torus), r is the
    // tube (minor) radius.
    Status init(Vec3d origin, Vec3d axis, Vec3d x_ref, double R, double r);

    SurfaceKind kind() const noexcept override { return SurfaceKind::Torus; }
    std::unique_ptr<Surface> clone() const override {
        return std::unique_ptr<Surface>(new TorusSurface(*this));
    }
    double u_min() const noexcept override { return 0.0; }
    double u_max() const noexcept override { return 2.0 * PI; }
    double v_min() const noexcept override { return 0.0; }
    double v_max() const noexcept override { return 2.0 * PI; }
    bool   u_periodic() const noexcept override { return true; }
    bool   v_periodic() const noexcept override { return true; }

    Status point(double u, double v, Vec3d& out) const override;
    Status point_and_partials(double u, double v, Vec3d&, Vec3d&, Vec3d&) const override;

private:
    Vec3d o_{}, ax_{0,0,1}, x_{1,0,0}, y_{0,1,0};
    double R_{2.0}, r_{0.5};
};

}  // namespace gmk
