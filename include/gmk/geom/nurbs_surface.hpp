#pragma once
//
// gmk::NurbsSurface -- a non-uniform rational B-spline tensor-product surface.
//
//   S(u, v) = sum_i sum_j  w_ij P_ij N_{i,p}(u) M_{j,q}(v)
//           / sum_i sum_j  w_ij        N_{i,p}(u) M_{j,q}(v)
//
//   ``p`` is the degree in u and ``q`` is the degree in v. Control points
//   form a (n+1) x (m+1) grid, stored row-major in a single contiguous
//   vector of homogeneous coordinates (x*w, y*w, z*w, w).
//
//   Implementation follows P&T algorithms A3.5 (point) and A4.4 (rational).
//

#include <cstddef>
#include <vector>

#include "gmk/geom/nurbs_curve.hpp"     // for ControlPointH
#include "gmk/geom/surface.hpp"

namespace gmk {

class NurbsSurface final : public Surface {
public:
    NurbsSurface() = default;

    // Initialise. ``ku`` length = n_u + p + 1, ``kv`` length = n_v + q + 1.
    // Control points are row-major: cp[i*n_v + j].
    Status init(int p, int q,
                const double* ku, std::size_t n_ku,
                const double* kv, std::size_t n_kv,
                const Vec3d*  cp,
                const double* w  /* may be nullptr */,
                std::size_t   n_u, std::size_t n_v);

    SurfaceKind kind() const noexcept override { return SurfaceKind::Nurbs; }
    std::unique_ptr<Surface> clone() const override {
        return std::unique_ptr<Surface>(new NurbsSurface(*this));
    }
    int  degree_u() const noexcept { return p_; }
    int  degree_v() const noexcept { return q_; }
    std::size_t n_u() const noexcept { return n_u_; }
    std::size_t n_v() const noexcept { return n_v_; }

    double u_min() const noexcept override {
        return ku_.empty() ? 0.0 : ku_[static_cast<std::size_t>(p_)];
    }
    double u_max() const noexcept override {
        return ku_.empty() ? 0.0 : ku_[ku_.size() - 1 - static_cast<std::size_t>(p_)];
    }
    double v_min() const noexcept override {
        return kv_.empty() ? 0.0 : kv_[static_cast<std::size_t>(q_)];
    }
    double v_max() const noexcept override {
        return kv_.empty() ? 0.0 : kv_[kv_.size() - 1 - static_cast<std::size_t>(q_)];
    }

    Status point(double u, double v, Vec3d& out)                            const override;
    Status point_and_partials(double u, double v, Vec3d&, Vec3d&, Vec3d&)   const override;

private:
    int find_span(const std::vector<double>& U, int n, int deg, double t) const noexcept;
    void basis(const std::vector<double>& U, int i, int deg, double t, double* N) const noexcept;
    void basis_derivs(const std::vector<double>& U, int i, int deg, double t,
                      int d, double* ders) const noexcept;

    int p_{0}, q_{0};
    std::vector<double> ku_, kv_;
    std::vector<ControlPointH> cps_;  // row-major (n_u_ rows of n_v_)
    std::size_t n_u_{0}, n_v_{0};
};

}  // namespace gmk
