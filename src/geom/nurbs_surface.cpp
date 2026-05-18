#include "gmk/geom/nurbs_surface.hpp"

#include <algorithm>
#include <cmath>

namespace gmk {

namespace {
constexpr int MAX_DEG = 8;
}

Status NurbsSurface::init(int p, int q,
                          const double* ku, std::size_t n_ku,
                          const double* kv, std::size_t n_kv,
                          const Vec3d*  cp,
                          const double* w,
                          std::size_t   n_u, std::size_t n_v) {
    if (p < 1 || p > MAX_DEG || q < 1 || q > MAX_DEG) return Status::OutOfRange;
    if (n_u < static_cast<std::size_t>(p + 1) ||
        n_v < static_cast<std::size_t>(q + 1))         return Status::DegenerateInput;
    if (n_ku != n_u + static_cast<std::size_t>(p) + 1) return Status::InvalidArgument;
    if (n_kv != n_v + static_cast<std::size_t>(q) + 1) return Status::InvalidArgument;
    if (!ku || !kv || !cp)                              return Status::InvalidArgument;

    for (std::size_t i = 1; i < n_ku; ++i) if (ku[i] < ku[i-1]) return Status::InvalidArgument;
    for (std::size_t i = 1; i < n_kv; ++i) if (kv[i] < kv[i-1]) return Status::InvalidArgument;
    if (w) {
        for (std::size_t i = 0; i < n_u*n_v; ++i)
            if (!(w[i] > 0.0)) return Status::InvalidArgument;
    }

    p_ = p; q_ = q;
    ku_.assign(ku, ku + n_ku);
    kv_.assign(kv, kv + n_kv);
    n_u_ = n_u; n_v_ = n_v;
    cps_.resize(n_u * n_v);
    for (std::size_t i = 0; i < n_u; ++i) {
        for (std::size_t j = 0; j < n_v; ++j) {
            std::size_t idx = i*n_v + j;
            double wi = w ? w[idx] : 1.0;
            cps_[idx] = ControlPointH{ cp[idx].x*wi, cp[idx].y*wi, cp[idx].z*wi, wi };
        }
    }
    return Status::Ok;
}

int NurbsSurface::find_span(const std::vector<double>& U, int n, int deg, double t) const noexcept {
    if (t >= U[static_cast<std::size_t>(n + 1)]) return n;
    if (t <= U[static_cast<std::size_t>(deg)])   return deg;
    int low = deg, high = n + 1, mid = (low + high) / 2;
    while (t < U[static_cast<std::size_t>(mid)] ||
           t >= U[static_cast<std::size_t>(mid + 1)]) {
        if (t < U[static_cast<std::size_t>(mid)]) high = mid;
        else                                       low  = mid;
        mid = (low + high) / 2;
    }
    return mid;
}

void NurbsSurface::basis(const std::vector<double>& U, int i, int deg, double t, double* N) const noexcept {
    double left[MAX_DEG + 1];
    double right[MAX_DEG + 1];
    N[0] = 1.0;
    for (int j = 1; j <= deg; ++j) {
        left[j]  = t - U[static_cast<std::size_t>(i + 1 - j)];
        right[j] = U[static_cast<std::size_t>(i + j)] - t;
        double saved = 0.0;
        for (int r = 0; r < j; ++r) {
            double tmp = N[r] / (right[r + 1] + left[j - r]);
            N[r] = saved + right[r + 1] * tmp;
            saved = left[j - r] * tmp;
        }
        N[j] = saved;
    }
}

void NurbsSurface::basis_derivs(const std::vector<double>& U, int i, int deg, double t,
                                int d, double* ders) const noexcept {
    double ndu[MAX_DEG + 1][MAX_DEG + 1];
    double a[2][MAX_DEG + 1];
    double left[MAX_DEG + 2];
    double right[MAX_DEG + 2];

    ndu[0][0] = 1.0;
    for (int j = 1; j <= deg; ++j) {
        left[j]  = t - U[static_cast<std::size_t>(i + 1 - j)];
        right[j] = U[static_cast<std::size_t>(i + j)] - t;
        double saved = 0.0;
        for (int r = 0; r < j; ++r) {
            ndu[j][r] = right[r + 1] + left[j - r];
            double tmp = ndu[r][j - 1] / ndu[j][r];
            ndu[r][j] = saved + right[r + 1] * tmp;
            saved = left[j - r] * tmp;
        }
        ndu[j][j] = saved;
    }
    for (int j = 0; j <= deg; ++j) ders[0 * (deg + 1) + j] = ndu[j][deg];

    for (int r = 0; r <= deg; ++r) {
        int s1 = 0, s2 = 1;
        a[0][0] = 1.0;
        for (int k = 1; k <= d; ++k) {
            double dk = 0.0;
            int rk = r - k;
            int pk = deg - k;
            if (r >= k) {
                a[s2][0] = a[s1][0] / ndu[pk + 1][rk];
                dk = a[s2][0] * ndu[rk][pk];
            }
            int j1 = (rk >= -1)       ? 1     : -rk;
            int j2 = (r - 1 <= pk)    ? k - 1 : deg - r;
            for (int j = j1; j <= j2; ++j) {
                a[s2][j] = (a[s1][j] - a[s1][j - 1]) / ndu[pk + 1][rk + j];
                dk += a[s2][j] * ndu[rk + j][pk];
            }
            if (r <= pk) {
                a[s2][k] = -a[s1][k - 1] / ndu[pk + 1][r];
                dk += a[s2][k] * ndu[r][pk];
            }
            ders[k * (deg + 1) + r] = dk;
            std::swap(s1, s2);
        }
    }
    int rfact = deg;
    for (int k = 1; k <= d; ++k) {
        for (int j = 0; j <= deg; ++j) {
            ders[k * (deg + 1) + j] *= static_cast<double>(rfact);
        }
        rfact *= (deg - k);
    }
}

Status NurbsSurface::point(double u, double v, Vec3d& out) const {
    if (cps_.empty()) return Status::InvalidState;
    if (u < u_min()) u = u_min(); if (u > u_max()) u = u_max();
    if (v < v_min()) v = v_min(); if (v > v_max()) v = v_max();

    int n = static_cast<int>(n_u_) - 1;
    int m = static_cast<int>(n_v_) - 1;
    int spu = find_span(ku_, n, p_, u);
    int spv = find_span(kv_, m, q_, v);
    double Nu[MAX_DEG + 1], Nv[MAX_DEG + 1];
    basis(ku_, spu, p_, u, Nu);
    basis(kv_, spv, q_, v, Nv);

    double cx = 0, cy = 0, cz = 0, cw = 0;
    for (int i = 0; i <= p_; ++i) {
        double cxi = 0, cyi = 0, czi = 0, cwi = 0;
        std::size_t row = static_cast<std::size_t>(spu - p_ + i);
        for (int j = 0; j <= q_; ++j) {
            const ControlPointH& P = cps_[row * n_v_ + static_cast<std::size_t>(spv - q_ + j)];
            cxi += Nv[j] * P.x; cyi += Nv[j] * P.y;
            czi += Nv[j] * P.z; cwi += Nv[j] * P.w;
        }
        cx += Nu[i] * cxi; cy += Nu[i] * cyi;
        cz += Nu[i] * czi; cw += Nu[i] * cwi;
    }
    if (std::fabs(cw) < 1e-30) return Status::Singular;
    out = Vec3d{ cx/cw, cy/cw, cz/cw };
    return Status::Ok;
}

Status NurbsSurface::point_and_partials(double u, double v,
                                        Vec3d& p, Vec3d& du, Vec3d& dv) const {
    if (cps_.empty()) return Status::InvalidState;
    if (u < u_min()) u = u_min(); if (u > u_max()) u = u_max();
    if (v < v_min()) v = v_min(); if (v > v_max()) v = v_max();

    int n = static_cast<int>(n_u_) - 1;
    int m = static_cast<int>(n_v_) - 1;
    int spu = find_span(ku_, n, p_, u);
    int spv = find_span(kv_, m, q_, v);
    double dNu[2 * (MAX_DEG + 1)];
    double dNv[2 * (MAX_DEG + 1)];
    basis_derivs(ku_, spu, p_, u, 1, dNu);
    basis_derivs(kv_, spv, q_, v, 1, dNv);

    // Compute homogeneous A and its first derivatives in u and v.
    double Ax = 0, Ay = 0, Az = 0, Aw = 0;
    double Ax_u = 0, Ay_u = 0, Az_u = 0, Aw_u = 0;
    double Ax_v = 0, Ay_v = 0, Az_v = 0, Aw_v = 0;

    for (int i = 0; i <= p_; ++i) {
        std::size_t row = static_cast<std::size_t>(spu - p_ + i);
        for (int j = 0; j <= q_; ++j) {
            const ControlPointH& P = cps_[row * n_v_ + static_cast<std::size_t>(spv - q_ + j)];
            double nu  = dNu[0*(p_+1) + i];
            double nu1 = dNu[1*(p_+1) + i];
            double nv  = dNv[0*(q_+1) + j];
            double nv1 = dNv[1*(q_+1) + j];

            double b   = nu  * nv;
            double bu  = nu1 * nv;
            double bv  = nu  * nv1;

            Ax   += b  * P.x; Ay   += b  * P.y; Az   += b  * P.z; Aw   += b  * P.w;
            Ax_u += bu * P.x; Ay_u += bu * P.y; Az_u += bu * P.z; Aw_u += bu * P.w;
            Ax_v += bv * P.x; Ay_v += bv * P.y; Az_v += bv * P.z; Aw_v += bv * P.w;
        }
    }
    if (std::fabs(Aw) < 1e-30) return Status::Singular;
    double iw = 1.0 / Aw;
    p  = Vec3d{ Ax*iw, Ay*iw, Az*iw };
    du = Vec3d{ (Ax_u - Aw_u*p.x)*iw, (Ay_u - Aw_u*p.y)*iw, (Az_u - Aw_u*p.z)*iw };
    dv = Vec3d{ (Ax_v - Aw_v*p.x)*iw, (Ay_v - Aw_v*p.y)*iw, (Az_v - Aw_v*p.z)*iw };
    return Status::Ok;
}

}  // namespace gmk
