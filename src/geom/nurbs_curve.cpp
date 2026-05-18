#include "gmk/geom/nurbs_curve.hpp"

#include <algorithm>
#include <cmath>

namespace gmk {

// ---------------------------------------------------------------------------
// Initialisation.
// ---------------------------------------------------------------------------
Status NurbsCurve::init(int degree,
                        const double* knots, std::size_t n_knots,
                        const Vec3d* cp,
                        const double* w,
                        std::size_t n_cp) {
    if (degree < 1 || degree > 8)              return Status::OutOfRange;
    if (n_cp < static_cast<std::size_t>(degree + 1))
                                                return Status::DegenerateInput;
    if (n_knots != n_cp + static_cast<std::size_t>(degree) + 1)
                                                return Status::InvalidArgument;
    if (!knots || !cp)                          return Status::InvalidArgument;

    // Knots must be non-decreasing.
    for (std::size_t i = 1; i < n_knots; ++i) {
        if (knots[i] < knots[i-1]) return Status::InvalidArgument;
    }
    // Weights must be strictly positive (allow nullptr -> all 1).
    if (w) {
        for (std::size_t i = 0; i < n_cp; ++i) {
            if (!(w[i] > 0.0)) return Status::InvalidArgument;
        }
    }

    p_ = degree;
    knots_.assign(knots, knots + n_knots);
    cps_.resize(n_cp);
    for (std::size_t i = 0; i < n_cp; ++i) {
        double wi = w ? w[i] : 1.0;
        cps_[i] = ControlPointH{ cp[i].x * wi, cp[i].y * wi, cp[i].z * wi, wi };
    }
    return Status::Ok;
}

bool NurbsCurve::is_closed() const noexcept {
    if (cps_.size() < 2) return false;
    // Compare the first and last *de-homogenised* control points.
    const auto& a = cps_.front();
    const auto& b = cps_.back();
    double ax = a.x / a.w, ay = a.y / a.w, az = a.z / a.w;
    double bx = b.x / b.w, by = b.y / b.w, bz = b.z / b.w;
    double eps = 1e-12;
    return std::fabs(ax-bx) < eps && std::fabs(ay-by) < eps && std::fabs(az-bz) < eps;
}

// ---------------------------------------------------------------------------
// FindSpan -- Piegl & Tiller A2.1, with binary search.
// ---------------------------------------------------------------------------
int NurbsCurve::find_span(double u) const noexcept {
    int n = static_cast<int>(cps_.size()) - 1;
    if (u >= knots_[static_cast<std::size_t>(n + 1)]) return n;
    if (u <= knots_[static_cast<std::size_t>(p_)])    return p_;
    int low  = p_;
    int high = n + 1;
    int mid  = (low + high) / 2;
    while (u < knots_[static_cast<std::size_t>(mid)] ||
           u >= knots_[static_cast<std::size_t>(mid + 1)]) {
        if (u < knots_[static_cast<std::size_t>(mid)]) high = mid;
        else                                            low  = mid;
        mid = (low + high) / 2;
    }
    return mid;
}

// ---------------------------------------------------------------------------
// BasisFuns -- Piegl & Tiller A2.2.
// ---------------------------------------------------------------------------
void NurbsCurve::basis_funs(int i, double u, double* N) const noexcept {
    double left[16];
    double right[16];
    N[0] = 1.0;
    for (int j = 1; j <= p_; ++j) {
        left[j]  = u - knots_[static_cast<std::size_t>(i + 1 - j)];
        right[j] = knots_[static_cast<std::size_t>(i + j)] - u;
        double saved = 0.0;
        for (int r = 0; r < j; ++r) {
            double tmp = N[r] / (right[r + 1] + left[j - r]);
            N[r] = saved + right[r + 1] * tmp;
            saved = left[j - r] * tmp;
        }
        N[j] = saved;
    }
}

// ---------------------------------------------------------------------------
// Basis function derivatives -- Piegl & Tiller A2.3.
// Writes (d+1) rows of (p+1) values into ``ders`` (row-major).
// ---------------------------------------------------------------------------
void NurbsCurve::basis_funs_derivs(int i, double u, int d, double* ders) const noexcept {
    constexpr int MAX_DEG = 8;
    double ndu[MAX_DEG + 1][MAX_DEG + 1];
    double a[2][MAX_DEG + 1];
    double left[MAX_DEG + 2];
    double right[MAX_DEG + 2];

    ndu[0][0] = 1.0;
    for (int j = 1; j <= p_; ++j) {
        left[j]  = u - knots_[static_cast<std::size_t>(i + 1 - j)];
        right[j] = knots_[static_cast<std::size_t>(i + j)] - u;
        double saved = 0.0;
        for (int r = 0; r < j; ++r) {
            ndu[j][r] = right[r + 1] + left[j - r];
            double tmp = ndu[r][j - 1] / ndu[j][r];
            ndu[r][j] = saved + right[r + 1] * tmp;
            saved = left[j - r] * tmp;
        }
        ndu[j][j] = saved;
    }

    for (int j = 0; j <= p_; ++j) ders[0 * (p_ + 1) + j] = ndu[j][p_];

    for (int r = 0; r <= p_; ++r) {
        int s1 = 0, s2 = 1;
        a[0][0] = 1.0;
        for (int k = 1; k <= d; ++k) {
            double dk = 0.0;
            int    rk = r - k;
            int    pk = p_ - k;
            if (r >= k) {
                a[s2][0] = a[s1][0] / ndu[pk + 1][rk];
                dk = a[s2][0] * ndu[rk][pk];
            }
            int j1 = (rk >= -1)       ? 1     : -rk;
            int j2 = (r - 1 <= pk)    ? k - 1 : p_ - r;
            for (int j = j1; j <= j2; ++j) {
                a[s2][j] = (a[s1][j] - a[s1][j - 1]) / ndu[pk + 1][rk + j];
                dk += a[s2][j] * ndu[rk + j][pk];
            }
            if (r <= pk) {
                a[s2][k] = -a[s1][k - 1] / ndu[pk + 1][r];
                dk += a[s2][k] * ndu[r][pk];
            }
            ders[k * (p_ + 1) + r] = dk;
            std::swap(s1, s2);
        }
    }

    // Multiply by the factorial-like factor p!/(p-k)!.
    int rfact = p_;
    for (int k = 1; k <= d; ++k) {
        for (int j = 0; j <= p_; ++j) {
            ders[k * (p_ + 1) + j] *= static_cast<double>(rfact);
        }
        rfact *= (p_ - k);
    }
}

// ---------------------------------------------------------------------------
// Point evaluation (rational, Piegl & Tiller A4.1).
// ---------------------------------------------------------------------------
Status NurbsCurve::point(double u, Vec3d& out) const {
    if (cps_.empty()) return Status::InvalidState;
    double um0 = u_min(), um1 = u_max();
    if (u < um0) u = um0;
    if (u > um1) u = um1;
    int span = find_span(u);
    double N[16];
    basis_funs(span, u, N);

    double cx = 0, cy = 0, cz = 0, cw = 0;
    for (int j = 0; j <= p_; ++j) {
        const ControlPointH& P = cps_[static_cast<std::size_t>(span - p_ + j)];
        cx += N[j] * P.x;
        cy += N[j] * P.y;
        cz += N[j] * P.z;
        cw += N[j] * P.w;
    }
    if (std::fabs(cw) < 1e-30) return Status::Singular;
    out = Vec3d{cx / cw, cy / cw, cz / cw};
    return Status::Ok;
}

// ---------------------------------------------------------------------------
// Point + tangent (rational curve derivative).
// ---------------------------------------------------------------------------
Status NurbsCurve::point_and_tangent(double u, Vec3d& out_p, Vec3d& out_t) const {
    Vec3d ders[3];
    Status s = derivatives(u, 1, ders, 2);
    if (s != Status::Ok) return s;
    out_p = ders[0];
    out_t = ders[1];
    return Status::Ok;
}

// Compute up to ``max_order`` derivatives. ``n_out`` is at least max_order+1.
Status NurbsCurve::derivatives(double u, int d, Vec3d* out, int n_out) const {
    if (cps_.empty())               return Status::InvalidState;
    if (d < 0 || d > 2)             return Status::OutOfRange;  // up to 2nd order in this implementation
    if (n_out < d + 1)              return Status::InvalidArgument;
    if (d > p_) {
        for (int k = p_ + 1; k <= d; ++k) out[k] = Vec3d{0,0,0};
        d = p_;
    }
    double um0 = u_min(), um1 = u_max();
    if (u < um0) u = um0;
    if (u > um1) u = um1;

    int span = find_span(u);
    constexpr int MAX_DEG = 8;
    double ders[3 * (MAX_DEG + 1)];
    basis_funs_derivs(span, u, d, ders);

    // Homogeneous Aders[k] = sum_j ders[k][j] * (w_i*P_i, w_i)
    Vec3d Ax[3]; double Aw[3];
    for (int k = 0; k <= d; ++k) { Ax[k] = Vec3d{}; Aw[k] = 0.0; }
    for (int k = 0; k <= d; ++k) {
        for (int j = 0; j <= p_; ++j) {
            const ControlPointH& P = cps_[static_cast<std::size_t>(span - p_ + j)];
            double b = ders[k * (p_ + 1) + j];
            Ax[k].x += b * P.x;
            Ax[k].y += b * P.y;
            Ax[k].z += b * P.z;
            Aw[k]   += b * P.w;
        }
    }

    if (std::fabs(Aw[0]) < 1e-30) return Status::Singular;

    // De-homogenise the derivatives. Piegl & Tiller A4.2 with binomial
    // coefficients computed inline (small d).
    out[0] = Ax[0] * (1.0 / Aw[0]);
    if (d >= 1) {
        Vec3d t = Vec3d{ Ax[1].x - Aw[1] * out[0].x,
                         Ax[1].y - Aw[1] * out[0].y,
                         Ax[1].z - Aw[1] * out[0].z };
        out[1] = t * (1.0 / Aw[0]);
    }
    if (d >= 2) {
        Vec3d v = Vec3d{ Ax[2].x - 2.0 * Aw[1] * out[1].x - Aw[2] * out[0].x,
                         Ax[2].y - 2.0 * Aw[1] * out[1].y - Aw[2] * out[0].y,
                         Ax[2].z - 2.0 * Aw[1] * out[1].z - Aw[2] * out[0].z };
        out[2] = v * (1.0 / Aw[0]);
    }
    return Status::Ok;
}

// ---------------------------------------------------------------------------
// Knot insertion -- Piegl & Tiller A5.1, ``s`` times.
// ---------------------------------------------------------------------------
Status NurbsCurve::insert_knot(double u_new, int times) {
    if (cps_.empty())              return Status::InvalidState;
    if (times <= 0)                return Status::Ok;
    if (u_new < u_min() || u_new > u_max()) return Status::OutOfRange;

    int n = static_cast<int>(cps_.size()) - 1;
    int k = find_span(u_new);
    int s = 0;
    for (int i = k; i > 0 && std::fabs(knots_[static_cast<std::size_t>(i)] - u_new) < 1e-15; --i) ++s;
    if (s + times > p_) return Status::InvalidArgument;  // would exceed multiplicity bound

    std::vector<ControlPointH> Q(static_cast<std::size_t>(n + 1 + times));
    std::vector<double>        UQ(knots_.size() + static_cast<std::size_t>(times));

    for (int i = 0; i <= k; ++i)         UQ[static_cast<std::size_t>(i)] = knots_[static_cast<std::size_t>(i)];
    for (int i = 1; i <= times; ++i)     UQ[static_cast<std::size_t>(k + i)] = u_new;
    for (std::size_t i = static_cast<std::size_t>(k + 1); i < knots_.size(); ++i)
        UQ[i + static_cast<std::size_t>(times)] = knots_[i];

    for (int i = 0; i <= k - p_; ++i)     Q[static_cast<std::size_t>(i)] = cps_[static_cast<std::size_t>(i)];
    for (int i = k - s; i <= n; ++i)      Q[static_cast<std::size_t>(i + times)] = cps_[static_cast<std::size_t>(i)];

    std::vector<ControlPointH> Rw(static_cast<std::size_t>(p_ - s + 1));
    for (int i = 0; i <= p_ - s; ++i) Rw[static_cast<std::size_t>(i)] = cps_[static_cast<std::size_t>(k - p_ + i)];

    int L = 0;
    for (int j = 1; j <= times; ++j) {
        L = k - p_ + j;
        for (int i = 0; i <= p_ - j - s; ++i) {
            double alpha = (u_new - knots_[static_cast<std::size_t>(L + i)]) /
                           (knots_[static_cast<std::size_t>(i + k + 1)] - knots_[static_cast<std::size_t>(L + i)]);
            Rw[static_cast<std::size_t>(i)].x = alpha * Rw[static_cast<std::size_t>(i + 1)].x + (1.0 - alpha) * Rw[static_cast<std::size_t>(i)].x;
            Rw[static_cast<std::size_t>(i)].y = alpha * Rw[static_cast<std::size_t>(i + 1)].y + (1.0 - alpha) * Rw[static_cast<std::size_t>(i)].y;
            Rw[static_cast<std::size_t>(i)].z = alpha * Rw[static_cast<std::size_t>(i + 1)].z + (1.0 - alpha) * Rw[static_cast<std::size_t>(i)].z;
            Rw[static_cast<std::size_t>(i)].w = alpha * Rw[static_cast<std::size_t>(i + 1)].w + (1.0 - alpha) * Rw[static_cast<std::size_t>(i)].w;
        }
        Q[static_cast<std::size_t>(L)]                   = Rw[0];
        Q[static_cast<std::size_t>(k + times - j - s)]   = Rw[static_cast<std::size_t>(p_ - j - s)];
    }
    for (int i = L + 1; i < k - s; ++i)
        Q[static_cast<std::size_t>(i)] = Rw[static_cast<std::size_t>(i - L)];

    knots_ = std::move(UQ);
    cps_   = std::move(Q);
    return Status::Ok;
}

}  // namespace gmk
