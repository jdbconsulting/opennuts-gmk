#include "gmk/tess/tessellator.hpp"

#include <algorithm>
#include <cmath>

#include "gmk/geom/analytic_surfaces.hpp"
#include "gmk/geom/surface.hpp"

namespace gmk::tess {

namespace {

// Estimate a reasonable segment count along one parametric direction. For
// analytic surfaces we know the local radius / extent and can compute a
// chord-bound directly; for general NURBS we fall back on a heuristic
// proportional to the surface span and degree.
int estimate_segments(const Surface& s, bool in_u, const TessOptions& opts) {
    double span = in_u ? (s.u_max() - s.u_min()) : (s.v_max() - s.v_min());

    int suggested = opts.min_segments;
    switch (s.kind()) {
        case SurfaceKind::Plane:
            suggested = std::max(suggested, 1);
            break;
        case SurfaceKind::Sphere: {
            // span is 2π in u, π in v -- a uniform grid gives a chord
            // error of ~r*(1 - cos(π/N)). Solve for N.
            auto& sp = static_cast<const SphereSurface&>(s);
            double r  = sp.radius();
            double target = opts.chord_tolerance_m;
            double cos_arg = 1.0 - std::min(1.0, target / r);
            if (cos_arg < -1.0) cos_arg = -1.0;
            double dtheta = std::acos(cos_arg);
            int n = static_cast<int>(std::ceil(span / std::max(dtheta, 1e-9)));
            suggested = std::max(suggested, n);
            break;
        }
        case SurfaceKind::Cylinder: {
            auto& cy = static_cast<const CylinderSurface&>(s);
            if (in_u) {
                double r = cy.radius();
                double cos_arg = 1.0 - std::min(1.0, opts.chord_tolerance_m / r);
                if (cos_arg < -1.0) cos_arg = -1.0;
                double dtheta = std::acos(cos_arg);
                int n = static_cast<int>(std::ceil(span / std::max(dtheta, 1e-9)));
                suggested = std::max(suggested, n);
            } else {
                suggested = std::max(suggested, 1);
            }
            break;
        }
        case SurfaceKind::Cone: {
            // Treat similarly to cylinder for the angular direction.
            if (in_u) suggested = std::max(suggested, 24);
            else      suggested = std::max(suggested, 2);
            break;
        }
        case SurfaceKind::Torus: {
            suggested = std::max(suggested, 32);
            break;
        }
        case SurfaceKind::Nurbs:
            // Use a generic 16 segments per parametric unit-span; users
            // can tighten via chord_tolerance.
            suggested = std::max(suggested, 32);
            break;
    }
    return std::clamp(suggested, opts.min_segments, opts.max_segments);
}

}  // namespace

Status tessellate(const brep::Body& body, const TessOptions& opts, BodyMesh& out) {
    out = BodyMesh{};

    body.for_each_face([&](brep::FaceId fh, const brep::Face& face) {
        const Surface* s = body.surface(face.surface);
        if (!s) return;
        int nu = estimate_segments(*s, true,  opts);
        int nv = estimate_segments(*s, false, opts);
        // Make sure the grid has at least one sample.
        if (nu < 1) nu = 1;
        if (nv < 1) nv = 1;

        double u_min = s->u_min(), u_max = s->u_max();
        double v_min = s->v_min(), v_max = s->v_max();

        FaceMesh fm;
        fm.face = fh;
        fm.positions_m.reserve(static_cast<std::size_t>((nu + 1) * (nv + 1)));
        fm.normals.reserve(static_cast<std::size_t>((nu + 1) * (nv + 1)));
        fm.triangles.reserve(static_cast<std::size_t>(nu * nv * 6));

        // Sample (nu+1) x (nv+1) grid.
        for (int j = 0; j <= nv; ++j) {
            double v = v_min + (v_max - v_min) * static_cast<double>(j) / nv;
            for (int i = 0; i <= nu; ++i) {
                double u = u_min + (u_max - u_min) * static_cast<double>(i) / nu;
                Vec3d p{}, n{};
                Status sp = s->point(u, v, p);
                Status sn = s->normal(u, v, n);
                if (sp != Status::Ok) p = Vec3d{};
                if (sn != Status::Ok) n = Vec3d{0, 0, 1};
                if (face.sense == brep::FaceSense::Reversed) n = Vec3d{-n.x, -n.y, -n.z};
                fm.positions_m.push_back(p);
                fm.normals.push_back(n);
            }
        }
        auto idx = [&](int i, int j) -> std::uint32_t {
            return static_cast<std::uint32_t>(j * (nu + 1) + i);
        };
        // Triangle pairs.
        for (int j = 0; j < nv; ++j) {
            for (int i = 0; i < nu; ++i) {
                std::uint32_t a = idx(i,   j);
                std::uint32_t b = idx(i+1, j);
                std::uint32_t c = idx(i+1, j+1);
                std::uint32_t d = idx(i,   j+1);
                if (face.sense == brep::FaceSense::Reversed) {
                    fm.triangles.push_back(a); fm.triangles.push_back(c); fm.triangles.push_back(b);
                    fm.triangles.push_back(a); fm.triangles.push_back(d); fm.triangles.push_back(c);
                } else {
                    fm.triangles.push_back(a); fm.triangles.push_back(b); fm.triangles.push_back(c);
                    fm.triangles.push_back(a); fm.triangles.push_back(c); fm.triangles.push_back(d);
                }
            }
        }
        out.faces.push_back(std::move(fm));
    });
    return Status::Ok;
}

Status flatten(BodyMesh& m) {
    if (m.flat_built) return Status::Ok;
    m.flat_positions_m.clear();
    m.flat_normals.clear();
    m.flat_triangles.clear();
    for (const auto& fm : m.faces) {
        std::uint32_t base = static_cast<std::uint32_t>(m.flat_positions_m.size());
        m.flat_positions_m.insert(m.flat_positions_m.end(),
                                  fm.positions_m.begin(), fm.positions_m.end());
        m.flat_normals.insert(m.flat_normals.end(),
                              fm.normals.begin(), fm.normals.end());
        m.flat_triangles.reserve(m.flat_triangles.size() + fm.triangles.size());
        for (std::uint32_t i : fm.triangles) m.flat_triangles.push_back(base + i);
    }
    m.flat_built = true;
    return Status::Ok;
}

}  // namespace gmk::tess
