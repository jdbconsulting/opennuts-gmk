#include "gmk/tess/tessellator.hpp"

#include <algorithm>
#include <cmath>

#include "gmk/geom/analytic_curves.hpp"
#include "gmk/geom/analytic_surfaces.hpp"
#include "gmk/geom/surface.hpp"

namespace gmk::tess {

namespace {

// Translate a chord tolerance into a segment count around a circle of
// radius r. The maximum chord-arc deviation for a regular N-gon
// inscribed in a circle of radius r is r*(1 - cos(π/N)); solving for N
// gives N = π / acos(1 - tol/r).
int circle_segments_for_tol(double radius, double span,
                            const TessOptions& opts) {
    if (radius <= 0.0) return opts.min_segments;
    double cos_arg = 1.0 - std::min(1.0, opts.chord_tolerance_m / radius);
    if (cos_arg < -1.0) cos_arg = -1.0;
    double dtheta = std::acos(cos_arg);
    int n = static_cast<int>(std::ceil(span / std::max(dtheta, 1e-9)));
    return std::clamp(std::max(n, opts.min_segments),
                      opts.min_segments, opts.max_segments);
}

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
            suggested = std::max(suggested,
                                 circle_segments_for_tol(sp.radius(), span, opts));
            break;
        }
        case SurfaceKind::Cylinder: {
            auto& cy = static_cast<const CylinderSurface&>(s);
            if (in_u) {
                suggested = std::max(suggested,
                                     circle_segments_for_tol(cy.radius(), span, opts));
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

// If `face` is a planar face whose outer loop is a single full-circle
// edge (and has no inner loops), return that CircleCurve. This is the
// shape produced by make_cylinder/make_cone for cap faces; tessellating
// such a face as the underlying square parametric domain leaves the
// solid with square ends.
const CircleCurve* face_disk_boundary(const brep::Body& body,
                                      const brep::Face& face) {
    const Surface* s = body.surface(face.surface);
    if (!s || s->kind() != SurfaceKind::Plane) return nullptr;
    if (!face.inner.empty()) return nullptr;
    const brep::Loop* outer = body.loop(face.outer);
    if (!outer || !outer->first.valid()) return nullptr;
    const brep::Coedge* ce = body.coedge(outer->first);
    if (!ce) return nullptr;
    // The loop must be a single coedge (closed in a single circular edge).
    if (ce->next != outer->first) return nullptr;
    const brep::Edge* e = body.edge(ce->edge);
    if (!e) return nullptr;
    const Curve* c = body.curve(e->curve);
    if (!c || c->kind() != CurveKind::Circle) return nullptr;
    // Make sure the edge covers the full circle so a closed disk is valid.
    constexpr double TWO_PI = 2.0 * PI;
    if (std::fabs((e->t_max - e->t_min) - TWO_PI) > 1e-9) return nullptr;
    return static_cast<const CircleCurve*>(c);
}

// Tessellate a planar disk face as a triangle fan from the centre to the
// rim. The rim is sampled in the *circle's* (xaxis, yaxis) frame so it
// coincides exactly with the rim vertices of any cylinder/cone lateral
// surface that shares the same circle frame and chord tolerance.
void tessellate_disk(const brep::Face& face,
                     const PlaneSurface& plane,
                     const CircleCurve& circle,
                     const TessOptions& opts,
                     FaceMesh& fm) {
    const int n = circle_segments_for_tol(circle.radius(), 2.0 * PI, opts);

    Vec3d outward = plane.normal_vec();
    if (face.sense == brep::FaceSense::Reversed) {
        outward = Vec3d{-outward.x, -outward.y, -outward.z};
    }

    const Vec3d c  = circle.origin();
    const Vec3d xa = circle.xaxis();
    const Vec3d ya = circle.yaxis();
    const double r = circle.radius();

    fm.positions_m.reserve(static_cast<std::size_t>(n + 2));
    fm.normals.reserve(static_cast<std::size_t>(n + 2));
    fm.triangles.reserve(static_cast<std::size_t>(n) * 3u);

    fm.positions_m.push_back(c);
    fm.normals.push_back(outward);
    for (int i = 0; i <= n; ++i) {
        double theta = (2.0 * PI) * static_cast<double>(i) / static_cast<double>(n);
        double cs = std::cos(theta), sn = std::sin(theta);
        Vec3d p{ c.x + r * (cs * xa.x + sn * ya.x),
                 c.y + r * (cs * xa.y + sn * ya.y),
                 c.z + r * (cs * xa.z + sn * ya.z) };
        fm.positions_m.push_back(p);
        fm.normals.push_back(outward);
    }

    // Triangle fan: (centre=0, rim_i, rim_{i+1}). Winding chosen so the
    // resulting triangle normal matches the face's outward direction.
    // (centre, p_i, p_{i+1}) has normal +circle_normal (= xa × ya); flip
    // the order if that disagrees with the outward direction.
    const Vec3d fan_normal = xa.cross(ya);
    const bool flip = fan_normal.dot(outward) < 0.0;
    for (int i = 0; i < n; ++i) {
        std::uint32_t a = 0u;
        std::uint32_t b = static_cast<std::uint32_t>(1 + i);
        std::uint32_t d = static_cast<std::uint32_t>(1 + i + 1);
        fm.triangles.push_back(a);
        if (flip) {
            fm.triangles.push_back(d);
            fm.triangles.push_back(b);
        } else {
            fm.triangles.push_back(b);
            fm.triangles.push_back(d);
        }
    }
}

}  // namespace

Status tessellate(const brep::Body& body, const TessOptions& opts, BodyMesh& out) {
    out = BodyMesh{};

    body.for_each_face([&](brep::FaceId fh, const brep::Face& face) {
        const Surface* s = body.surface(face.surface);
        if (!s) return;

        FaceMesh fm;
        fm.face = fh;

        // Special case: planar face whose outer loop is a single full
        // circular edge (cylinder/cone caps). Tessellate as a triangle
        // fan disk whose rim shares vertices with the lateral surface
        // tessellation, so the resulting solid is fully closed.
        if (const CircleCurve* circle = face_disk_boundary(body, face)) {
            const auto& plane = static_cast<const PlaneSurface&>(*s);
            tessellate_disk(face, plane, *circle, opts, fm);
            out.faces.push_back(std::move(fm));
            return;
        }

        int nu = estimate_segments(*s, true,  opts);
        int nv = estimate_segments(*s, false, opts);
        if (nu < 1) nu = 1;
        if (nv < 1) nv = 1;

        double u_min = s->u_min(), u_max = s->u_max();
        double v_min = s->v_min(), v_max = s->v_max();

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
