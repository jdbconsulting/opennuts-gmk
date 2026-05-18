#include "gmk/brep/primitives.hpp"

#include <array>
#include <memory>

#include "gmk/geom/analytic_curves.hpp"
#include "gmk/geom/analytic_surfaces.hpp"

namespace gmk::brep {

// Helper: assert the body is empty.
static bool body_is_empty(const Body& b) {
    return b.vertex_count() == 0 && b.edge_count() == 0 &&
           b.face_count() == 0   && b.shell_count() == 0;
}

// ---------------------------------------------------------------------------
// Box. 6 faces, 12 edges, 8 vertices. Each face is a planar surface
// bounded by 4 line edges in a single outer loop.
// ---------------------------------------------------------------------------
Status make_box(Body& body, Vec3i c, length_t hx, length_t hy, length_t hz) {
    if (!body_is_empty(body))            return Status::InvalidState;
    if (hx <= 0 || hy <= 0 || hz <= 0)   return Status::DegenerateInput;

    body.set_kind(BodyKind::Solid);

    // 8 corner vertices, indexed by bits (x,y,z) of i.
    std::array<VertexId, 8> V{};
    for (int i = 0; i < 8; ++i) {
        Vec3i p{
            c.x + ((i & 1) ? hx : -hx),
            c.y + ((i & 2) ? hy : -hy),
            c.z + ((i & 4) ? hz : -hz),
        };
        V[static_cast<std::size_t>(i)] = body.new_vertex(p);
    }

    // 12 edges. We index them as (corner_a, corner_b). For each edge we
    // create a line curve.
    struct EdgePair { int a, b; };
    constexpr std::array<EdgePair, 12> EDGES{{
        {0,1}, {2,3}, {4,5}, {6,7},        // along +x
        {0,2}, {1,3}, {4,6}, {5,7},        // along +y
        {0,4}, {1,5}, {2,6}, {3,7},        // along +z
    }};
    std::array<EdgeId, 12> E{};
    std::array<CurveId, 12> EC{};
    for (std::size_t i = 0; i < 12; ++i) {
        Vec3d p0 = to_vec3d_m(body.vertex(V[static_cast<std::size_t>(EDGES[i].a)])->position);
        Vec3d p1 = to_vec3d_m(body.vertex(V[static_cast<std::size_t>(EDGES[i].b)])->position);
        Vec3d d  = p1 - p0;
        auto line = std::make_unique<LineCurve>();
        Status s = line->init(p0, d, 0.0, 1.0);
        if (s != Status::Ok) return s;
        EC[i] = body.add_curve(std::move(line));
        E[i]  = body.new_edge(EC[i],
                              V[static_cast<std::size_t>(EDGES[i].a)],
                              V[static_cast<std::size_t>(EDGES[i].b)], 0.0, 1.0);
    }

    // 6 faces. For each face: pick the surface plane (centre + axes),
    // its outer loop is 4 coedges around the 4 boundary edges, with the
    // sense chosen so the outer loop is ccw with respect to the *outward*
    // normal.
    //
    // Face index   normal              vertex order (ccw from outside)
    //   0 -X        (-1, 0, 0)         0,2,6,4    (-x face)
    //   1 +X        (+1, 0, 0)         1,5,7,3    (+x face)
    //   2 -Y        (0, -1, 0)         0,4,5,1    (-y face)
    //   3 +Y        (0, +1, 0)         2,3,7,6    (+y face)
    //   4 -Z        (0, 0, -1)         0,1,3,2    (-z face)
    //   5 +Z        (0, 0, +1)         4,6,7,5    (+z face)
    struct FaceSpec {
        Vec3d centre_offset;
        Vec3d xaxis, yaxis;
        std::array<int, 4> verts;  // ccw from outside
    };
    const double dhx = length_to_m(hx);
    const double dhy = length_to_m(hy);
    const double dhz = length_to_m(hz);
    const Vec3d Cm   = to_vec3d_m(c);

    const std::array<FaceSpec, 6> FACES{{
        // -X face: outward normal -X. local xaxis = +Y, yaxis = +Z.
        {Vec3d{-dhx, 0, 0},  Vec3d{0,1,0}, Vec3d{0,0,1}, {0,2,6,4}},
        // +X face: outward +X. local xaxis = +Z, yaxis = +Y.
        {Vec3d{ dhx, 0, 0},  Vec3d{0,0,1}, Vec3d{0,1,0}, {1,5,7,3}},
        // -Y face: outward -Y. local xaxis = +Z, yaxis = +X.
        {Vec3d{0, -dhy, 0},  Vec3d{0,0,1}, Vec3d{1,0,0}, {0,4,5,1}},
        // +Y face: outward +Y. local xaxis = +X, yaxis = +Z.
        {Vec3d{0,  dhy, 0},  Vec3d{1,0,0}, Vec3d{0,0,1}, {2,3,7,6}},
        // -Z face: outward -Z. local xaxis = +X, yaxis = +Y.
        {Vec3d{0, 0, -dhz},  Vec3d{1,0,0}, Vec3d{0,1,0}, {0,1,3,2}},
        // +Z face: outward +Z. local xaxis = +Y, yaxis = +X.
        {Vec3d{0, 0,  dhz},  Vec3d{0,1,0}, Vec3d{1,0,0}, {4,6,7,5}},
    }};

    ShellId shell = body.new_shell(ShellKind::Closed);

    // Helper: find the edge index that connects vertex slot a to slot b
    // (either direction) and report whether the (a,b) ordering matches the
    // edge's stored direction.
    auto find_edge = [&](int a, int b, CoedgeSense& out_sense) -> int {
        for (std::size_t i = 0; i < 12; ++i) {
            if (EDGES[i].a == a && EDGES[i].b == b) { out_sense = CoedgeSense::SameAsCurve; return static_cast<int>(i); }
            if (EDGES[i].a == b && EDGES[i].b == a) { out_sense = CoedgeSense::Reversed;     return static_cast<int>(i); }
        }
        return -1;
    };

    for (std::size_t fi = 0; fi < 6; ++fi) {
        const FaceSpec& fs = FACES[fi];
        Vec3d origin{ Cm.x + fs.centre_offset.x,
                      Cm.y + fs.centre_offset.y,
                      Cm.z + fs.centre_offset.z };
        // Plane domain spans the box's extents on its face.
        double u_extent = (fs.xaxis.x != 0.0) ? dhx : (fs.xaxis.y != 0.0) ? dhy : dhz;
        double v_extent = (fs.yaxis.x != 0.0) ? dhx : (fs.yaxis.y != 0.0) ? dhy : dhz;
        auto pl = std::make_unique<PlaneSurface>();
        Status s = pl->init(origin, fs.xaxis, fs.yaxis,
                            -u_extent, u_extent, -v_extent, v_extent);
        if (s != Status::Ok) return s;
        SurfaceId sid = body.add_surface(std::move(pl));
        FaceId    f   = body.new_face(sid, FaceSense::SameAsSurface);
        LoopId    l   = body.new_loop(f, LoopKind::Outer);
        body.add_outer_loop_to_face(f, l);
        body.add_face_to_shell(shell, f);

        std::array<CoedgeId, 4> CE{};
        for (int i = 0; i < 4; ++i) {
            int va = fs.verts[static_cast<std::size_t>(i)];
            int vb = fs.verts[static_cast<std::size_t>((i + 1) % 4)];
            CoedgeSense sense;
            int eidx = find_edge(va, vb, sense);
            CE[static_cast<std::size_t>(i)] = body.new_coedge(E[static_cast<std::size_t>(eidx)], l, sense);
        }
        Status ls = body.link_loop(CE.data(), CE.size());
        if (ls != Status::Ok) return ls;
    }

    // Coedge mating: each box edge appears in exactly two coedges, one on
    // each adjacent face. We walk all coedges and pair them by edge id.
    // Since the loop linker has already populated every coedge, walking
    // the edges and finding their (up to) two coedges is enough.
    body.for_each_edge([&](EdgeId eh, Edge&) {
        CoedgeId found[2] = {};
        int      n = 0;
        body.coedge(CoedgeId{});  // suppress unused warning
        // Linear scan through coedges. Box has 24 coedges, so this is fine.
        // For larger bodies we'd keep a per-edge coedge list.
        // (Quadratic in O(coedges) -- acceptable here.)
        for (std::size_t slot = 1; slot < body.coedge_count() + 2; ++slot) {
            // Skip; can't iterate by slot via public API. Use for_each
            // pattern from outside the for_each_edge instead.
            (void)slot;
            break;
        }
        (void)eh; (void)found; (void)n;
    });
    // Properly do the matching now in a second pass:
    {
        std::vector<std::vector<CoedgeId>> per_edge(body.edge_count() * 2 + 4);
        // Use a flat map from edge.index -> list of coedge ids.
        // First collect.
        std::vector<std::pair<EdgeId, CoedgeId>> all;
        all.reserve(64);
        // We need a flexible iteration; reuse for_each.
        // Capture a reference to Body's coedge pool via a helper lambda.
        body.for_each_face([&](FaceId, Face& f) {
            // Walk the outer loop ring.
            LoopId lp = f.outer;
            Loop* L = body.loop(lp);
            if (!L) return;
            CoedgeId start = L->first;
            CoedgeId cur   = start;
            for (;;) {
                Coedge* c = body.coedge(cur);
                if (!c) break;
                all.push_back({c->edge, cur});
                cur = c->next;
                if (cur == start) break;
            }
        });
        for (std::size_t i = 0; i < all.size(); ++i) {
            for (std::size_t j = i + 1; j < all.size(); ++j) {
                if (all[i].first == all[j].first) {
                    body.mate_coedges(all[i].second, all[j].second);
                }
            }
        }
    }

    // Vertex.outgoing pointers: pick any coedge whose edge starts at the
    // vertex (in coedge-sense terms).
    body.for_each_face([&](FaceId, Face& f) {
        LoopId lp = f.outer;
        Loop* L = body.loop(lp);
        if (!L) return;
        CoedgeId start = L->first; CoedgeId cur = start;
        for (;;) {
            Coedge* c = body.coedge(cur);
            if (!c) break;
            Edge* e = body.edge(c->edge);
            if (e) {
                VertexId vs = (c->sense == CoedgeSense::SameAsCurve) ? e->start : e->end;
                Vertex* V = body.vertex(vs);
                if (V && !V->outgoing.valid()) V->outgoing = cur;
            }
            cur = c->next;
            if (cur == start) break;
        }
    });

    body.register_shell(shell);
    body.mark_bbox_dirty();
    return Status::Ok;
}

// ---------------------------------------------------------------------------
// Sphere. One face, surface = SphereSurface, no edges (sphere parametric
// domain is u-periodic and clamped at poles, but we don't model the poles
// as explicit singular vertices in this minimal builder). The face's outer
// loop is empty -- callers that need explicit pole vertices should refine
// later.
// ---------------------------------------------------------------------------
Status make_sphere(Body& body, Vec3i c, length_t r) {
    if (!body_is_empty(body))     return Status::InvalidState;
    if (r <= 0)                   return Status::DegenerateInput;

    body.set_kind(BodyKind::Solid);
    auto sph = std::make_unique<SphereSurface>();
    Status s = sph->init(to_vec3d_m(c), Vec3d{0,0,1}, Vec3d{1,0,0}, length_to_m(r));
    if (s != Status::Ok) return s;
    SurfaceId sid = body.add_surface(std::move(sph));
    FaceId    fid = body.new_face(sid, FaceSense::SameAsSurface);
    // Empty outer loop -- the surface is naturally closed in u and clamped in v.
    LoopId    lid = body.new_loop(fid, LoopKind::Outer);
    body.add_outer_loop_to_face(fid, lid);
    ShellId   shell = body.new_shell(ShellKind::Closed);
    body.add_face_to_shell(shell, fid);
    body.register_shell(shell);
    body.mark_bbox_dirty();
    // Track an AABB explicitly because the body has no vertices.
    // We do this by adding two helper vertices on the bbox; they're useful
    // for downstream operations (bounding, tessellation seed points).
    body.new_vertex(Vec3i{c.x - r, c.y - r, c.z - r});
    body.new_vertex(Vec3i{c.x + r, c.y + r, c.z + r});
    return Status::Ok;
}

// ---------------------------------------------------------------------------
// Cylinder. Three faces: a lateral (cylindrical) face plus two circular
// caps. Two circular edges (top and bottom rim), two vertices on the seam
// of the lateral face.
// ---------------------------------------------------------------------------
Status make_cylinder(Body& body, Vec3i base, Vec3d axis, length_t r, length_t height) {
    if (!body_is_empty(body))     return Status::InvalidState;
    if (r <= 0 || height <= 0)    return Status::DegenerateInput;
    if (axis.norm_sq() < 1e-30)   return Status::DegenerateInput;

    body.set_kind(BodyKind::Solid);
    Vec3d ax_unit = axis.normalized();
    Vec3d xref{1, 0, 0};
    if (std::fabs(ax_unit.dot(xref)) > 0.99) xref = Vec3d{0, 1, 0};

    const Vec3d base_m = to_vec3d_m(base);
    const double h_m   = length_to_m(height);
    const double r_m   = length_to_m(r);
    const Vec3d top_m  = base_m + ax_unit * h_m;

    // Surfaces.
    auto lat = std::make_unique<CylinderSurface>();
    Status s = lat->init(base_m, ax_unit, xref, r_m, 0.0, h_m);
    if (s != Status::Ok) return s;
    SurfaceId lat_sid = body.add_surface(std::move(lat));

    Vec3d xax, yax;
    {
        // Same orthonormalisation as cylinder.
        double an = ax_unit.norm();
        Vec3d axu = ax_unit * (1.0 / an);
        double dxn = xref.dot(axu);
        xax = (Vec3d{xref.x - dxn*axu.x, xref.y - dxn*axu.y, xref.z - dxn*axu.z});
        xax = xax * (1.0 / xax.norm());
        yax = axu.cross(xax);
    }

    auto bot = std::make_unique<PlaneSurface>();
    // The bottom plane's normal must point downward (-axis) for outward.
    // PlaneSurface uses xaxis × yaxis as the normal; pass them in opposite
    // order to flip the normal.
    Status sb = bot->init(base_m, xax, Vec3d{-yax.x, -yax.y, -yax.z}, -r_m, r_m, -r_m, r_m);
    if (sb != Status::Ok) return sb;
    SurfaceId bot_sid = body.add_surface(std::move(bot));

    auto top = std::make_unique<PlaneSurface>();
    Status st = top->init(top_m, xax, yax, -r_m, r_m, -r_m, r_m);
    if (st != Status::Ok) return st;
    SurfaceId top_sid = body.add_surface(std::move(top));

    // Curves.
    auto bot_circle = std::make_unique<CircleCurve>();
    bot_circle->init(base_m, ax_unit, xax, r_m);
    CurveId bot_cid = body.add_curve(std::move(bot_circle));

    auto top_circle = std::make_unique<CircleCurve>();
    top_circle->init(top_m, ax_unit, xax, r_m);
    CurveId top_cid = body.add_curve(std::move(top_circle));

    // Seam line on the lateral surface (along axis at u=0). Used by the
    // lateral face's outer loop because cylinder is u-periodic.
    auto seam = std::make_unique<LineCurve>();
    seam->init(base_m + xax * r_m, ax_unit, 0.0, h_m);
    CurveId seam_cid = body.add_curve(std::move(seam));

    // Vertices on the seam.
    VertexId v_bot = body.new_vertex(Vec3i{
        base.x + static_cast<length_t>(std::llround(xax.x * r_m * 1e15)),
        base.y + static_cast<length_t>(std::llround(xax.y * r_m * 1e15)),
        base.z + static_cast<length_t>(std::llround(xax.z * r_m * 1e15)),
    });
    VertexId v_top = body.new_vertex(Vec3i{
        base.x + static_cast<length_t>(std::llround((xax.x * r_m + ax_unit.x * h_m) * 1e15)),
        base.y + static_cast<length_t>(std::llround((xax.y * r_m + ax_unit.y * h_m) * 1e15)),
        base.z + static_cast<length_t>(std::llround((xax.z * r_m + ax_unit.z * h_m) * 1e15)),
    });

    // Edges.
    EdgeId e_bot  = body.new_edge(bot_cid,  v_bot, v_bot, 0.0, 2.0 * PI);
    EdgeId e_top  = body.new_edge(top_cid,  v_top, v_top, 0.0, 2.0 * PI);
    EdgeId e_seam = body.new_edge(seam_cid, v_bot, v_top, 0.0, h_m);

    ShellId shell = body.new_shell(ShellKind::Closed);

    // Lateral face. Outer loop: seam-up, top-circle (rev), seam-down (rev),
    // bot-circle (forward). Concretely, four coedges around the rectangle
    // (u, v) in [0, 2π] x [0, h].
    {
        FaceId f = body.new_face(lat_sid, FaceSense::SameAsSurface);
        LoopId l = body.new_loop(f, LoopKind::Outer);
        body.add_outer_loop_to_face(f, l);
        body.add_face_to_shell(shell, f);

        // 4 coedges -- bottom circle (sense same), seam-up (rev), top
        // circle (rev), seam-up (same).
        CoedgeId ce[4];
        ce[0] = body.new_coedge(e_bot,  l, CoedgeSense::SameAsCurve);
        ce[1] = body.new_coedge(e_seam, l, CoedgeSense::SameAsCurve);
        ce[2] = body.new_coedge(e_top,  l, CoedgeSense::Reversed);
        ce[3] = body.new_coedge(e_seam, l, CoedgeSense::Reversed);
        body.link_loop(ce, 4);
    }
    // Bottom cap. Outer loop: bottom circle (rev) so the loop is ccw with
    // respect to the cap's outward normal (which points -axis).
    {
        FaceId f = body.new_face(bot_sid, FaceSense::SameAsSurface);
        LoopId l = body.new_loop(f, LoopKind::Outer);
        body.add_outer_loop_to_face(f, l);
        body.add_face_to_shell(shell, f);
        CoedgeId ce = body.new_coedge(e_bot, l, CoedgeSense::Reversed);
        body.link_loop(&ce, 1);
    }
    // Top cap.
    {
        FaceId f = body.new_face(top_sid, FaceSense::SameAsSurface);
        LoopId l = body.new_loop(f, LoopKind::Outer);
        body.add_outer_loop_to_face(f, l);
        body.add_face_to_shell(shell, f);
        CoedgeId ce = body.new_coedge(e_top, l, CoedgeSense::SameAsCurve);
        body.link_loop(&ce, 1);
    }

    body.register_shell(shell);
    body.mark_bbox_dirty();
    return Status::Ok;
}

// ---------------------------------------------------------------------------
// Cone/frustum.
// ---------------------------------------------------------------------------
Status make_cone(Body& body, Vec3i base, Vec3d axis,
                 length_t r_base, length_t r_top, length_t height) {
    if (!body_is_empty(body))                            return Status::InvalidState;
    if (height <= 0 || r_base < 0 || r_top < 0)          return Status::DegenerateInput;
    if (r_base == 0 && r_top == 0)                       return Status::DegenerateInput;
    if (axis.norm_sq() < 1e-30)                          return Status::DegenerateInput;

    body.set_kind(BodyKind::Solid);
    Vec3d ax = axis.normalized();
    Vec3d xref{1,0,0}; if (std::fabs(ax.dot(xref)) > 0.99) xref = Vec3d{0,1,0};
    Vec3d xa, ya;
    {
        Vec3d axu = ax;
        double dxn = xref.dot(axu);
        xa = (Vec3d{xref.x - dxn*axu.x, xref.y - dxn*axu.y, xref.z - dxn*axu.z});
        xa = xa * (1.0 / xa.norm());
        ya = axu.cross(xa);
    }

    Vec3d base_m = to_vec3d_m(base);
    double h_m   = length_to_m(height);
    double rb_m  = length_to_m(r_base);
    double rt_m  = length_to_m(r_top);
    Vec3d top_m  = base_m + ax * h_m;
    double half_angle = std::atan2(rt_m - rb_m, h_m);

    // Lateral cone surface.
    auto cs = std::make_unique<ConeSurface>();
    if (Status sx = cs->init(base_m, ax, xa, rb_m, half_angle, 0.0, h_m); sx != Status::Ok) return sx;
    SurfaceId lat_sid = body.add_surface(std::move(cs));

    ShellId shell = body.new_shell(ShellKind::Closed);

    // Caps.
    SurfaceId bot_sid{}, top_sid{};
    if (r_base > 0) {
        auto bot = std::make_unique<PlaneSurface>();
        Status sx = bot->init(base_m, xa, Vec3d{-ya.x,-ya.y,-ya.z}, -rb_m, rb_m, -rb_m, rb_m);
        if (sx != Status::Ok) return sx;
        bot_sid = body.add_surface(std::move(bot));
    }
    if (r_top > 0) {
        auto top = std::make_unique<PlaneSurface>();
        Status sx = top->init(top_m, xa, ya, -rt_m, rt_m, -rt_m, rt_m);
        if (sx != Status::Ok) return sx;
        top_sid = body.add_surface(std::move(top));
    }

    // Bottom circle.
    CurveId bot_cid{}, top_cid{};
    if (r_base > 0) {
        auto cb = std::make_unique<CircleCurve>();
        cb->init(base_m, ax, xa, rb_m);
        bot_cid = body.add_curve(std::move(cb));
    }
    if (r_top > 0) {
        auto ct = std::make_unique<CircleCurve>();
        ct->init(top_m, ax, xa, rt_m);
        top_cid = body.add_curve(std::move(ct));
    }

    // Seam.
    Vec3d seam_base = base_m + xa * rb_m;
    Vec3d seam_dir  = (top_m + xa * rt_m) - seam_base;
    auto seam = std::make_unique<LineCurve>();
    seam->init(seam_base, seam_dir, 0.0, 1.0);
    CurveId seam_cid = body.add_curve(std::move(seam));

    // Vertices on seam.
    auto vec3i_from = [](Vec3d v) {
        return Vec3i{ static_cast<length_t>(std::llround(v.x * 1e15)),
                      static_cast<length_t>(std::llround(v.y * 1e15)),
                      static_cast<length_t>(std::llround(v.z * 1e15)) };
    };
    VertexId v_bot = body.new_vertex(vec3i_from(seam_base));
    VertexId v_top = body.new_vertex(vec3i_from(seam_base + seam_dir));

    EdgeId e_bot{}, e_top{};
    if (bot_cid.valid()) e_bot = body.new_edge(bot_cid, v_bot, v_bot, 0.0, 2.0 * PI);
    if (top_cid.valid()) e_top = body.new_edge(top_cid, v_top, v_top, 0.0, 2.0 * PI);
    EdgeId e_seam = body.new_edge(seam_cid, v_bot, v_top, 0.0, 1.0);

    // Lateral face.
    {
        FaceId f = body.new_face(lat_sid, FaceSense::SameAsSurface);
        LoopId l = body.new_loop(f, LoopKind::Outer);
        body.add_outer_loop_to_face(f, l);
        body.add_face_to_shell(shell, f);

        std::vector<CoedgeId> ce;
        if (e_bot.valid())  ce.push_back(body.new_coedge(e_bot,  l, CoedgeSense::SameAsCurve));
        ce.push_back(body.new_coedge(e_seam, l, CoedgeSense::SameAsCurve));
        if (e_top.valid())  ce.push_back(body.new_coedge(e_top,  l, CoedgeSense::Reversed));
        ce.push_back(body.new_coedge(e_seam, l, CoedgeSense::Reversed));
        body.link_loop(ce.data(), ce.size());
    }
    if (e_bot.valid()) {
        FaceId f = body.new_face(bot_sid, FaceSense::SameAsSurface);
        LoopId l = body.new_loop(f, LoopKind::Outer);
        body.add_outer_loop_to_face(f, l);
        body.add_face_to_shell(shell, f);
        CoedgeId ce = body.new_coedge(e_bot, l, CoedgeSense::Reversed);
        body.link_loop(&ce, 1);
    }
    if (e_top.valid()) {
        FaceId f = body.new_face(top_sid, FaceSense::SameAsSurface);
        LoopId l = body.new_loop(f, LoopKind::Outer);
        body.add_outer_loop_to_face(f, l);
        body.add_face_to_shell(shell, f);
        CoedgeId ce = body.new_coedge(e_top, l, CoedgeSense::SameAsCurve);
        body.link_loop(&ce, 1);
    }
    body.register_shell(shell);
    body.mark_bbox_dirty();
    return Status::Ok;
}

// ---------------------------------------------------------------------------
// Torus.  Single face on a TorusSurface, doubly periodic so no edges in the
// minimal representation.  AABB anchors added similar to sphere.
// ---------------------------------------------------------------------------
Status make_torus(Body& body, Vec3i centre, Vec3d axis, length_t R, length_t r) {
    if (!body_is_empty(body)) return Status::InvalidState;
    if (R <= 0 || r <= 0 || R <= r) return Status::DegenerateInput;
    if (axis.norm_sq() < 1e-30) return Status::DegenerateInput;
    body.set_kind(BodyKind::Solid);
    Vec3d ax = axis.normalized();
    Vec3d xref{1,0,0}; if (std::fabs(ax.dot(xref)) > 0.99) xref = Vec3d{0,1,0};
    auto t = std::make_unique<TorusSurface>();
    Status s = t->init(to_vec3d_m(centre), ax, xref, length_to_m(R), length_to_m(r));
    if (s != Status::Ok) return s;
    SurfaceId sid = body.add_surface(std::move(t));
    FaceId    fid = body.new_face(sid, FaceSense::SameAsSurface);
    LoopId    lid = body.new_loop(fid, LoopKind::Outer);
    body.add_outer_loop_to_face(fid, lid);
    ShellId   sh  = body.new_shell(ShellKind::Closed);
    body.add_face_to_shell(sh, fid);
    body.register_shell(sh);

    length_t outer = R + r;
    body.new_vertex(Vec3i{centre.x - outer, centre.y - outer, centre.z - r});
    body.new_vertex(Vec3i{centre.x + outer, centre.y + outer, centre.z + r});
    body.mark_bbox_dirty();
    return Status::Ok;
}

}  // namespace gmk::brep
