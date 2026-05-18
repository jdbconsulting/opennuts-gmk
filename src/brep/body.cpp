#include "gmk/brep/body.hpp"

#include <utility>

namespace gmk::brep {

CurveId Body::add_curve(std::unique_ptr<Curve>&& c) {
    if (!c) return CurveId{};
    CurveId h = curves_.alloc();
    *curves_.get(h) = std::move(c);
    return h;
}
SurfaceId Body::add_surface(std::unique_ptr<Surface>&& s) {
    if (!s) return SurfaceId{};
    SurfaceId h = surfaces_.alloc();
    *surfaces_.get(h) = std::move(s);
    return h;
}
Curve* Body::curve_get_(CurveId h) {
    auto* slot = curves_.get(h);
    return slot ? slot->get() : nullptr;
}
const Curve* Body::curve_get_(CurveId h) const {
    auto* slot = curves_.get(h);
    return slot ? slot->get() : nullptr;
}
Surface* Body::surface_get_(SurfaceId h) {
    auto* slot = surfaces_.get(h);
    return slot ? slot->get() : nullptr;
}
const Surface* Body::surface_get_(SurfaceId h) const {
    auto* slot = surfaces_.get(h);
    return slot ? slot->get() : nullptr;
}

VertexId Body::new_vertex(Vec3i p) {
    VertexId h = vertices_.alloc();
    Vertex* v = vertices_.get(h);
    v->position = p;
    mark_bbox_dirty();
    return h;
}

EdgeId Body::new_edge(CurveId c, VertexId vs, VertexId ve, double t0, double t1) {
    EdgeId h = edges_.alloc();
    Edge* e = edges_.get(h);
    e->curve = c;
    e->start = vs;
    e->end   = ve;
    e->t_min = t0;
    e->t_max = t1;
    return h;
}

CoedgeId Body::new_coedge(EdgeId e, LoopId l, CoedgeSense sense) {
    CoedgeId h = coedges_.alloc();
    Coedge* c = coedges_.get(h);
    c->edge = e;
    c->loop = l;
    c->sense = sense;
    return h;
}

LoopId Body::new_loop(FaceId f, LoopKind k) {
    LoopId h = loops_.alloc();
    Loop* l = loops_.get(h);
    l->face = f;
    l->kind = k;
    return h;
}

FaceId Body::new_face(SurfaceId s, FaceSense sense) {
    FaceId h = faces_.alloc();
    Face* f = faces_.get(h);
    f->surface = s;
    f->sense = sense;
    return h;
}

ShellId Body::new_shell(ShellKind k) {
    ShellId h = shells_.alloc();
    Shell* s = shells_.get(h);
    s->kind = k;
    return h;
}

Status Body::link_loop(const CoedgeId* ces, std::size_t n) {
    if (!ces || n == 0) return Status::InvalidArgument;
    for (std::size_t i = 0; i < n; ++i) {
        Coedge* c = coedges_.get(ces[i]);
        if (!c) return Status::NotFound;
        c->prev = ces[(i + n - 1) % n];
        c->next = ces[(i + 1) % n];
    }
    // Cache the first coedge on the loop.
    Coedge* first = coedges_.get(ces[0]);
    if (first) {
        Loop* l = loops_.get(first->loop);
        if (l) l->first = ces[0];
    }
    return Status::Ok;
}

void Body::mate_coedges(CoedgeId a, CoedgeId b) noexcept {
    Coedge* ca = coedges_.get(a);
    Coedge* cb = coedges_.get(b);
    if (!ca || !cb) return;
    ca->partner = b;
    cb->partner = a;
    // Also wire the Edge's two coedge slots.
    if (ca->edge.valid()) {
        Edge* e = edges_.get(ca->edge);
        if (e) {
            if (!e->coedge_a.valid()) e->coedge_a = a;
            else if (e->coedge_a != a) e->coedge_b = a;
            if (!e->coedge_b.valid() && e->coedge_a != b) e->coedge_b = b;
        }
    }
}

void Body::add_face_to_shell(ShellId s, FaceId f) {
    Shell* sh = shells_.get(s); Face* fa = faces_.get(f);
    if (!sh || !fa) return;
    sh->faces.push_back(f);
    fa->shell = s;
}
void Body::add_outer_loop_to_face(FaceId f, LoopId l) {
    Face* fa = faces_.get(f); Loop* lo = loops_.get(l);
    if (!fa || !lo) return;
    fa->outer = l;
    lo->face = f;
    lo->kind = LoopKind::Outer;
}
void Body::add_inner_loop_to_face(FaceId f, LoopId l) {
    Face* fa = faces_.get(f); Loop* lo = loops_.get(l);
    if (!fa || !lo) return;
    fa->inner.push_back(l);
    lo->face = f;
    lo->kind = LoopKind::Inner;
}

AABB Body::bbox() const {
    if (!bbox_dirty_) return cached_bbox_;
    AABB box{};
    vertices_.for_each([&](VertexId, const Vertex& v) {
        box.include(v.position);
    });
    cached_bbox_ = box;
    bbox_dirty_  = false;
    return box;
}

// ---------------------------------------------------------------------------
// Deep clone. We walk geometry first, recording id -> id maps, then walk
// topology and remap every reference.
// ---------------------------------------------------------------------------
Body Body::clone() const {
    Body out;
    out.kind_ = kind_;

    // We keep ID maps in dense vectors indexed by source slot, because slot
    // indices are small integers. ``map[i]`` is the destination handle for
    // source slot ``i``.
    auto build_map = [](std::size_t n) { return std::vector<std::uint32_t>(n, 0); };

    std::vector<std::uint32_t> map_curve   = build_map(curves_.slot_count());
    std::vector<std::uint32_t> map_surface = build_map(surfaces_.slot_count());
    std::vector<std::uint32_t> map_vertex  = build_map(vertices_.slot_count());
    std::vector<std::uint32_t> map_edge    = build_map(edges_.slot_count());
    std::vector<std::uint32_t> map_coedge  = build_map(coedges_.slot_count());
    std::vector<std::uint32_t> map_loop    = build_map(loops_.slot_count());
    std::vector<std::uint32_t> map_face    = build_map(faces_.slot_count());
    std::vector<std::uint32_t> map_shell   = build_map(shells_.slot_count());

    // We also need the generation of the destination handle to remap fully.
    auto remap_curve = [&](CurveId h) -> CurveId {
        if (!h.valid() || h.index >= map_curve.size() || !map_curve[h.index]) return CurveId{};
        return out.curves_.handle_of_slot(map_curve[h.index]);
    };
    auto remap_surface = [&](SurfaceId h) -> SurfaceId {
        if (!h.valid() || h.index >= map_surface.size() || !map_surface[h.index]) return SurfaceId{};
        return out.surfaces_.handle_of_slot(map_surface[h.index]);
    };
    auto remap_vertex = [&](VertexId h) -> VertexId {
        if (!h.valid() || h.index >= map_vertex.size() || !map_vertex[h.index]) return VertexId{};
        return out.vertices_.handle_of_slot(map_vertex[h.index]);
    };
    auto remap_edge = [&](EdgeId h) -> EdgeId {
        if (!h.valid() || h.index >= map_edge.size() || !map_edge[h.index]) return EdgeId{};
        return out.edges_.handle_of_slot(map_edge[h.index]);
    };
    auto remap_coedge = [&](CoedgeId h) -> CoedgeId {
        if (!h.valid() || h.index >= map_coedge.size() || !map_coedge[h.index]) return CoedgeId{};
        return out.coedges_.handle_of_slot(map_coedge[h.index]);
    };
    auto remap_loop = [&](LoopId h) -> LoopId {
        if (!h.valid() || h.index >= map_loop.size() || !map_loop[h.index]) return LoopId{};
        return out.loops_.handle_of_slot(map_loop[h.index]);
    };
    auto remap_face = [&](FaceId h) -> FaceId {
        if (!h.valid() || h.index >= map_face.size() || !map_face[h.index]) return FaceId{};
        return out.faces_.handle_of_slot(map_face[h.index]);
    };
    auto remap_shell = [&](ShellId h) -> ShellId {
        if (!h.valid() || h.index >= map_shell.size() || !map_shell[h.index]) return ShellId{};
        return out.shells_.handle_of_slot(map_shell[h.index]);
    };

    // 1. Geometry.
    curves_.for_each([&](CurveId h, const std::unique_ptr<Curve>& cp) {
        if (cp) {
            CurveId nh = out.add_curve(cp->clone());
            map_curve[h.index] = nh.index;
        }
    });
    surfaces_.for_each([&](SurfaceId h, const std::unique_ptr<Surface>& sp) {
        if (sp) {
            SurfaceId nh = out.add_surface(sp->clone());
            map_surface[h.index] = nh.index;
        }
    });

    // 2. Vertices.
    vertices_.for_each([&](VertexId h, const Vertex& v) {
        VertexId nh = out.new_vertex(v.position);
        map_vertex[h.index] = nh.index;
    });
    // 3. Shells, faces, loops are allocated first (empty), then wired up
    //    after the coedges/edges so cross-references can be remapped.
    shells_.for_each([&](ShellId h, const Shell& s) {
        ShellId nh = out.new_shell(s.kind);
        map_shell[h.index] = nh.index;
    });
    faces_.for_each([&](FaceId h, const Face& f) {
        // surface remap requires the geometry maps to be built (done above).
        FaceId nh = out.new_face(remap_surface(f.surface), f.sense);
        map_face[h.index] = nh.index;
    });
    loops_.for_each([&](LoopId h, const Loop& l) {
        LoopId nh = out.new_loop(remap_face(l.face), l.kind);
        map_loop[h.index] = nh.index;
    });
    edges_.for_each([&](EdgeId h, const Edge& e) {
        EdgeId nh = out.new_edge(remap_curve(e.curve),
                                  remap_vertex(e.start), remap_vertex(e.end),
                                  e.t_min, e.t_max);
        map_edge[h.index] = nh.index;
    });
    coedges_.for_each([&](CoedgeId h, const Coedge& c) {
        CoedgeId nh = out.new_coedge(remap_edge(c.edge), remap_loop(c.loop), c.sense);
        map_coedge[h.index] = nh.index;
    });

    // 4. Now wire cross-references.
    vertices_.for_each([&](VertexId h, const Vertex& v) {
        Vertex* nv = out.vertex(remap_vertex(h));
        if (nv) nv->outgoing = remap_coedge(v.outgoing);
    });
    coedges_.for_each([&](CoedgeId h, const Coedge& c) {
        Coedge* nc = out.coedge(remap_coedge(h));
        if (nc) {
            nc->prev    = remap_coedge(c.prev);
            nc->next    = remap_coedge(c.next);
            nc->partner = remap_coedge(c.partner);
        }
    });
    edges_.for_each([&](EdgeId h, const Edge& e) {
        Edge* ne = out.edge(remap_edge(h));
        if (ne) {
            ne->coedge_a = remap_coedge(e.coedge_a);
            ne->coedge_b = remap_coedge(e.coedge_b);
        }
    });
    loops_.for_each([&](LoopId h, const Loop& l) {
        Loop* nl = out.loop(remap_loop(h));
        if (nl) nl->first = remap_coedge(l.first);
    });
    faces_.for_each([&](FaceId h, const Face& f) {
        Face* nf = out.face(remap_face(h));
        if (nf) {
            nf->outer = remap_loop(f.outer);
            nf->inner.reserve(f.inner.size());
            for (LoopId lid : f.inner) nf->inner.push_back(remap_loop(lid));
            nf->shell = remap_shell(f.shell);
        }
    });
    shells_.for_each([&](ShellId h, const Shell& s) {
        Shell* ns = out.shell(remap_shell(h));
        if (ns) {
            ns->faces.reserve(s.faces.size());
            for (FaceId fid : s.faces) ns->faces.push_back(remap_face(fid));
        }
    });

    for (ShellId sid : shell_list_) {
        out.shell_list_.push_back(remap_shell(sid));
    }
    out.bbox_dirty_ = true;
    return out;
}

}  // namespace gmk::brep
