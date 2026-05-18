#pragma once
//
// gmk::brep::Body -- the owning container for one geometric model. Every
// entity (vertex, edge, coedge, loop, face, shell) is stored in a typed
// pool inside the body. Geometric carriers (curve, surface) are stored in
// parallel pools as unique_ptr to abstract Curve/Surface bases.
//
// Bodies are move-only. They never share entities; copying a body deep-
// clones its contents.
//
// All edits go through the Builder/Editor helpers (brep/builder.hpp) which
// validate inputs before mutating storage. Direct field manipulation is
// supported for performance-critical inner loops.
//

#include <memory>
#include <vector>

#include "gmk/brep/ids.hpp"
#include "gmk/brep/topology.hpp"
#include "gmk/geom/curve.hpp"
#include "gmk/geom/surface.hpp"
#include "gmk/math/aabb.hpp"
#include "gmk/pool.hpp"
#include "gmk/result.hpp"

namespace gmk::brep {

class Body {
public:
    Body() = default;
    // Bodies are move-only. Deep cloning is provided by ``clone()`` below
    // which walks every entity and creates a new body with fresh handles.
    Body(const Body&)            = delete;
    Body& operator=(const Body&) = delete;
    Body(Body&&) noexcept            = default;
    Body& operator=(Body&&) noexcept = default;
    ~Body() = default;

    // Deep-copy this body into a new one. Used by ops that take a body
    // by value (e.g. transform_copy). Returns an empty body if anything
    // goes wrong.
    Body clone() const;

    BodyKind kind() const noexcept { return kind_; }
    void     set_kind(BodyKind k)  noexcept { kind_ = k; }

    // Geometry carriers. The body takes ownership.
    CurveId   add_curve(std::unique_ptr<Curve>&& c);
    SurfaceId add_surface(std::unique_ptr<Surface>&& s);
    Curve*    curve(CurveId h)               { return curve_get_(h); }
    const Curve* curve(CurveId h) const      { return curve_get_(h); }
    Surface*  surface(SurfaceId h)           { return surface_get_(h); }
    const Surface* surface(SurfaceId h) const{ return surface_get_(h); }

    // Topology accessors. nullptr if the handle is stale or invalid.
    Vertex*  vertex(VertexId h)               noexcept { return vertices_.get(h); }
    const Vertex*  vertex(VertexId h)   const noexcept { return vertices_.get(h); }
    Edge*    edge(EdgeId h)                   noexcept { return edges_.get(h); }
    const Edge*    edge(EdgeId h)       const noexcept { return edges_.get(h); }
    Coedge*  coedge(CoedgeId h)               noexcept { return coedges_.get(h); }
    const Coedge*  coedge(CoedgeId h)   const noexcept { return coedges_.get(h); }
    Loop*    loop(LoopId h)                   noexcept { return loops_.get(h); }
    const Loop*    loop(LoopId h)       const noexcept { return loops_.get(h); }
    Face*    face(FaceId h)                   noexcept { return faces_.get(h); }
    const Face*    face(FaceId h)       const noexcept { return faces_.get(h); }
    Shell*   shell(ShellId h)                 noexcept { return shells_.get(h); }
    const Shell*   shell(ShellId h)     const noexcept { return shells_.get(h); }

    // Allocation helpers.
    VertexId new_vertex(Vec3i p);
    EdgeId   new_edge(CurveId curve, VertexId vs, VertexId ve, double t0, double t1);
    CoedgeId new_coedge(EdgeId e, LoopId l, CoedgeSense sense);
    LoopId   new_loop(FaceId f, LoopKind k);
    FaceId   new_face(SurfaceId s, FaceSense sense);
    ShellId  new_shell(ShellKind k = ShellKind::Closed);

    // Stitching helpers.
    Status   link_loop(const CoedgeId* coedges, std::size_t n);
    void     mate_coedges(CoedgeId a, CoedgeId b) noexcept;
    void     add_face_to_shell(ShellId s, FaceId f);
    void     add_outer_loop_to_face(FaceId f, LoopId l);
    void     add_inner_loop_to_face(FaceId f, LoopId l);

    // Counts.
    std::size_t vertex_count() const noexcept { return vertices_.size(); }
    std::size_t edge_count()   const noexcept { return edges_.size(); }
    std::size_t coedge_count() const noexcept { return coedges_.size(); }
    std::size_t loop_count()   const noexcept { return loops_.size(); }
    std::size_t face_count()   const noexcept { return faces_.size(); }
    std::size_t shell_count()  const noexcept { return shells_.size(); }

    // Shell list (top-level entities).
    const std::vector<ShellId>& shells() const noexcept { return shell_list_; }
    void register_shell(ShellId s) { shell_list_.push_back(s); }

    // Bounding box of the body. The kernel keeps it lazily; ``bbox_dirty``
    // is set whenever an edit touches positions and ``bbox()`` re-walks
    // vertex positions on demand.
    AABB bbox() const;
    void mark_bbox_dirty() noexcept { bbox_dirty_ = true; }

    // Iteration.
    template <typename Fn> void for_each_face(Fn&& fn)   { faces_.for_each(std::forward<Fn>(fn)); }
    template <typename Fn> void for_each_face(Fn&& fn) const { faces_.for_each(std::forward<Fn>(fn)); }
    template <typename Fn> void for_each_edge(Fn&& fn)   { edges_.for_each(std::forward<Fn>(fn)); }
    template <typename Fn> void for_each_edge(Fn&& fn) const { edges_.for_each(std::forward<Fn>(fn)); }

private:
    Curve*   curve_get_(CurveId h);
    const Curve*  curve_get_(CurveId h) const;
    Surface* surface_get_(SurfaceId h);
    const Surface* surface_get_(SurfaceId h) const;

    // Geometry pools (we use raw pointers in the pool's slot type to avoid
    // copy-construction issues with unique_ptr inside std::vector).
    Pool<std::unique_ptr<Curve>,   CurveTag>   curves_;
    Pool<std::unique_ptr<Surface>, SurfaceTag> surfaces_;

    // Topology pools.
    Pool<Vertex, VertexTag>   vertices_;
    Pool<Edge,   EdgeTag>     edges_;
    Pool<Coedge, CoedgeTag>   coedges_;
    Pool<Loop,   LoopTag>     loops_;
    Pool<Face,   FaceTag>     faces_;
    Pool<Shell,  ShellTag>    shells_;

    std::vector<ShellId> shell_list_;

    BodyKind kind_{BodyKind::Solid};
    mutable AABB cached_bbox_{};
    mutable bool bbox_dirty_{true};
};

}  // namespace gmk::brep
