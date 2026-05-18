#pragma once
//
// B-rep topology entities. The data layout follows the classical coedge
// (half-edge) model:
//
//   Body  ──> Shell* ──> Face* ──> Loop* ──> Coedge*
//                                    │
//                                    └──> Edge ──> Vertex × 2
//                                            └──> Curve
//   Face ──> Surface
//   Coedge has a "partner" coedge on the *other* side of the same edge,
//   so the local neighbourhood of an edge is reachable in O(1).
//
// Every numerical field that names a 3D position is stored in kernel
// integer units (length_t fm). Every parametric value is stored as a
// double.
//

#include <cstdint>
#include <vector>

#include "gmk/brep/ids.hpp"
#include "gmk/math/aabb.hpp"
#include "gmk/math/vec.hpp"

namespace gmk::brep {

enum class CoedgeSense : std::uint8_t {
    SameAsCurve = 0,
    Reversed    = 1,
};

enum class LoopKind : std::uint8_t {
    Outer = 0,   // outer boundary, ccw with respect to surface normal
    Inner = 1,   // inner boundary, cw with respect to surface normal
};

enum class FaceSense : std::uint8_t {
    SameAsSurface = 0,
    Reversed      = 1,
};

enum class ShellKind : std::uint8_t {
    Closed = 0,  // solid boundary
    Open   = 1,  // sheet body
};

enum class BodyKind : std::uint8_t {
    Solid  = 0,
    Sheet  = 1,
    Wire   = 2,
    Point  = 3,
};

struct Vertex {
    Vec3i    position{0,0,0};
    CoedgeId outgoing{};  // one of the coedges starting at this vertex
};

struct Edge {
    CurveId  curve{};
    VertexId start{};
    VertexId end{};
    // The two coedges sharing this edge; ``coedge_b`` is null for a wire edge.
    CoedgeId coedge_a{};
    CoedgeId coedge_b{};
    // Parameter interval on the underlying curve that this edge occupies.
    double   t_min{0.0};
    double   t_max{1.0};
};

struct Coedge {
    EdgeId      edge{};
    LoopId      loop{};
    CoedgeSense sense{CoedgeSense::SameAsCurve};
    // Loop ring: previous and next coedge in the same loop.
    CoedgeId    prev{};
    CoedgeId    next{};
    // The other half-edge across the underlying Edge (null if dangling).
    CoedgeId    partner{};
};

struct Loop {
    FaceId    face{};
    CoedgeId  first{};
    LoopKind  kind{LoopKind::Outer};
};

struct Face {
    SurfaceId surface{};
    LoopId    outer{};
    std::vector<LoopId> inner;
    FaceSense sense{FaceSense::SameAsSurface};
    ShellId   shell{};
};

struct Shell {
    std::vector<FaceId> faces;
    ShellKind kind{ShellKind::Closed};
};

}  // namespace gmk::brep
