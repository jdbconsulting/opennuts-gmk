#pragma once
//
// B-rep entity IDs. Each one is a Handle<Tag> with index + generation,
// providing constant-time validation and resistance to ABA bugs.
//

#include "gmk/pool.hpp"

namespace gmk::brep {

struct VertexTag  {};
struct EdgeTag    {};
struct CoedgeTag  {};
struct LoopTag    {};
struct FaceTag    {};
struct ShellTag   {};
struct CurveTag   {};
struct SurfaceTag {};

using VertexId  = Handle<VertexTag>;
using EdgeId    = Handle<EdgeTag>;
using CoedgeId  = Handle<CoedgeTag>;
using LoopId    = Handle<LoopTag>;
using FaceId    = Handle<FaceTag>;
using ShellId   = Handle<ShellTag>;
using CurveId   = Handle<CurveTag>;
using SurfaceId = Handle<SurfaceTag>;

}  // namespace gmk::brep
