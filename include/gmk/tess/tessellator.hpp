#pragma once
//
// Surface tessellation. Produces a triangle mesh that approximates a
// brep::Body for display purposes. The mesh is the union of per-face
// tessellations; faces that share an edge are *not* stitched at the
// vertex-buffer level here -- that's a job for the display extension's
// indexing pass which knows where shared seams are.
//
// The tessellator samples each face's parametric domain on a grid whose
// resolution depends on:
//   - surface kind (analytic surfaces get a default density);
//   - chordal tolerance request (max deviation in metres between the
//     mesh and the underlying surface);
//   - a hard cap on triangles per face to prevent runaway tessellation.
//
// The output is double-precision and metric-unit; the caller converts to
// fixed-point or to float-32 for GPU upload at the boundary.
//

#include <cstdint>
#include <vector>

#include "gmk/brep/body.hpp"
#include "gmk/math/vec.hpp"
#include "gmk/result.hpp"

namespace gmk::tess {

struct TessOptions {
    double chord_tolerance_m = 1e-4;     // max chord-surface deviation
    double angle_tolerance   = 0.262;    // ~15° max normal deviation
    int    min_segments      = 6;        // minimum samples along each periodic direction
    int    max_segments      = 256;      // hard cap per parametric direction
};

// A face mesh in surface-local order: positions[i] / normals[i] are paired,
// and ``triangles`` holds 3*N indices into them.
struct FaceMesh {
    brep::FaceId            face{};
    std::vector<Vec3d>      positions_m;
    std::vector<Vec3d>      normals;
    std::vector<std::uint32_t> triangles;
};

struct BodyMesh {
    std::vector<FaceMesh> faces;
    // Optional flat (de-duplicated) layout for direct GPU upload. Built
    // lazily by ``flatten()``.
    std::vector<Vec3d>      flat_positions_m;
    std::vector<Vec3d>      flat_normals;
    std::vector<std::uint32_t> flat_triangles;
    bool                       flat_built{false};
};

// Tessellate every face in the body. Returns Ok on success. On any face
// failing tessellation the function continues; the resulting mesh's
// failure_count tracks how many failed.
Status tessellate(const brep::Body& body, const TessOptions& opts, BodyMesh& out);

// Build a single combined buffer of (positions, normals, triangles) by
// concatenating per-face meshes. No vertex welding is performed.
Status flatten(BodyMesh& mesh);

}  // namespace gmk::tess
