#pragma once
//
// Helpers to populate a brep::Body with the canonical CAD primitives.
//
// Each helper appends a fully-formed solid body to the given Body. The
// Body must be empty on entry (these are constructors, not editors).
//
// All inputs are in kernel integer units (length_t fm). The functions
// validate inputs and return Status; on failure the body is left in its
// original empty state.
//

#include "gmk/brep/body.hpp"
#include "gmk/math/vec.hpp"
#include "gmk/result.hpp"
#include "gmk/units.hpp"

namespace gmk::brep {

// Axis-aligned box centred at origin with half-extents (hx, hy, hz).
Status make_box(Body& body, Vec3i centre, length_t hx, length_t hy, length_t hz);

// Sphere of radius r centred at origin. Built as one face on a sphere
// surface with two singular poles. The body has one shell, one face,
// one outer loop, but no interior edges (the parametric domain is
// already closed in u and clamped at the poles in v).
Status make_sphere(Body& body, Vec3i centre, length_t r);

// Right circular cylinder. The axis goes from ``base`` upward by
// ``height``; the body has 3 faces (lateral + 2 caps) and the
// appropriate edges and vertices.
Status make_cylinder(Body& body, Vec3i base, Vec3d axis, length_t r, length_t height);

// Right circular cone. ``r_base`` is the radius at the base, ``r_top``
// may be 0 for an apex; otherwise it's a frustum (truncated cone).
Status make_cone(Body& body, Vec3i base, Vec3d axis,
                 length_t r_base, length_t r_top, length_t height);

// Torus with major radius R (axis -> tube centre) and minor radius r.
Status make_torus(Body& body, Vec3i centre, Vec3d axis, length_t R, length_t r);

}  // namespace gmk::brep
