#pragma once
//
// Boolean operations between bodies.
//
// NOTE: The implementation in src/ops/boolean.cpp currently returns
// Status::NotImplemented. Real NURBS-on-NURBS boolean operations require
// surface-surface intersection (SSI), curve-on-surface tracking, region
// classification and edge stitching -- on the order of multiple thousand
// lines on top of this kernel. The API and contract are stable and ready
// to receive that implementation in a follow-up.
//

#include "gmk/brep/body.hpp"
#include "gmk/result.hpp"

namespace gmk::ops {

Status union_bodies(const brep::Body& a, const brep::Body& b, brep::Body& out);
Status intersect_bodies(const brep::Body& a, const brep::Body& b, brep::Body& out);
Status subtract_bodies(const brep::Body& a, const brep::Body& b, brep::Body& out);

}  // namespace gmk::ops
