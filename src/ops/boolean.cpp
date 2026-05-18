#include "gmk/ops/boolean.hpp"

namespace gmk::ops {

// All three operations share the same implementation skeleton: they would
// take both bodies, find surface-surface intersection curves, split the
// faces along those curves, classify the resulting pieces according to
// the operation type, and re-stitch into the output body.
//
// Until that pipeline lands, every boolean is reported as not implemented.
// Callers can still depend on the API.
Status union_bodies(const brep::Body&, const brep::Body&, brep::Body&) {
    return Status::NotImplemented;
}
Status intersect_bodies(const brep::Body&, const brep::Body&, brep::Body&) {
    return Status::NotImplemented;
}
Status subtract_bodies(const brep::Body&, const brep::Body&, brep::Body&) {
    return Status::NotImplemented;
}

}  // namespace gmk::ops
