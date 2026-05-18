#pragma once
//
// gmk::lang::eval_program -- compile an OpenNuts source string into B-rep
// bodies.
//
// The evaluator walks the parsed AST, honours the active ``unit`` directive
// for length conversions, and builds one brep::Body per ``body NAME { ... }``
// declaration in the source. The bodies are handed off to a BodyStore which
// the caller controls -- the geometry server attaches its session map; tests
// attach a simple vector.
//
// The evaluator collects two kinds of diagnostic:
//
//   - syntactic diagnostics (forwarded from gmk::lang::Parser);
//   - semantic diagnostics (unknown argument names, bad arity, out-of-range
//     values, primitive construction failures).
//

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "gmk/brep/body.hpp"
#include "gmk/lang/ast.hpp"
#include "gmk/lang/parser.hpp"
#include "gmk/result.hpp"

namespace gmk::lang {

struct EvalBody {
    std::string  name;
    std::int64_t id;        // assigned by the BodyStore
};

class BodyStore {
public:
    virtual ~BodyStore() = default;
    // Take ownership of ``body`` and assign it a stable id. The id must
    // never be 0; 0 is reserved for "absent".
    virtual std::int64_t add(brep::Body body) = 0;
};

// Parse + build. Always returns Status::Ok; per-error reporting goes
// through ``out_diags``.
Status eval_program(std::string_view          source,
                    BodyStore&                store,
                    std::vector<EvalBody>&    out_bodies,
                    std::vector<Diagnostic>&  out_diags);

}  // namespace gmk::lang
