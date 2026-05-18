#pragma once
//
// Body integrity checks. The kernel runs ``validate(body)`` at construction
// time during tests and offers it on demand to consumers who want to do
// fault detection after a long modelling session. The check is structural
// only -- it does not verify geometric containment or that loops are
// non-self-intersecting; those are far more expensive checks that live in
// a higher-tier toolkit.
//

#include <string>
#include <vector>

#include "gmk/brep/body.hpp"
#include "gmk/result.hpp"

namespace gmk::brep {

struct ValidationIssue {
    enum class Severity { Info, Warning, Error };
    Severity    severity;
    std::string message;
    // Optional identifying info -- not all issues have a clear owner.
    std::uint32_t slot{0};
};

struct ValidationReport {
    std::vector<ValidationIssue> issues;
    int error_count{0};
    int warning_count{0};
    bool ok() const noexcept { return error_count == 0; }
};

// Run all structural checks on ``body``. Returns Status::Ok always; the
// presence of errors is signalled via ``report.error_count`` and
// individual issue entries. Never throws, never allocates beyond the
// report's vector.
Status validate(const Body& body, ValidationReport& report);

}  // namespace gmk::brep
