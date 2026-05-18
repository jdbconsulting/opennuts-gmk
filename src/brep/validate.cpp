#include "gmk/brep/validate.hpp"

#include <unordered_set>

namespace gmk::brep {

namespace {

void add(ValidationReport& r, ValidationIssue::Severity s, std::string msg, std::uint32_t slot = 0) {
    r.issues.push_back({s, std::move(msg), slot});
    if (s == ValidationIssue::Severity::Error)        r.error_count++;
    else if (s == ValidationIssue::Severity::Warning) r.warning_count++;
}

}  // namespace

Status validate(const Body& body, ValidationReport& report) {
    report = ValidationReport{};

    // --- Edges ---
    body.for_each_edge([&](EdgeId h, const Edge& e) {
        if (!e.curve.valid())
            add(report, ValidationIssue::Severity::Error,
                "edge has null curve", h.index);
        else if (!body.curve(e.curve))
            add(report, ValidationIssue::Severity::Error,
                "edge references stale curve", h.index);

        if (!e.start.valid() || !body.vertex(e.start))
            add(report, ValidationIssue::Severity::Error,
                "edge has invalid start vertex", h.index);
        if (!e.end.valid() || !body.vertex(e.end))
            add(report, ValidationIssue::Severity::Error,
                "edge has invalid end vertex", h.index);

        if (e.t_max <= e.t_min)
            add(report, ValidationIssue::Severity::Error,
                "edge parameter interval is empty or inverted", h.index);
    });

    // --- Faces / loops / coedges ---
    body.for_each_face([&](FaceId fh, const Face& f) {
        if (!f.surface.valid() || !body.surface(f.surface)) {
            add(report, ValidationIssue::Severity::Error,
                "face has invalid surface", fh.index);
        }
        if (!f.outer.valid() || !body.loop(f.outer)) {
            add(report, ValidationIssue::Severity::Error,
                "face has invalid outer loop", fh.index);
            return;
        }
        // Walk the outer loop.
        std::unordered_set<std::uint32_t> seen;
        const Loop* L = body.loop(f.outer);
        CoedgeId start = L->first;
        CoedgeId cur   = start;
        std::size_t steps = 0;
        if (!start.valid()) {
            // Empty loop is OK (sphere/torus full-domain face).
            return;
        }
        for (;;) {
            ++steps;
            if (steps > body.coedge_count() + 4) {
                add(report, ValidationIssue::Severity::Error,
                    "loop ring failed to close", fh.index);
                return;
            }
            const Coedge* c = body.coedge(cur);
            if (!c) {
                add(report, ValidationIssue::Severity::Error,
                    "loop contains invalid coedge", fh.index);
                return;
            }
            if (!seen.insert(cur.index).second) {
                add(report, ValidationIssue::Severity::Error,
                    "loop visits coedge twice", fh.index);
                return;
            }
            if (!c->edge.valid() || !body.edge(c->edge)) {
                add(report, ValidationIssue::Severity::Error,
                    "coedge references invalid edge", fh.index);
                return;
            }
            cur = c->next;
            if (cur == start) break;
        }
    });

    // --- Shells ---
    for (ShellId sh : body.shells()) {
        const Shell* S = body.shell(sh);
        if (!S) {
            add(report, ValidationIssue::Severity::Error,
                "body lists invalid shell", sh.index);
            continue;
        }
        for (FaceId fh : S->faces) {
            if (!body.face(fh)) {
                add(report, ValidationIssue::Severity::Error,
                    "shell lists stale face", sh.index);
            }
        }
    }
    return Status::Ok;
}

}  // namespace gmk::brep
