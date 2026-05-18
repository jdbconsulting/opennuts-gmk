#include "gmk/lang/eval.hpp"

#include <cmath>
#include <string>
#include <unordered_map>
#include <variant>

#include "gmk/brep/primitives.hpp"
#include "gmk/units.hpp"

namespace gmk::lang {

namespace {

// Convert a (unit, scalar) pair to kernel femtometres.
length_t length_in_units(double v, ast::UnitKind u) {
    switch (u) {
        case ast::UnitKind::Mm:   return mm_to_length(v);
        case ast::UnitKind::Cm:   return mm_to_length(v * 10.0);
        case ast::UnitKind::M:    return m_to_length(v);
        case ast::UnitKind::Inch: return inch_to_length(v);
        case ast::UnitKind::Foot: return inch_to_length(v * 12.0);
    }
    return mm_to_length(v);
}

Vec3i vec3_in_units(const ast::Vec3Literal& v, ast::UnitKind u) {
    return Vec3i{
        length_in_units(v.x, u),
        length_in_units(v.y, u),
        length_in_units(v.z, u),
    };
}

void add_diag(std::vector<Diagnostic>& diags, ast::Span span,
              std::string msg, std::string code = "") {
    diags.push_back(Diagnostic{
        Diagnostic::Severity::Error,
        span.begin,
        1,
        std::move(msg),
        std::move(code),
    });
}

// Lookup an argument by name, falling back to positional index. Returns
// nullptr if absent. ``found_positional`` is set if a positional match
// was used (so callers can detect ambiguous use of named + positional).
const ast::Arg* find_arg(const std::vector<ast::Arg>& args,
                         std::string_view name,
                         std::size_t      pos) {
    for (const auto& a : args) if (a.name == name) return &a;
    if (pos < args.size() && args[pos].name.empty()) return &args[pos];
    return nullptr;
}

// Build a single primitive into the body. Returns true on success.
bool build_primitive(const ast::Primitive&     p,
                     ast::UnitKind             unit,
                     brep::Body&               body,
                     std::vector<Diagnostic>&  diags) {
    Vec3i centre{0, 0, 0};
    if (p.has_at)                centre = vec3_in_units(p.at_pos, unit);
    Vec3d axis{0, 0, 1};
    if (p.has_axis)              axis   = Vec3d{p.axis_vec.x, p.axis_vec.y, p.axis_vec.z};

    auto require = [&](const char* name, std::size_t pos, double& out) -> bool {
        const ast::Arg* a = find_arg(p.args, name, pos);
        if (!a) {
            add_diag(diags, p.span,
                     std::string("missing required argument '") + name + "'", "E001");
            return false;
        }
        out = a->value;
        return true;
    };

    switch (p.kind) {
        case ast::Primitive::Kind::Box: {
            double w = 0, d = 0, h = 0;
            if (!require("width", 0, w) || !require("depth", 1, d) || !require("height", 2, h))
                return false;
            // Convention: width along X, depth along Y, height along Z.
            // "at" specifies the centre. Half-extents are w/2, d/2, h/2.
            Status s = brep::make_box(body, centre,
                                       length_in_units(w * 0.5, unit),
                                       length_in_units(d * 0.5, unit),
                                       length_in_units(h * 0.5, unit));
            if (s != Status::Ok) {
                add_diag(diags, p.span, std::string("box: ") + status_name(s), "E002");
                return false;
            }
            return true;
        }
        case ast::Primitive::Kind::Sphere: {
            double r = 0;
            if (!require("radius", 0, r)) return false;
            Status s = brep::make_sphere(body, centre, length_in_units(r, unit));
            if (s != Status::Ok) {
                add_diag(diags, p.span, std::string("sphere: ") + status_name(s), "E002");
                return false;
            }
            return true;
        }
        case ast::Primitive::Kind::Cylinder: {
            double r = 0, h = 0;
            if (!require("radius", 0, r) || !require("height", 1, h)) return false;
            Status s = brep::make_cylinder(body, centre, axis,
                                            length_in_units(r, unit),
                                            length_in_units(h, unit));
            if (s != Status::Ok) {
                add_diag(diags, p.span, std::string("cylinder: ") + status_name(s), "E002");
                return false;
            }
            return true;
        }
        case ast::Primitive::Kind::Cone: {
            double rb = 0, rt = 0, h = 0;
            if (!require("radius_base", 0, rb) || !require("radius_top", 1, rt) ||
                !require("height", 2, h)) return false;
            Status s = brep::make_cone(body, centre, axis,
                                        length_in_units(rb, unit),
                                        length_in_units(rt, unit),
                                        length_in_units(h, unit));
            if (s != Status::Ok) {
                add_diag(diags, p.span, std::string("cone: ") + status_name(s), "E002");
                return false;
            }
            return true;
        }
        case ast::Primitive::Kind::Torus: {
            double R = 0, r = 0;
            if (!require("major_radius", 0, R) || !require("minor_radius", 1, r)) return false;
            Status s = brep::make_torus(body, centre, axis,
                                         length_in_units(R, unit),
                                         length_in_units(r, unit));
            if (s != Status::Ok) {
                add_diag(diags, p.span, std::string("torus: ") + status_name(s), "E002");
                return false;
            }
            return true;
        }
    }
    return false;
}

}  // namespace

Status eval_program(std::string_view source,
                    BodyStore& store,
                    std::vector<EvalBody>& out_bodies,
                    std::vector<Diagnostic>& out_diags) {
    out_bodies.clear();
    out_diags.clear();

    Parser parser{source};
    ast::Program prog = parser.parse();
    for (const auto& d : parser.diagnostics()) out_diags.push_back(d);

    ast::UnitKind active_unit = ast::UnitKind::Mm;

    for (const auto& item : prog.items) {
        if (auto* u = std::get_if<ast::UnitDirective>(&item)) {
            active_unit = u->unit;
            continue;
        }
        if (std::get_if<ast::ToleranceDirective>(&item)) {
            // Tolerance directives are advisory in this kernel slice.
            continue;
        }
        const ast::BodyDef* b = std::get_if<ast::BodyDef>(&item);
        if (!b) continue;
        if (b->name.empty()) continue;

        // Each body statement currently builds an independent primitive
        // and we ship one body per primitive (named NAME_<index>) so that
        // the viewer can render each separately. A future revision will
        // perform boolean composition inside the body declaration.
        std::size_t emitted = 0;
        for (std::size_t i = 0; i < b->statements.size(); ++i) {
            const auto& stmt = b->statements[i];
            if (auto* prim = std::get_if<ast::Primitive>(&stmt)) {
                brep::Body body;
                if (!build_primitive(*prim, active_unit, body, out_diags)) continue;
                std::string nm = b->name;
                if (b->statements.size() > 1) {
                    nm += '#';
                    nm += std::to_string(i);
                }
                std::int64_t id = store.add(std::move(body));
                if (id != 0) {
                    out_bodies.push_back(EvalBody{std::move(nm), id});
                    ++emitted;
                }
            } else if (std::get_if<ast::BooleanOp>(&stmt)) {
                add_diag(out_diags, b->span,
                         "boolean operations are not yet implemented", "E003");
            }
        }
        (void)emitted;
    }
    return Status::Ok;
}

}  // namespace gmk::lang
