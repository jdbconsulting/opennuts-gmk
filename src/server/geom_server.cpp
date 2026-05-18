#include "gmk/server/geom_server.hpp"

#include <cmath>
#include <istream>
#include <ostream>
#include <utility>

#include "gmk/brep/primitives.hpp"
#include "gmk/brep/validate.hpp"
#include "gmk/lang/eval.hpp"
#include "gmk/lang/parser.hpp"
#include "gmk/tess/tessellator.hpp"
#include "gmk/units.hpp"
#include "gmk/version.hpp"

namespace gmk::geom_server {

Server::Server(std::istream& in, std::ostream& out, jsonrpc::Framing f)
    : in_{in}, out_{out}, framing_{f} {}

std::int64_t Server::add(brep::Body body) {
    BodyId id = next_id_++;
    bodies_.emplace(id, std::move(body));
    return static_cast<std::int64_t>(id);
}

length_t Server::length_from_units(double v, const std::string& unit, bool& ok) {
    ok = true;
    if (unit.empty() || unit == "mm") return mm_to_length(v);
    if (unit == "m"   || unit == "meter")  return m_to_length(v);
    if (unit == "cm")                       return mm_to_length(v * 10.0);
    if (unit == "um"  || unit == "micron")  return um_to_length(v);
    if (unit == "in"  || unit == "inch")    return inch_to_length(v);
    if (unit == "ft"  || unit == "foot")    return inch_to_length(v * 12.0);
    ok = false;
    return 0;
}

bool Server::vec3_from_array(const json::Value& v, length_t out[3],
                             const std::string& unit) {
    if (!v.is_array() || v.as_array().size() != 3) return false;
    bool ok = true;
    for (int i = 0; i < 3; ++i) {
        double d = v.as_array()[static_cast<std::size_t>(i)].as_double(0.0);
        bool   ok_one = true;
        out[i] = length_from_units(d, unit, ok_one);
        if (!ok_one) ok = false;
    }
    return ok;
}

bool Server::vec3d_from_array(const json::Value& v, double out[3]) {
    if (!v.is_array() || v.as_array().size() != 3) return false;
    for (int i = 0; i < 3; ++i)
        out[i] = v.as_array()[static_cast<std::size_t>(i)].as_double(0.0);
    return true;
}

namespace {

const char* diag_severity_name(gmk::lang::Diagnostic::Severity s) {
    switch (s) {
        case gmk::lang::Diagnostic::Severity::Hint:    return "hint";
        case gmk::lang::Diagnostic::Severity::Info:    return "info";
        case gmk::lang::Diagnostic::Severity::Warning: return "warning";
        case gmk::lang::Diagnostic::Severity::Error:   return "error";
    }
    return "error";
}

json::Value diag_to_json(const gmk::lang::Diagnostic& d) {
    json::Object o;
    o.emplace_back("severity", json::Value{diag_severity_name(d.severity)});
    o.emplace_back("line",     json::Value{static_cast<std::int64_t>(d.pos.line)});
    o.emplace_back("column",   json::Value{static_cast<std::int64_t>(d.pos.column)});
    o.emplace_back("offset",   json::Value{static_cast<std::int64_t>(d.pos.offset)});
    o.emplace_back("length",   json::Value{static_cast<std::int64_t>(d.length)});
    o.emplace_back("message",  json::Value{d.message});
    if (!d.code.empty()) o.emplace_back("code", json::Value{d.code});
    return json::Value{std::move(o)};
}

json::Value bbox_to_json(const AABB& b, double scale) {
    json::Array mn, mx;
    mn.push_back(json::Value{length_to_m(b.min.x) * scale});
    mn.push_back(json::Value{length_to_m(b.min.y) * scale});
    mn.push_back(json::Value{length_to_m(b.min.z) * scale});
    mx.push_back(json::Value{length_to_m(b.max.x) * scale});
    mx.push_back(json::Value{length_to_m(b.max.y) * scale});
    mx.push_back(json::Value{length_to_m(b.max.z) * scale});
    json::Object o;
    o.emplace_back("min", json::Value{std::move(mn)});
    o.emplace_back("max", json::Value{std::move(mx)});
    return json::Value{std::move(o)};
}

double unit_scale_to_m(const std::string& unit) {
    // multiplier so that ``length_to_m(v) * scale`` gives the value in the
    // requested unit.
    if (unit.empty() || unit == "m" || unit == "meter") return 1.0;
    if (unit == "mm") return 1e3;
    if (unit == "cm") return 1e2;
    if (unit == "um" || unit == "micron") return 1e6;
    if (unit == "in" || unit == "inch") return 1.0 / 0.0254;
    if (unit == "ft" || unit == "foot") return 1.0 / 0.3048;
    return 1.0;
}

}  // namespace

json::Value Server::handle(const std::string& m, const json::Value* params,
                           int& err, std::string& err_msg) {
    auto get_str = [&](const char* key, std::string def = "") -> std::string {
        if (!params) return def;
        const json::Value* v = params->find(key);
        if (!v || !v->is_string()) return def;
        return std::string{v->as_string()};
    };
    auto get_num = [&](const char* key, double def = 0.0) -> double {
        if (!params) return def;
        const json::Value* v = params->find(key);
        if (!v || !v->is_number()) return def;
        return v->as_double();
    };
    auto get_id = [&](const char* key, BodyId def = 0) -> BodyId {
        if (!params) return def;
        const json::Value* v = params->find(key);
        if (!v || !v->is_number()) return def;
        return static_cast<BodyId>(v->as_int());
    };

    if (m == "gmk.version") {
        json::Object o;
        o.emplace_back("version", json::Value{VERSION_STRING});
        return json::Value{std::move(o)};
    }

    if (m == "gmk.session.info") {
        json::Object o;
        std::size_t total_faces = 0;
        for (auto& [_, b] : bodies_) total_faces += b.face_count();
        o.emplace_back("bodies",     json::Value{static_cast<std::int64_t>(bodies_.size())});
        o.emplace_back("mesh_faces", json::Value{static_cast<std::int64_t>(total_faces)});
        return json::Value{std::move(o)};
    }

    if (m == "gmk.body.delete") {
        BodyId id = get_id("id");
        auto it = bodies_.find(id);
        if (it == bodies_.end()) {
            err = jsonrpc::kInvalidParams; err_msg = "unknown body id";
            return json::Value{};
        }
        bodies_.erase(it);
        json::Object o;
        o.emplace_back("ok", json::Value{true});
        return json::Value{std::move(o)};
    }

    if (m == "gmk.session.clear") {
        bodies_.clear();
        next_id_ = 1;
        json::Object o;
        o.emplace_back("ok", json::Value{true});
        return json::Value{std::move(o)};
    }

    if (m == "gmk.lang.parse") {
        std::string source = get_str("source");
        gmk::lang::Parser parser{source};
        parser.parse();
        json::Array diags;
        for (const auto& d : parser.diagnostics()) diags.push_back(diag_to_json(d));
        json::Object o;
        o.emplace_back("diagnostics", json::Value{std::move(diags)});
        return json::Value{std::move(o)};
    }

    if (m == "gmk.lang.eval") {
        std::string source = get_str("source");
        bool clear = false;
        if (params) {
            const json::Value* c = params->find("clear");
            if (c && c->is_bool()) clear = c->as_bool();
        }
        if (clear) {
            bodies_.clear();
            next_id_ = 1;
        }
        std::vector<gmk::lang::EvalBody>   evbodies;
        std::vector<gmk::lang::Diagnostic> evdiags;
        gmk::lang::eval_program(source, *this, evbodies, evdiags);
        json::Array bs;
        for (const auto& b : evbodies) {
            json::Object o;
            o.emplace_back("name", json::Value{b.name});
            o.emplace_back("id",   json::Value{b.id});
            bs.push_back(json::Value{std::move(o)});
        }
        json::Array diags;
        for (const auto& d : evdiags) diags.push_back(diag_to_json(d));
        json::Object o;
        o.emplace_back("bodies",      json::Value{std::move(bs)});
        o.emplace_back("diagnostics", json::Value{std::move(diags)});
        return json::Value{std::move(o)};
    }

    auto unit = get_str("unit", "mm");

    if (m == "gmk.primitive.box") {
        if (!params || !params->find("center")) {
            err = jsonrpc::kInvalidParams; err_msg = "missing 'center'";
            return json::Value{};
        }
        length_t c[3];
        if (!vec3_from_array(*params->find("center"), c, unit)) {
            err = jsonrpc::kInvalidParams; err_msg = "center must be [x,y,z]";
            return json::Value{};
        }
        bool ok;
        length_t hx = length_from_units(get_num("hx", 0), unit, ok);
        length_t hy = length_from_units(get_num("hy", 0), unit, ok);
        length_t hz = length_from_units(get_num("hz", 0), unit, ok);
        brep::Body body;
        Status s = brep::make_box(body, Vec3i{c[0], c[1], c[2]}, hx, hy, hz);
        if (s != Status::Ok) {
            err = jsonrpc::kInvalidParams; err_msg = std::string{status_name(s)};
            return json::Value{};
        }
        BodyId id = next_id_++;
        bodies_.emplace(id, std::move(body));
        json::Object o;
        o.emplace_back("id", json::Value{static_cast<std::int64_t>(id)});
        return json::Value{std::move(o)};
    }
    if (m == "gmk.primitive.sphere") {
        length_t c[3] = {0,0,0};
        if (params && params->find("center")) {
            if (!vec3_from_array(*params->find("center"), c, unit)) {
                err = jsonrpc::kInvalidParams; err_msg = "center must be [x,y,z]";
                return json::Value{};
            }
        }
        bool ok;
        length_t r = length_from_units(get_num("r", 0), unit, ok);
        brep::Body body;
        Status s = brep::make_sphere(body, Vec3i{c[0], c[1], c[2]}, r);
        if (s != Status::Ok) {
            err = jsonrpc::kInvalidParams; err_msg = std::string{status_name(s)};
            return json::Value{};
        }
        BodyId id = next_id_++;
        bodies_.emplace(id, std::move(body));
        json::Object o;
        o.emplace_back("id", json::Value{static_cast<std::int64_t>(id)});
        return json::Value{std::move(o)};
    }
    if (m == "gmk.primitive.cylinder") {
        length_t base[3] = {0,0,0};
        double   ax[3]   = {0,0,1};
        if (params && params->find("base")) {
            if (!vec3_from_array(*params->find("base"), base, unit)) {
                err = jsonrpc::kInvalidParams; err_msg = "base must be [x,y,z]";
                return json::Value{};
            }
        }
        if (params && params->find("axis")) {
            vec3d_from_array(*params->find("axis"), ax);
        }
        bool ok;
        length_t r = length_from_units(get_num("r", 0), unit, ok);
        length_t h = length_from_units(get_num("h", 0), unit, ok);
        brep::Body body;
        Status s = brep::make_cylinder(body, Vec3i{base[0], base[1], base[2]},
                                       Vec3d{ax[0], ax[1], ax[2]}, r, h);
        if (s != Status::Ok) {
            err = jsonrpc::kInvalidParams; err_msg = std::string{status_name(s)};
            return json::Value{};
        }
        BodyId id = next_id_++;
        bodies_.emplace(id, std::move(body));
        json::Object o;
        o.emplace_back("id", json::Value{static_cast<std::int64_t>(id)});
        return json::Value{std::move(o)};
    }
    if (m == "gmk.primitive.cone") {
        length_t base[3] = {0,0,0};
        double   ax[3]   = {0,0,1};
        if (params && params->find("base")) vec3_from_array(*params->find("base"), base, unit);
        if (params && params->find("axis")) vec3d_from_array(*params->find("axis"), ax);
        bool ok;
        length_t r_base = length_from_units(get_num("r_base", 0), unit, ok);
        length_t r_top  = length_from_units(get_num("r_top",  0), unit, ok);
        length_t h      = length_from_units(get_num("h",      0), unit, ok);
        brep::Body body;
        Status s = brep::make_cone(body, Vec3i{base[0], base[1], base[2]},
                                   Vec3d{ax[0], ax[1], ax[2]}, r_base, r_top, h);
        if (s != Status::Ok) {
            err = jsonrpc::kInvalidParams; err_msg = std::string{status_name(s)};
            return json::Value{};
        }
        BodyId id = next_id_++;
        bodies_.emplace(id, std::move(body));
        json::Object o;
        o.emplace_back("id", json::Value{static_cast<std::int64_t>(id)});
        return json::Value{std::move(o)};
    }
    if (m == "gmk.primitive.torus") {
        length_t c[3] = {0,0,0};
        double   ax[3] = {0,0,1};
        if (params && params->find("center")) vec3_from_array(*params->find("center"), c, unit);
        if (params && params->find("axis"))   vec3d_from_array(*params->find("axis"),  ax);
        bool ok;
        length_t R = length_from_units(get_num("R", 0), unit, ok);
        length_t r = length_from_units(get_num("r", 0), unit, ok);
        brep::Body body;
        Status s = brep::make_torus(body, Vec3i{c[0], c[1], c[2]},
                                    Vec3d{ax[0], ax[1], ax[2]}, R, r);
        if (s != Status::Ok) {
            err = jsonrpc::kInvalidParams; err_msg = std::string{status_name(s)};
            return json::Value{};
        }
        BodyId id = next_id_++;
        bodies_.emplace(id, std::move(body));
        json::Object o;
        o.emplace_back("id", json::Value{static_cast<std::int64_t>(id)});
        return json::Value{std::move(o)};
    }
    if (m == "gmk.body.bbox") {
        BodyId id = get_id("id");
        auto it = bodies_.find(id);
        if (it == bodies_.end()) {
            err = jsonrpc::kInvalidParams; err_msg = "unknown body id";
            return json::Value{};
        }
        return bbox_to_json(it->second.bbox(), unit_scale_to_m(unit));
    }
    if (m == "gmk.body.validate") {
        BodyId id = get_id("id");
        auto it = bodies_.find(id);
        if (it == bodies_.end()) {
            err = jsonrpc::kInvalidParams; err_msg = "unknown body id";
            return json::Value{};
        }
        brep::ValidationReport rep;
        brep::validate(it->second, rep);
        json::Array issues;
        for (const auto& i : rep.issues) {
            json::Object o;
            const char* sev = "info";
            switch (i.severity) {
                case brep::ValidationIssue::Severity::Error:   sev = "error";   break;
                case brep::ValidationIssue::Severity::Warning: sev = "warning"; break;
                case brep::ValidationIssue::Severity::Info:    sev = "info";    break;
            }
            o.emplace_back("severity", json::Value{sev});
            o.emplace_back("message",  json::Value{i.message});
            o.emplace_back("slot",     json::Value{static_cast<std::int64_t>(i.slot)});
            issues.push_back(json::Value{std::move(o)});
        }
        json::Object o;
        o.emplace_back("ok",          json::Value{rep.ok()});
        o.emplace_back("errors",      json::Value{static_cast<std::int64_t>(rep.error_count)});
        o.emplace_back("warnings",    json::Value{static_cast<std::int64_t>(rep.warning_count)});
        o.emplace_back("issues",      json::Value{std::move(issues)});
        return json::Value{std::move(o)};
    }
    if (m == "gmk.tessellate") {
        BodyId id = get_id("id");
        auto it = bodies_.find(id);
        if (it == bodies_.end()) {
            err = jsonrpc::kInvalidParams; err_msg = "unknown body id";
            return json::Value{};
        }
        tess::TessOptions opts;
        opts.chord_tolerance_m = get_num("chord_tol_m", opts.chord_tolerance_m);
        tess::BodyMesh mesh;
        Status s = tess::tessellate(it->second, opts, mesh);
        if (s != Status::Ok) {
            err = jsonrpc::kInternalError; err_msg = std::string{status_name(s)};
            return json::Value{};
        }
        tess::flatten(mesh);
        double scale = unit_scale_to_m(unit);

        json::Array positions, normals, triangles;
        positions.reserve(mesh.flat_positions_m.size() * 3);
        normals  .reserve(mesh.flat_normals.size()     * 3);
        triangles.reserve(mesh.flat_triangles.size());
        for (const auto& p : mesh.flat_positions_m) {
            positions.push_back(json::Value{p.x * scale});
            positions.push_back(json::Value{p.y * scale});
            positions.push_back(json::Value{p.z * scale});
        }
        for (const auto& n : mesh.flat_normals) {
            normals.push_back(json::Value{n.x});
            normals.push_back(json::Value{n.y});
            normals.push_back(json::Value{n.z});
        }
        for (auto t : mesh.flat_triangles) {
            triangles.push_back(json::Value{static_cast<std::int64_t>(t)});
        }
        json::Object o;
        o.emplace_back("positions", json::Value{std::move(positions)});
        o.emplace_back("normals",   json::Value{std::move(normals)});
        o.emplace_back("triangles", json::Value{std::move(triangles)});
        o.emplace_back("vertex_count", json::Value{static_cast<std::int64_t>(mesh.flat_positions_m.size())});
        return json::Value{std::move(o)};
    }

    err = jsonrpc::kMethodNotFound;
    err_msg = "method not found: " + m;
    return json::Value{};
}

json::Value Server::dispatch_request(const json::Value& req) {
    const json::Value* method = req.find("method");
    const json::Value* idv    = req.find("id");
    const json::Value* params = req.find("params");
    json::Value id = idv ? *idv : json::Value{};
    if (!method || !method->is_string()) {
        return jsonrpc::make_error(id, jsonrpc::kInvalidRequest, "missing method");
    }
    int err = 0; std::string err_msg;
    json::Value res = handle(std::string{method->as_string()}, params, err, err_msg);
    if (err != 0) return jsonrpc::make_error(id, err, std::move(err_msg));
    return jsonrpc::make_result(id, std::move(res));
}

Status Server::run() {
    for (;;) {
        auto msg = jsonrpc::read_message(in_, framing_);
        if (!msg) {
            if (msg.status() == Status::Io) return Status::Ok;
            return msg.status();
        }
        json::Value reply = dispatch_request(msg.value());
        jsonrpc::write_message(out_, reply, framing_);
    }
}

}  // namespace gmk::geom_server
