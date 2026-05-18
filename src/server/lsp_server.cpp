#include "gmk/server/lsp_server.hpp"

#include <istream>
#include <ostream>
#include <string>
#include <utility>

#include "gmk/lang/parser.hpp"
#include "gmk/server/jsonrpc.hpp"
#include "gmk/version.hpp"

namespace gmk::lsp {

LspServer::LspServer(std::istream& in, std::ostream& out)
    : in_{in}, out_{out} {}

void LspServer::send_response(const json::Value& id, json::Value result) {
    jsonrpc::write_message(out_, jsonrpc::make_result(id, std::move(result)),
                            jsonrpc::Framing::LspHeaders);
}
void LspServer::send_error(const json::Value& id, int code, std::string msg) {
    jsonrpc::write_message(out_, jsonrpc::make_error(id, code, std::move(msg)),
                            jsonrpc::Framing::LspHeaders);
}
void LspServer::send_notification(std::string method, json::Value params) {
    json::Object o;
    o.emplace_back("jsonrpc", json::Value{"2.0"});
    o.emplace_back("method",  json::Value{std::move(method)});
    o.emplace_back("params",  std::move(params));
    jsonrpc::write_message(out_, json::Value{std::move(o)},
                            jsonrpc::Framing::LspHeaders);
}

namespace {

// Translate a SourcePos -> {line, character} LSP Position (0-based).
json::Value lsp_position(lang::SourcePos p) {
    json::Object o;
    o.emplace_back("line",      json::Value{static_cast<std::int64_t>(p.line - 1)});
    o.emplace_back("character", json::Value{static_cast<std::int64_t>(p.column - 1)});
    return json::Value{std::move(o)};
}

json::Value lsp_range(lang::SourcePos start, std::uint32_t length) {
    json::Object o;
    o.emplace_back("start", lsp_position(start));
    lang::SourcePos endp = start;
    endp.column += length;
    o.emplace_back("end", lsp_position(endp));
    return json::Value{std::move(o)};
}

int diag_severity_lsp(lang::Diagnostic::Severity s) {
    switch (s) {
        case lang::Diagnostic::Severity::Error:   return 1;
        case lang::Diagnostic::Severity::Warning: return 2;
        case lang::Diagnostic::Severity::Info:    return 3;
        case lang::Diagnostic::Severity::Hint:    return 4;
    }
    return 1;
}

}  // namespace

void LspServer::publish_diagnostics(const std::string& uri, const Document& doc) {
    lang::Parser parser(doc.text);
    parser.parse();
    json::Array arr;
    for (const auto& d : parser.diagnostics()) {
        json::Object o;
        o.emplace_back("range",    lsp_range(d.pos, d.length));
        o.emplace_back("severity", json::Value{static_cast<std::int64_t>(diag_severity_lsp(d.severity))});
        if (!d.code.empty())
            o.emplace_back("code",     json::Value{d.code});
        o.emplace_back("source",   json::Value{"opennuts"});
        o.emplace_back("message",  json::Value{d.message});
        arr.push_back(json::Value{std::move(o)});
    }
    json::Object params;
    params.emplace_back("uri",         json::Value{uri});
    params.emplace_back("version",     json::Value{doc.version});
    params.emplace_back("diagnostics", json::Value{std::move(arr)});
    send_notification("textDocument/publishDiagnostics",
                      json::Value{std::move(params)});
}

Status LspServer::dispatch(const json::Value& msg) {
    const json::Value* method = msg.find("method");
    if (!method || !method->is_string()) {
        // Responses to requests we issued land here; we don't issue any
        // server-to-client requests yet.
        return Status::Ok;
    }
    std::string m{method->as_string()};
    const json::Value* idv    = msg.find("id");
    const json::Value* params = msg.find("params");
    json::Value id = idv ? *idv : json::Value{};

    if (m == "initialize") {
        // Reply with our capabilities.
        json::Object caps;
        caps.emplace_back("textDocumentSync",
                          json::Value{static_cast<std::int64_t>(1)});  // Full sync
        caps.emplace_back("hoverProvider", json::Value{true});
        json::Object server_info;
        server_info.emplace_back("name",    json::Value{"opennuts-lsp"});
        server_info.emplace_back("version", json::Value{VERSION_STRING});
        json::Object result;
        result.emplace_back("capabilities", json::Value{std::move(caps)});
        result.emplace_back("serverInfo",   json::Value{std::move(server_info)});
        send_response(id, json::Value{std::move(result)});
        initialized_ = true;
        return Status::Ok;
    }
    if (m == "initialized") return Status::Ok;
    if (m == "shutdown") {
        shutdown_ = true;
        send_response(id, json::Value{});
        return Status::Ok;
    }
    if (m == "exit") {
        return Status::Io;  // signal loop to exit
    }
    if (m == "textDocument/didOpen") {
        if (!params) return Status::Ok;
        const json::Value* td = params->find("textDocument");
        if (!td) return Status::Ok;
        std::string uri{td->find("uri") ? td->find("uri")->as_string() : ""};
        std::string text{td->find("text") ? td->find("text")->as_string() : ""};
        std::int64_t ver = td->find("version") ? td->find("version")->as_int() : 0;
        Document doc; doc.text = std::move(text); doc.version = ver;
        docs_[uri] = doc;
        publish_diagnostics(uri, docs_[uri]);
        return Status::Ok;
    }
    if (m == "textDocument/didChange") {
        if (!params) return Status::Ok;
        const json::Value* td = params->find("textDocument");
        const json::Value* chs = params->find("contentChanges");
        if (!td || !chs || !chs->is_array()) return Status::Ok;
        std::string uri{td->find("uri") ? td->find("uri")->as_string() : ""};
        std::int64_t ver = td->find("version") ? td->find("version")->as_int() : 0;
        auto it = docs_.find(uri);
        if (it == docs_.end()) return Status::Ok;
        // Full-sync only.
        const auto& arr = chs->as_array();
        if (!arr.empty()) {
            const json::Value& last = arr.back();
            const json::Value* tx = last.find("text");
            if (tx && tx->is_string()) it->second.text = std::string{tx->as_string()};
        }
        it->second.version = ver;
        publish_diagnostics(uri, it->second);
        return Status::Ok;
    }
    if (m == "textDocument/didClose") {
        if (params) {
            const json::Value* td = params->find("textDocument");
            if (td) {
                std::string uri{td->find("uri") ? td->find("uri")->as_string() : ""};
                docs_.erase(uri);
            }
        }
        return Status::Ok;
    }
    if (m == "textDocument/hover") {
        // Look up the word under the cursor; if it's a primitive
        // keyword we return a short description.
        if (!params) { send_response(id, json::Value{}); return Status::Ok; }
        const json::Value* td  = params->find("textDocument");
        const json::Value* pos = params->find("position");
        if (!td || !pos) { send_response(id, json::Value{}); return Status::Ok; }
        std::string uri{td->find("uri") ? td->find("uri")->as_string() : ""};
        auto it = docs_.find(uri);
        if (it == docs_.end()) { send_response(id, json::Value{}); return Status::Ok; }
        std::int64_t line = pos->find("line")      ? pos->find("line")->as_int()      : 0;
        std::int64_t col  = pos->find("character") ? pos->find("character")->as_int() : 0;

        // Locate the byte offset of (line, col).
        const std::string& src = it->second.text;
        std::size_t off = 0; std::int64_t ln = 0;
        while (off < src.size() && ln < line) {
            if (src[off++] == '\n') ++ln;
        }
        std::size_t col_off = off + static_cast<std::size_t>(col);
        if (col_off > src.size()) col_off = src.size();
        // Expand to word boundaries.
        auto is_word = [](char c) {
            return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                   (c >= '0' && c <= '9') || c == '_';
        };
        std::size_t a = col_off;
        while (a > 0 && is_word(src[a - 1])) --a;
        std::size_t b = col_off;
        while (b < src.size() && is_word(src[b])) ++b;
        std::string word = src.substr(a, b - a);
        const char* desc = nullptr;
        if (word == "box")      desc = "box(width, depth, height) -- axis-aligned solid box.";
        else if (word == "sphere")   desc = "sphere(radius) -- solid sphere.";
        else if (word == "cylinder") desc = "cylinder(radius, height) -- right circular cylinder.";
        else if (word == "cone")     desc = "cone(radius_base, radius_top, height) -- cone or frustum.";
        else if (word == "torus")    desc = "torus(major_radius, minor_radius) -- torus.";
        else if (word == "unit")     desc = "unit MM|CM|M|IN|FT; -- set the active length unit.";
        else if (word == "body")     desc = "body NAME { ... } -- declare a new model body.";

        if (!desc) { send_response(id, json::Value{}); return Status::Ok; }
        json::Object res;
        json::Object contents;
        contents.emplace_back("kind",  json::Value{"plaintext"});
        contents.emplace_back("value", json::Value{std::string{desc}});
        res.emplace_back("contents", json::Value{std::move(contents)});
        send_response(id, json::Value{std::move(res)});
        return Status::Ok;
    }
    // Unknown methods: only error for requests, not notifications.
    if (idv && !idv->is_null()) {
        send_error(id, jsonrpc::kMethodNotFound, "method not found: " + m);
    }
    return Status::Ok;
}

Status LspServer::run() {
    for (;;) {
        auto msg = jsonrpc::read_message(in_, jsonrpc::Framing::LspHeaders);
        if (!msg) {
            if (msg.status() == Status::Io) return Status::Ok;  // peer closed
            return msg.status();
        }
        Status s = dispatch(msg.value());
        if (s == Status::Io) return Status::Ok;  // exit notification
        if (s != Status::Ok) {
            // Continue serving; per-request errors are reported via send_error.
        }
    }
}

}  // namespace gmk::lsp
