// WASM entry points. We deliberately use a small C ABI so the JS side can
// drive the kernel without depending on Embind. The exported surface is:
//
//   const char* gmk_version();
//   void*       gmk_session_new();
//   void        gmk_session_free(void* session);
//   const char* gmk_session_dispatch(void* session, const char* json_req);
//
// ``gmk_session_dispatch`` returns a pointer to a NUL-terminated UTF-8
// string owned by the session. The string is reused on the next call to
// ``gmk_session_dispatch``, so callers should copy it out before issuing
// another request.

#include <sstream>
#include <string>

#include "gmk/server/geom_server.hpp"
#include "gmk/version.hpp"

extern "C" {

struct GmkSession {
    // Order matters -- streams must be live before the server captures
    // them by reference.
    std::istringstream       null_in;
    std::ostringstream       null_out;
    gmk::geom_server::Server server;
    std::string              last_reply;
    GmkSession()
        : null_in{}, null_out{},
          server{null_in, null_out, gmk::jsonrpc::Framing::NewlineDelimited} {}
};

const char* gmk_version() {
    return gmk::VERSION_STRING;
}

void* gmk_session_new() {
    return new GmkSession();
}

void gmk_session_free(void* p) {
    delete static_cast<GmkSession*>(p);
}

const char* gmk_session_dispatch(void* p, const char* json_req) {
    auto* s = static_cast<GmkSession*>(p);
    if (!s) return "";
    std::size_t off = 0;
    auto parsed = gmk::json::parse(json_req ? json_req : "", &off);
    gmk::json::Value reply;
    if (!parsed) {
        reply = gmk::jsonrpc::make_error(gmk::json::Value{}, gmk::jsonrpc::kParseError,
                                         "parse error");
    } else {
        reply = s->server.dispatch_request(parsed.value());
    }
    s->last_reply = gmk::json::write(reply, /*compact=*/true);
    return s->last_reply.c_str();
}

}  // extern "C"
