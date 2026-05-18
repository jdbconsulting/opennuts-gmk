#pragma once
//
// Language-Server-Protocol server for OpenNuts MCAD source.
//
// The server runs over stdio with LSP's Content-Length framing. It
// implements the small set of capabilities that justify writing a server
// in the first place: open/change/close text sync, publishDiagnostics on
// every change, and hover for primitive keywords.
//
// The transport is configurable -- tests inject a fake std::istream /
// std::ostream pair so the protocol can be exercised without spawning a
// real process.
//

#include <iosfwd>
#include <string>
#include <unordered_map>

#include "gmk/result.hpp"
#include "gmk/server/json.hpp"

namespace gmk::lsp {

class LspServer {
public:
    LspServer(std::istream& in, std::ostream& out);

    // Run the event loop until the client closes the connection or
    // explicitly sends ``exit``.
    Status run();

private:
    // Per-document state: the source text and the last parsed diagnostics.
    struct Document {
        std::string text;
        std::int64_t version = 0;
    };

    Status dispatch(const json::Value& msg);
    void   send_response(const json::Value& id, json::Value result);
    void   send_error(const json::Value& id, int code, std::string msg);
    void   send_notification(std::string method, json::Value params);
    void   publish_diagnostics(const std::string& uri, const Document& doc);

    std::istream& in_;
    std::ostream& out_;
    bool          initialized_ = false;
    bool          shutdown_    = false;
    std::unordered_map<std::string, Document> docs_;
};

}  // namespace gmk::lsp
