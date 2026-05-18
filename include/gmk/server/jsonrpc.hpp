#pragma once
//
// Minimal JSON-RPC 2.0 framing used by the LSP server (Content-Length
// headers) and by the geometry server (one message per line, optional).
//
// Both transports read from an std::istream and write to an std::ostream,
// so the same code drives stdio or a TCP-attached socket adapter.
//

#include <functional>
#include <iosfwd>
#include <string>
#include <string_view>

#include "gmk/result.hpp"
#include "gmk/server/json.hpp"

namespace gmk::jsonrpc {

enum class Framing {
    LspHeaders,  // Content-Length: N\r\n\r\n<body>
    NewlineDelimited,  // one JSON value per line
};

// Read one framed JSON-RPC message. Returns Status::Io on EOF/IO error,
// Status::Parse on a malformed frame.
Result<json::Value> read_message(std::istream& in, Framing framing);

// Write a framed JSON-RPC message.
Status write_message(std::ostream& out, const json::Value& v, Framing framing);

// Build a JSON-RPC 2.0 result/error response with the given id (which may
// be Null).
json::Value make_result(const json::Value& id, json::Value result);
json::Value make_error(const json::Value& id, int code, std::string message);

// Standard error codes.
constexpr int kParseError       = -32700;
constexpr int kInvalidRequest   = -32600;
constexpr int kMethodNotFound   = -32601;
constexpr int kInvalidParams    = -32602;
constexpr int kInternalError    = -32603;

}  // namespace gmk::jsonrpc
