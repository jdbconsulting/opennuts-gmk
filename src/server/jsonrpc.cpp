#include "gmk/server/jsonrpc.hpp"

#include <cctype>
#include <cstring>
#include <istream>
#include <ostream>
#include <string>

namespace gmk::jsonrpc {

namespace {

// Read until "\r\n" or "\n". Returns Status::Io on EOF.
Status read_line(std::istream& in, std::string& out) {
    out.clear();
    char c;
    while (in.get(c)) {
        if (c == '\r') {
            if (in.peek() == '\n') in.get();
            return Status::Ok;
        }
        if (c == '\n') return Status::Ok;
        out.push_back(c);
    }
    return Status::Io;
}

}  // namespace

Result<json::Value> read_message(std::istream& in, Framing framing) {
    if (framing == Framing::NewlineDelimited) {
        std::string line;
        Status s = read_line(in, line);
        if (s != Status::Ok) return s;
        if (line.empty())    return Status::Io;
        std::size_t off = 0;
        auto v = json::parse(line, &off);
        if (!v) return v.status();
        return v;
    }

    // LSP / Content-Length framed.
    std::size_t content_len = 0;
    bool have_len = false;
    std::string line;
    for (;;) {
        Status s = read_line(in, line);
        if (s != Status::Ok) return s;
        if (line.empty()) break;  // end of headers

        // Parse "Content-Length: N" (case-insensitive prefix).
        const char* prefix = "Content-Length:";
        std::size_t pl = std::strlen(prefix);
        if (line.size() > pl) {
            bool ok = true;
            for (std::size_t i = 0; i < pl; ++i) {
                char a = static_cast<char>(std::tolower(static_cast<unsigned char>(line[i])));
                char b = static_cast<char>(std::tolower(static_cast<unsigned char>(prefix[i])));
                if (a != b) { ok = false; break; }
            }
            if (ok) {
                const char* q = line.data() + pl;
                const char* qend = line.data() + line.size();
                while (q < qend && (*q == ' ' || *q == '\t')) ++q;
                content_len = 0;
                while (q < qend && *q >= '0' && *q <= '9') {
                    content_len = content_len * 10 + static_cast<std::size_t>(*q - '0');
                    ++q;
                }
                have_len = true;
            }
        }
    }
    if (!have_len) return Status::Parse;

    std::string buf;
    buf.resize(content_len);
    if (!in.read(buf.data(), static_cast<std::streamsize>(content_len))) {
        return Status::Io;
    }
    std::size_t off = 0;
    auto v = json::parse(buf, &off);
    if (!v) return v.status();
    return v;
}

Status write_message(std::ostream& out, const json::Value& v, Framing framing) {
    std::string body = json::write(v, /*compact=*/true);
    if (framing == Framing::NewlineDelimited) {
        out << body << "\n";
        out.flush();
        return out.good() ? Status::Ok : Status::Io;
    }
    out << "Content-Length: " << body.size() << "\r\n\r\n";
    out.write(body.data(), static_cast<std::streamsize>(body.size()));
    out.flush();
    return out.good() ? Status::Ok : Status::Io;
}

json::Value make_result(const json::Value& id, json::Value result) {
    json::Object o;
    o.emplace_back("jsonrpc", json::Value{"2.0"});
    o.emplace_back("id",      id);
    o.emplace_back("result",  std::move(result));
    return json::Value{std::move(o)};
}

json::Value make_error(const json::Value& id, int code, std::string message) {
    json::Object err;
    err.emplace_back("code",    json::Value{static_cast<std::int64_t>(code)});
    err.emplace_back("message", json::Value{std::move(message)});
    json::Object o;
    o.emplace_back("jsonrpc", json::Value{"2.0"});
    o.emplace_back("id",      id);
    o.emplace_back("error",   json::Value{std::move(err)});
    return json::Value{std::move(o)};
}

}  // namespace gmk::jsonrpc
