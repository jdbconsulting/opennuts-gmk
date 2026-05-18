#include "gmk/server/json.hpp"

#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>

namespace gmk::json {

// ---------------------------------------------------------------------------
// Value lifetime management.
// ---------------------------------------------------------------------------
void Value::destroy() noexcept {
    switch (t_) {
        case Type::String:  v_.s.~basic_string();    break;
        case Type::ArrayT:   v_.a.~Array();           break;
        case Type::ObjectT:  v_.o.~Object();          break;
        default: break;
    }
    t_ = Type::Null;
}
void Value::construct_from(const Value& other) {
    t_ = other.t_;
    switch (t_) {
        case Type::Null:                                  break;
        case Type::Bool:   v_.b = other.v_.b;             break;
        case Type::Int:    v_.i = other.v_.i;             break;
        case Type::Double: v_.d = other.v_.d;             break;
        case Type::String: new (&v_.s) std::string(other.v_.s);  break;
        case Type::ArrayT:  new (&v_.a) Array(other.v_.a);        break;
        case Type::ObjectT: new (&v_.o) Object(other.v_.o);       break;
    }
}
void Value::construct_from(Value&& other) noexcept {
    t_ = other.t_;
    switch (t_) {
        case Type::Null:                                  break;
        case Type::Bool:   v_.b = other.v_.b;             break;
        case Type::Int:    v_.i = other.v_.i;             break;
        case Type::Double: v_.d = other.v_.d;             break;
        case Type::String: new (&v_.s) std::string(std::move(other.v_.s));  break;
        case Type::ArrayT:  new (&v_.a) Array(std::move(other.v_.a));        break;
        case Type::ObjectT: new (&v_.o) Object(std::move(other.v_.o));       break;
    }
    other.destroy();
}
Value& Value::operator=(const Value& other) {
    if (this != &other) { destroy(); construct_from(other); }
    return *this;
}
Value& Value::operator=(Value&& other) noexcept {
    if (this != &other) { destroy(); construct_from(std::move(other)); }
    return *this;
}

std::int64_t Value::as_int(std::int64_t def) const noexcept {
    if (t_ == Type::Int)    return v_.i;
    if (t_ == Type::Double) return static_cast<std::int64_t>(v_.d);
    if (t_ == Type::Bool)   return v_.b ? 1 : 0;
    return def;
}
double Value::as_double(double def) const noexcept {
    if (t_ == Type::Double) return v_.d;
    if (t_ == Type::Int)    return static_cast<double>(v_.i);
    if (t_ == Type::Bool)   return v_.b ? 1.0 : 0.0;
    return def;
}

const Value* Value::find(std::string_view key) const noexcept {
    if (t_ != Type::ObjectT) return nullptr;
    for (const auto& kv : v_.o) if (kv.first == key) return &kv.second;
    return nullptr;
}
Value* Value::find(std::string_view key) noexcept {
    if (t_ != Type::ObjectT) return nullptr;
    for (auto& kv : v_.o) if (kv.first == key) return &kv.second;
    return nullptr;
}
Value& Value::operator[](std::string_view key) {
    if (t_ != Type::ObjectT) { destroy(); t_ = Type::ObjectT; new (&v_.o) Object(); }
    for (auto& kv : v_.o) if (kv.first == key) return kv.second;
    v_.o.emplace_back(std::string(key), Value{});
    return v_.o.back().second;
}

// ---------------------------------------------------------------------------
// Parser.
// ---------------------------------------------------------------------------
namespace {

struct Parser {
    const char* p;
    const char* end;
    const char* begin;

    void skip_ws() {
        while (p < end) {
            char c = *p;
            if (c == ' ' || c == '\t' || c == '\n' || c == '\r') ++p;
            else break;
        }
    }
    bool eof() const { return p >= end; }
    std::size_t offset() const { return static_cast<std::size_t>(p - begin); }

    Result<Value> parse_value() {
        skip_ws();
        if (eof()) return Status::Parse;
        char c = *p;
        if (c == '{') return parse_object();
        if (c == '[') return parse_array();
        if (c == '"') return parse_string();
        if (c == 't' || c == 'f') return parse_bool();
        if (c == 'n') return parse_null();
        if (c == '-' || (c >= '0' && c <= '9')) return parse_number();
        return Status::Parse;
    }
    Result<Value> parse_null() {
        if (end - p >= 4 && std::memcmp(p, "null", 4) == 0) { p += 4; return Value{}; }
        return Status::Parse;
    }
    Result<Value> parse_bool() {
        if (end - p >= 4 && std::memcmp(p, "true",  4) == 0) { p += 4; return Value{true}; }
        if (end - p >= 5 && std::memcmp(p, "false", 5) == 0) { p += 5; return Value{false}; }
        return Status::Parse;
    }
    Result<Value> parse_number() {
        const char* start = p;
        if (*p == '-') ++p;
        bool is_float = false;
        while (p < end && (*p >= '0' && *p <= '9')) ++p;
        if (p < end && *p == '.') { is_float = true; ++p;
            while (p < end && (*p >= '0' && *p <= '9')) ++p;
        }
        if (p < end && (*p == 'e' || *p == 'E')) {
            is_float = true; ++p;
            if (p < end && (*p == '+' || *p == '-')) ++p;
            while (p < end && (*p >= '0' && *p <= '9')) ++p;
        }
        std::string tok(start, p);
        if (is_float) {
            char* eptr = nullptr;
            double d = std::strtod(tok.c_str(), &eptr);
            if (eptr != tok.c_str() + tok.size()) return Status::Parse;
            return Value{d};
        } else {
            char* eptr = nullptr;
            long long ll = std::strtoll(tok.c_str(), &eptr, 10);
            if (eptr != tok.c_str() + tok.size()) return Status::Parse;
            return Value{static_cast<std::int64_t>(ll)};
        }
    }
    Result<Value> parse_string() {
        std::string s;
        Status st = parse_string_to(s);
        if (st != Status::Ok) return st;
        return Value{std::move(s)};
    }
    Status parse_string_to(std::string& out) {
        if (eof() || *p != '"') return Status::Parse;
        ++p;
        out.clear();
        while (p < end) {
            char c = *p++;
            if (c == '"') return Status::Ok;
            if (c == '\\') {
                if (p >= end) return Status::Parse;
                char esc = *p++;
                switch (esc) {
                    case '"': out.push_back('"'); break;
                    case '\\': out.push_back('\\'); break;
                    case '/':  out.push_back('/');  break;
                    case 'b':  out.push_back('\b'); break;
                    case 'f':  out.push_back('\f'); break;
                    case 'n':  out.push_back('\n'); break;
                    case 'r':  out.push_back('\r'); break;
                    case 't':  out.push_back('\t'); break;
                    case 'u': {
                        if (end - p < 4) return Status::Parse;
                        unsigned cp = 0;
                        for (int i = 0; i < 4; ++i) {
                            char h = *p++;
                            unsigned v;
                            if (h >= '0' && h <= '9')      v = static_cast<unsigned>(h - '0');
                            else if (h >= 'a' && h <= 'f') v = static_cast<unsigned>(h - 'a' + 10);
                            else if (h >= 'A' && h <= 'F') v = static_cast<unsigned>(h - 'A' + 10);
                            else return Status::Parse;
                            cp = (cp << 4) | v;
                        }
                        // Encode UTF-8.
                        if (cp < 0x80) {
                            out.push_back(static_cast<char>(cp));
                        } else if (cp < 0x800) {
                            out.push_back(static_cast<char>(0xC0 | (cp >> 6)));
                            out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
                        } else {
                            out.push_back(static_cast<char>(0xE0 | (cp >> 12)));
                            out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
                            out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
                        }
                        break;
                    }
                    default: return Status::Parse;
                }
            } else {
                out.push_back(c);
            }
        }
        return Status::Parse;
    }
    Result<Value> parse_array() {
        ++p;  // consume '['
        Array a;
        skip_ws();
        if (!eof() && *p == ']') { ++p; return Value{std::move(a)}; }
        while (!eof()) {
            auto v = parse_value();
            if (!v) return v.status();
            a.push_back(std::move(v).value());
            skip_ws();
            if (eof()) return Status::Parse;
            if (*p == ',') { ++p; skip_ws(); continue; }
            if (*p == ']') { ++p; return Value{std::move(a)}; }
            return Status::Parse;
        }
        return Status::Parse;
    }
    Result<Value> parse_object() {
        ++p;  // consume '{'
        Object o;
        skip_ws();
        if (!eof() && *p == '}') { ++p; return Value{std::move(o)}; }
        while (!eof()) {
            skip_ws();
            if (eof() || *p != '"') return Status::Parse;
            std::string key;
            Status st = parse_string_to(key);
            if (st != Status::Ok) return st;
            skip_ws();
            if (eof() || *p != ':') return Status::Parse;
            ++p;
            auto v = parse_value();
            if (!v) return v.status();
            o.emplace_back(std::move(key), std::move(v).value());
            skip_ws();
            if (eof()) return Status::Parse;
            if (*p == ',') { ++p; continue; }
            if (*p == '}') { ++p; return Value{std::move(o)}; }
            return Status::Parse;
        }
        return Status::Parse;
    }
};

}  // namespace

Result<Value> parse(std::string_view text, std::size_t* err_off) {
    Parser p{text.data(), text.data() + text.size(), text.data()};
    auto v = p.parse_value();
    if (!v) {
        if (err_off) *err_off = p.offset();
        return v.status();
    }
    p.skip_ws();
    if (p.p != p.end) {
        if (err_off) *err_off = p.offset();
        return Status::Parse;
    }
    return v;
}

// ---------------------------------------------------------------------------
// Writer.
// ---------------------------------------------------------------------------
namespace {

void write_string(std::string& out, std::string_view s) {
    out.push_back('"');
    for (char c : s) {
        switch (c) {
            case '"':  out.append("\\\"");  break;
            case '\\': out.append("\\\\");  break;
            case '\b': out.append("\\b");   break;
            case '\f': out.append("\\f");   break;
            case '\n': out.append("\\n");   break;
            case '\r': out.append("\\r");   break;
            case '\t': out.append("\\t");   break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned>(c));
                    out.append(buf);
                } else {
                    out.push_back(c);
                }
        }
    }
    out.push_back('"');
}

void write_value(std::string& out, const Value& v, bool compact, int indent, int level) {
    switch (v.type()) {
        case Type::Null:   out.append("null"); break;
        case Type::Bool:   out.append(v.as_bool() ? "true" : "false"); break;
        case Type::Int: {
            char buf[32];
            std::snprintf(buf, sizeof(buf), "%lld",
                          static_cast<long long>(v.as_int()));
            out.append(buf); break;
        }
        case Type::Double: {
            char buf[40];
            double d = v.as_double();
            if (std::isnan(d) || std::isinf(d)) {
                out.append("null");
            } else {
                std::snprintf(buf, sizeof(buf), "%.17g", d);
                out.append(buf);
            }
            break;
        }
        case Type::String: write_string(out, v.as_string()); break;
        case Type::ArrayT: {
            const auto& a = v.as_array();
            if (a.empty()) { out.append("[]"); break; }
            out.push_back('[');
            for (std::size_t i = 0; i < a.size(); ++i) {
                if (!compact) {
                    out.push_back('\n');
                    out.append(static_cast<std::size_t>(indent * (level + 1)), ' ');
                }
                write_value(out, a[i], compact, indent, level + 1);
                if (i + 1 < a.size()) out.push_back(',');
            }
            if (!compact) {
                out.push_back('\n');
                out.append(static_cast<std::size_t>(indent * level), ' ');
            }
            out.push_back(']');
            break;
        }
        case Type::ObjectT: {
            const auto& o = v.as_object();
            if (o.empty()) { out.append("{}"); break; }
            out.push_back('{');
            for (std::size_t i = 0; i < o.size(); ++i) {
                if (!compact) {
                    out.push_back('\n');
                    out.append(static_cast<std::size_t>(indent * (level + 1)), ' ');
                }
                write_string(out, o[i].first);
                out.push_back(':');
                if (!compact) out.push_back(' ');
                write_value(out, o[i].second, compact, indent, level + 1);
                if (i + 1 < o.size()) out.push_back(',');
            }
            if (!compact) {
                out.push_back('\n');
                out.append(static_cast<std::size_t>(indent * level), ' ');
            }
            out.push_back('}');
            break;
        }
    }
}

}  // namespace

std::string write(const Value& v, bool compact, int indent) {
    std::string out;
    out.reserve(64);
    write_value(out, v, compact, indent, 0);
    return out;
}

}  // namespace gmk::json
