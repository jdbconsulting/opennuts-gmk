#pragma once
//
// Tiny, dependency-free JSON parser + writer.
//
// We deliberately keep the surface minimal: it parses and emits the seven
// JSON value types and nothing more. There is no schema layer, no streaming
// parser, no fancy error messages -- callers get a Status::Parse and the
// failing offset on error and that's it.
//
// JSON values are stored in a tagged union; nested arrays/objects are
// owned by the parent value. Strings are stored as std::string (UTF-8).
//

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include "gmk/result.hpp"

namespace gmk::json {

class Value;
using Object = std::vector<std::pair<std::string, Value>>;
using Array  = std::vector<Value>;

enum class Type : std::uint8_t {
    Null,
    Bool,
    Int,
    Double,
    String,
    ArrayT,
    ObjectT,
};
// Backwards-compatible spelling for callers that expect Type::ArrayT.
inline constexpr Type kArray  = Type::ArrayT;
inline constexpr Type kObject = Type::ObjectT;

class Value {
public:
    Value() noexcept                      : t_{Type::Null}    {}
    explicit Value(std::nullptr_t)        : t_{Type::Null}    {}
    explicit Value(bool b)                : t_{Type::Bool}    { v_.b = b; }
    explicit Value(int i)                 : t_{Type::Int}     { v_.i = i; }
    explicit Value(long i)                : t_{Type::Int}     { v_.i = static_cast<std::int64_t>(i); }
    explicit Value(long long i)           : t_{Type::Int}     { v_.i = static_cast<std::int64_t>(i); }
    explicit Value(double d)              : t_{Type::Double}  { v_.d = d; }
    explicit Value(const char* s)         : t_{Type::String}  { new (&v_.s) std::string(s); }
    explicit Value(std::string s)         : t_{Type::String}  { new (&v_.s) std::string(std::move(s)); }
    explicit Value(Array a)               : t_{Type::ArrayT}   { new (&v_.a) Array(std::move(a)); }
    explicit Value(Object o)              : t_{Type::ObjectT}  { new (&v_.o) Object(std::move(o)); }

    Value(const Value& other)             { construct_from(other); }
    Value(Value&& other) noexcept         { construct_from(std::move(other)); }
    Value& operator=(const Value& other);
    Value& operator=(Value&& other) noexcept;
    ~Value()                              { destroy(); }

    Type type() const noexcept { return t_; }
    bool is_null()   const noexcept { return t_ == Type::Null; }
    bool is_bool()   const noexcept { return t_ == Type::Bool; }
    bool is_int()    const noexcept { return t_ == Type::Int; }
    bool is_double() const noexcept { return t_ == Type::Double; }
    bool is_number() const noexcept { return t_ == Type::Int || t_ == Type::Double; }
    bool is_string() const noexcept { return t_ == Type::String; }
    bool is_array()  const noexcept { return t_ == Type::ArrayT; }
    bool is_object() const noexcept { return t_ == Type::ObjectT; }

    bool                    as_bool(bool       def = false) const noexcept { return is_bool()   ? v_.b : def; }
    std::int64_t            as_int (std::int64_t def = 0)   const noexcept;
    double                  as_double(double   def = 0.0)   const noexcept;
    std::string_view        as_string(std::string_view def = "") const noexcept {
        return is_string() ? std::string_view{v_.s} : def;
    }
    const Array&            as_array()  const { return v_.a; }
    Array&                  as_array()        { return v_.a; }
    const Object&           as_object() const { return v_.o; }
    Object&                 as_object()       { return v_.o; }

    // Object access: returns nullptr if not an object or key missing.
    const Value* find(std::string_view key) const noexcept;
    Value*       find(std::string_view key)       noexcept;
    Value&       operator[](std::string_view key);   // creates entry if absent

private:
    void destroy() noexcept;
    void construct_from(const Value& other);
    void construct_from(Value&& other) noexcept;

    Type t_{Type::Null};
    union Storage {
        Storage() {}
        ~Storage() {}
        bool         b;
        std::int64_t i;
        double       d;
        std::string  s;
        Array        a;
        Object       o;
    } v_;
};

// Parse a JSON document. Returns the value on success; on failure the
// status is Status::Parse and ``error_offset`` (if non-null) receives the
// byte offset of the first unexpected character.
Result<Value> parse(std::string_view text, std::size_t* error_offset = nullptr);

// Serialise. ``compact`` skips whitespace; ``indent`` is the number of
// spaces per level when not compact.
std::string write(const Value& v, bool compact = true, int indent = 2);

}  // namespace gmk::json
