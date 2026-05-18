#pragma once
//
// gmk::Result<T> -- a lean error-as-value type modelled on std::expected but
// without requiring the C++23 stdlib counterpart. The kernel never throws
// from its hot paths; every fallible operation returns Result<T>. Code that
// has to interop with exceptions (LSP server, JSON parsing) converts at the
// boundary.
//
// The Status enum is intentionally short -- consumers should distinguish the
// few classes of failure that they can recover from, not enumerate every
// possible cause.
//

#include <cstdint>
#include <new>
#include <type_traits>
#include <utility>

namespace gmk {

enum class Status : std::int32_t {
    Ok = 0,

    // Argument problems detectable before doing any work.
    InvalidArgument,
    OutOfRange,        // numeric input outside the supported domain
    DegenerateInput,   // e.g. zero-length vector, collapsed curve

    // Resource problems.
    OutOfMemory,
    Overflow,          // fixed-point arithmetic overflow

    // State problems.
    NotFound,          // entity id does not exist
    InvalidState,      // operation makes no sense in the current state
    TopologyViolation, // brep invariants would be broken

    // Algorithm problems.
    DidNotConverge,
    Singular,          // matrix/jacobian singular

    // External / wiring problems.
    Unsupported,       // feature exists but disabled in this build
    NotImplemented,    // intentional stub
    Io,
    Parse,
    Internal,          // kernel bug -- should never happen
};

constexpr const char* status_name(Status s) noexcept {
    switch (s) {
        case Status::Ok:                 return "Ok";
        case Status::InvalidArgument:    return "InvalidArgument";
        case Status::OutOfRange:         return "OutOfRange";
        case Status::DegenerateInput:    return "DegenerateInput";
        case Status::OutOfMemory:        return "OutOfMemory";
        case Status::Overflow:           return "Overflow";
        case Status::NotFound:           return "NotFound";
        case Status::InvalidState:       return "InvalidState";
        case Status::TopologyViolation:  return "TopologyViolation";
        case Status::DidNotConverge:     return "DidNotConverge";
        case Status::Singular:           return "Singular";
        case Status::Unsupported:        return "Unsupported";
        case Status::NotImplemented:     return "NotImplemented";
        case Status::Io:                 return "Io";
        case Status::Parse:              return "Parse";
        case Status::Internal:           return "Internal";
    }
    return "Unknown";
}

// ---------------------------------------------------------------------------
// Result<T>. Stores either a T or a Status. Trivially destructible if T is.
// ---------------------------------------------------------------------------
template <typename T>
class Result {
public:
    using value_type = T;

    constexpr Result() noexcept : status_{Status::Internal}, has_value_{false} {}
    constexpr Result(Status s) noexcept : status_{s}, has_value_{false} {}

    template <typename U = T,
              std::enable_if_t<std::is_constructible_v<T, U&&>, int> = 0>
    constexpr Result(U&& v) : has_value_{true} {
        ::new (storage_) T(std::forward<U>(v));
        status_ = Status::Ok;
    }

    Result(const Result& o) : status_{o.status_}, has_value_{o.has_value_} {
        if (has_value_) ::new (storage_) T(o.value_ref());
    }
    Result(Result&& o) noexcept(std::is_nothrow_move_constructible_v<T>)
        : status_{o.status_}, has_value_{o.has_value_} {
        if (has_value_) ::new (storage_) T(std::move(o.value_ref()));
    }
    Result& operator=(const Result& o) {
        if (this != &o) { destroy_(); status_ = o.status_; has_value_ = o.has_value_;
                          if (has_value_) ::new (storage_) T(o.value_ref()); }
        return *this;
    }
    Result& operator=(Result&& o)
        noexcept(std::is_nothrow_move_constructible_v<T>) {
        destroy_(); status_ = o.status_; has_value_ = o.has_value_;
        if (has_value_) ::new (storage_) T(std::move(o.value_ref()));
        return *this;
    }
    ~Result() { destroy_(); }

    constexpr bool   ok()       const noexcept { return has_value_; }
    constexpr        operator bool() const noexcept { return has_value_; }
    constexpr Status status()   const noexcept { return status_; }

    T&        value()       &      { return value_ref(); }
    const T&  value() const &      { return value_ref(); }
    T         value() &&           { return std::move(value_ref()); }
    T*        operator->()         { return &value_ref(); }
    const T*  operator->()  const  { return &value_ref(); }
    T&        operator*()   &      { return value_ref(); }
    const T&  operator*()   const& { return value_ref(); }

    template <typename U>
    T value_or(U&& fallback) const& {
        return has_value_ ? value_ref() : T{std::forward<U>(fallback)};
    }

private:
    void destroy_() noexcept {
        if (has_value_) {
            value_ref().~T();
            has_value_ = false;
        }
    }
    T&       value_ref()       { return *std::launder(reinterpret_cast<T*>(storage_)); }
    const T& value_ref() const { return *std::launder(reinterpret_cast<const T*>(storage_)); }

    alignas(T) unsigned char storage_[sizeof(T)]{};
    Status status_{Status::Internal};
    bool   has_value_{false};
};

// Convenience helper: propagate Result up if it failed.
#define GMK_TRY(expr)                                                          \
    do {                                                                       \
        auto _r = (expr);                                                      \
        if (!_r.ok()) return _r.status();                                      \
    } while (0)

#define GMK_TRY_ASSIGN(out, expr)                                              \
    do {                                                                       \
        auto _r = (expr);                                                      \
        if (!_r.ok()) return _r.status();                                      \
        (out) = std::move(_r).value();                                         \
    } while (0)

}  // namespace gmk
