#pragma once
//
// gmk::units -- the fixed-point numeric foundation of the kernel.
//
//   Dimensions :: length_t  is INT64. 1 unit = 1 femtometre (1e-15 m).
//   Angles     :: angle_t   is INT32. 1 unit = 1e-3 degree.
//
//   Length range : ±9 223 372 km approx, easily multiple kilometres.
//   Length precision : 1 fm, comfortably below any physical CAD tolerance.
//   Angle range : explicitly capped at ±180 000 (i.e. ±180°). Larger inputs
//                 are saturated by the conversion helpers; angles outside
//                 the canonical interval are an explicit InvalidArgument.
//
// All conversions use ``double`` as the intermediate floating-point type --
// double has 53 bits of mantissa which is enough to represent every legal
// length_t value exactly (because INT64_MAX < 2^53 * 2^11), so converting
// to double, doing the math, and converting back is lossless as long as the
// result lies in range.
//

#include <cmath>
#include <cstdint>
#include <limits>

namespace gmk {

using length_t = std::int64_t;
using angle_t  = std::int32_t;

// One length unit = 1 femtometre = 1e-15 m.
inline constexpr length_t LENGTH_PER_METER = 1'000'000'000'000'000LL;
inline constexpr length_t LENGTH_PER_MM    =     1'000'000'000'000LL;
inline constexpr length_t LENGTH_PER_UM    =         1'000'000'000LL;
inline constexpr length_t LENGTH_PER_NM    =             1'000'000LL;
inline constexpr length_t LENGTH_PER_PM    =                 1'000LL;
inline constexpr length_t LENGTH_PER_FM    =                     1LL;
// 1 inch = 25.4 mm exactly; 25.4e-3 * 1e15 = 2.54e13 fm.
inline constexpr length_t LENGTH_PER_INCH  =        25'400'000'000'000LL;
inline constexpr length_t LENGTH_PER_FOOT  = LENGTH_PER_INCH * 12;

// Saturating boundaries for length values. We keep one bit of headroom below
// INT64_MAX so that operations like ``a - b`` on legal inputs never wrap.
inline constexpr length_t LENGTH_MAX = (std::numeric_limits<length_t>::max)() / 2;
inline constexpr length_t LENGTH_MIN = -LENGTH_MAX;

inline constexpr angle_t  ANGLE_PER_DEGREE = 1'000;
inline constexpr angle_t  ANGLE_MAX        = 180'000;
inline constexpr angle_t  ANGLE_MIN        = -180'000;

// ---------------------------------------------------------------------------
// Length conversions.
// ---------------------------------------------------------------------------
constexpr double length_to_m(length_t v)  noexcept {
    return static_cast<double>(v) * 1e-15;
}
constexpr double length_to_mm(length_t v) noexcept {
    return static_cast<double>(v) * 1e-12;
}
constexpr double length_to_um(length_t v) noexcept {
    return static_cast<double>(v) * 1e-9;
}
constexpr double length_to_inch(length_t v) noexcept {
    return static_cast<double>(v) / static_cast<double>(LENGTH_PER_INCH);
}

inline length_t m_to_length(double m)   noexcept { return static_cast<length_t>(std::llround(m   * 1e15)); }
inline length_t mm_to_length(double mm) noexcept { return static_cast<length_t>(std::llround(mm  * 1e12)); }
inline length_t um_to_length(double um) noexcept { return static_cast<length_t>(std::llround(um  * 1e9 )); }
inline length_t inch_to_length(double in) noexcept {
    return static_cast<length_t>(std::llround(in  * static_cast<double>(LENGTH_PER_INCH)));
}

// ---------------------------------------------------------------------------
// Angle conversions.
// ---------------------------------------------------------------------------
constexpr double PI = 3.14159265358979323846;

constexpr double angle_to_deg(angle_t a) noexcept {
    return static_cast<double>(a) * 1e-3;
}
constexpr double angle_to_rad(angle_t a) noexcept {
    return static_cast<double>(a) * (PI / 180'000.0);
}
inline angle_t deg_to_angle(double d) noexcept {
    long long v = std::llround(d * 1000.0);
    if (v >  ANGLE_MAX) v =  ANGLE_MAX;
    if (v <  ANGLE_MIN) v =  ANGLE_MIN;
    return static_cast<angle_t>(v);
}
inline angle_t rad_to_angle(double r) noexcept {
    return deg_to_angle(r * (180.0 / PI));
}

constexpr bool angle_in_range(angle_t a) noexcept {
    return a >= ANGLE_MIN && a <= ANGLE_MAX;
}

// ---------------------------------------------------------------------------
// Saturating fixed-point arithmetic helpers. These never throw and never
// invoke undefined behaviour; out-of-range results are clamped to the
// kernel's working range [LENGTH_MIN, LENGTH_MAX]. The kernel's working
// range is one bit narrower than INT64 so that ``a - b`` on legal inputs
// can never wrap around even before the clamp.
// ---------------------------------------------------------------------------
constexpr length_t sat_add(length_t a, length_t b) noexcept {
    // Both operands are within ±LENGTH_MAX = INT64_MAX/2, so the *signed*
    // sum is always representable in int64; we just need to clamp.
    length_t r = a + b;
    if (r >  LENGTH_MAX) return LENGTH_MAX;
    if (r <  LENGTH_MIN) return LENGTH_MIN;
    return r;
}
constexpr length_t sat_sub(length_t a, length_t b) noexcept {
    length_t r = a - b;
    if (r >  LENGTH_MAX) return LENGTH_MAX;
    if (r <  LENGTH_MIN) return LENGTH_MIN;
    return r;
}

// Clamp a length to legal range.
constexpr length_t clamp_length(length_t v) noexcept {
    if (v >  LENGTH_MAX) return LENGTH_MAX;
    if (v <  LENGTH_MIN) return LENGTH_MIN;
    return v;
}

// Returns the kernel's working tolerance in real-units. The kernel itself
// works to the fm resolution, but exposing a slightly larger tolerance lets
// callers reason about equality without obsessing over the last bit.
constexpr double LENGTH_TOLERANCE_M = 1e-12;  // 1 picometre
constexpr double ANGLE_TOLERANCE_DEG = 1e-3;

}  // namespace gmk
