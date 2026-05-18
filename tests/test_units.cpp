#include "test_main.hpp"

#include "gmk/math/vec.hpp"
#include "gmk/units.hpp"

using namespace gmk;

GMK_TEST("units: 1 inch == 2.54e13 fm") {
    GMK_EXPECT_EQ(LENGTH_PER_INCH, 25'400'000'000'000LL);
}

GMK_TEST("units: round trip mm <-> length") {
    length_t v = mm_to_length(123.456);
    double back = length_to_mm(v);
    GMK_EXPECT_NEAR(back, 123.456, 1e-9);
}

GMK_TEST("units: angle conversions clamp to ±180°") {
    GMK_EXPECT_EQ(deg_to_angle(200.0), ANGLE_MAX);
    GMK_EXPECT_EQ(deg_to_angle(-200.0), ANGLE_MIN);
    GMK_EXPECT_NEAR(angle_to_deg(45000), 45.0, 1e-9);
    GMK_EXPECT_NEAR(angle_to_rad(180000), PI, 1e-12);
}

GMK_TEST("units: saturating add does not overflow") {
    length_t a = LENGTH_MAX - 1;
    length_t b = 100;
    length_t c = sat_add(a, b);
    GMK_EXPECT_EQ(c, LENGTH_MAX);
}

GMK_TEST("units: distance metric scale") {
    Vec3i a{0, 0, 0};
    Vec3i b{mm_to_length(3.0), mm_to_length(4.0), 0};
    double d = distance_m(a, b);
    GMK_EXPECT_NEAR(d, 0.005, 1e-9);  // 5mm = 0.005m
}
