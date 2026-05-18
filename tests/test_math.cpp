#include "test_main.hpp"

#include <cstdlib>

#include "gmk/math/aabb.hpp"
#include "gmk/math/transform.hpp"
#include "gmk/math/vec.hpp"

using namespace gmk;

GMK_TEST("math: Vec3d arithmetic") {
    Vec3d a{1, 2, 3}, b{4, -5, 6};
    Vec3d c = a + b;
    GMK_EXPECT_NEAR(c.x, 5, 1e-12);
    GMK_EXPECT_NEAR(c.y, -3, 1e-12);
    GMK_EXPECT_NEAR(c.z, 9, 1e-12);
    GMK_EXPECT_NEAR(a.dot(b), 1*4 + 2*-5 + 3*6, 1e-12);
    Vec3d cr = Vec3d{1, 0, 0}.cross(Vec3d{0, 1, 0});
    GMK_EXPECT_NEAR(cr.x, 0, 1e-12);
    GMK_EXPECT_NEAR(cr.y, 0, 1e-12);
    GMK_EXPECT_NEAR(cr.z, 1, 1e-12);
}

GMK_TEST("math: rotation_z(π/2) maps X to Y") {
    Mat3d M = Mat3d::rotation_z(PI / 2.0);
    Vec3d r = M * Vec3d{1, 0, 0};
    GMK_EXPECT_NEAR(r.x, 0, 1e-12);
    GMK_EXPECT_NEAR(r.y, 1, 1e-12);
    GMK_EXPECT_NEAR(r.z, 0, 1e-12);
}

GMK_TEST("math: AABB includes points") {
    AABB box;
    GMK_EXPECT(box.empty());
    box.include(Vec3i{0, 0, 0});
    box.include(Vec3i{10, 20, 30});
    box.include(Vec3i{-5, 7, 5});
    GMK_EXPECT_EQ(box.min.x, -5);
    GMK_EXPECT_EQ(box.max.z, 30);
}

GMK_TEST("math: Transform identity round-trips to within ~fm") {
    Transform t;
    Vec3i p{m_to_length(1.0), m_to_length(2.0), m_to_length(3.0)};
    bool sat = false;
    Vec3i q = t.apply(p, &sat);
    GMK_EXPECT(!sat);
    // Round-trip via double may incur sub-fm drift. The kernel's stated
    // working tolerance is 1 pm = 1000 fm; we hold ourselves to that here.
    GMK_EXPECT(std::abs(p.x - q.x) <= 1000);
    GMK_EXPECT(std::abs(p.y - q.y) <= 1000);
    GMK_EXPECT(std::abs(p.z - q.z) <= 1000);
}

GMK_TEST("math: Transform composition is inverse-aware") {
    Transform t;
    t.rotation = Mat3d::rotation_z(PI / 4);
    t.translation = Vec3i{mm_to_length(10), mm_to_length(20), 0};
    Transform tinv = t.inverse();
    Transform composed = t * tinv;
    Vec3i p{mm_to_length(5), mm_to_length(7), mm_to_length(-3)};
    Vec3i out = composed.apply(p);
    GMK_EXPECT_NEAR(length_to_mm(out.x - p.x), 0.0, 1e-3);
    GMK_EXPECT_NEAR(length_to_mm(out.y - p.y), 0.0, 1e-3);
    GMK_EXPECT_NEAR(length_to_mm(out.z - p.z), 0.0, 1e-3);
}
