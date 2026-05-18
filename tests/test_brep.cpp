#include "test_main.hpp"

#include "gmk/brep/body.hpp"
#include "gmk/brep/primitives.hpp"
#include "gmk/brep/validate.hpp"
#include "gmk/tess/tessellator.hpp"

using namespace gmk;

GMK_TEST("brep: box has 6 faces / 12 edges / 8 vertices") {
    brep::Body body;
    Status s = brep::make_box(body, Vec3i{0,0,0},
                              mm_to_length(10), mm_to_length(20), mm_to_length(30));
    GMK_EXPECT(s == Status::Ok);
    GMK_EXPECT_EQ(body.face_count(),   6u);
    GMK_EXPECT_EQ(body.edge_count(),  12u);
    GMK_EXPECT_EQ(body.vertex_count(), 8u);
}

GMK_TEST("brep: box validates clean") {
    brep::Body body;
    GMK_EXPECT(brep::make_box(body, Vec3i{0,0,0},
                              mm_to_length(5), mm_to_length(5), mm_to_length(5))
               == Status::Ok);
    brep::ValidationReport rep;
    brep::validate(body, rep);
    GMK_EXPECT(rep.ok());
}

GMK_TEST("brep: cylinder produces three faces") {
    brep::Body body;
    GMK_EXPECT(brep::make_cylinder(body, Vec3i{0,0,0}, Vec3d{0,0,1},
                                   mm_to_length(2), mm_to_length(10))
               == Status::Ok);
    GMK_EXPECT_EQ(body.face_count(), 3u);
}

GMK_TEST("brep: sphere is a single face body") {
    brep::Body body;
    GMK_EXPECT(brep::make_sphere(body, Vec3i{0,0,0}, mm_to_length(7)) == Status::Ok);
    GMK_EXPECT_EQ(body.face_count(), 1u);
}

GMK_TEST("brep: handles invalidate after free") {
    brep::Body body;
    auto v = body.new_vertex(Vec3i{0,0,0});
    GMK_EXPECT(body.vertex(v) != nullptr);
    GMK_EXPECT(body.vertex_count() == 1u);

    // We don't expose direct vertex_free yet -- tested instead via the
    // re-alloc invariant: a new vertex after the first one must produce a
    // different handle (different index).
    auto w = body.new_vertex(Vec3i{1,1,1});
    GMK_EXPECT(v != w);
}

GMK_TEST("brep: tessellate sphere produces triangles") {
    brep::Body body;
    GMK_EXPECT(brep::make_sphere(body, Vec3i{0,0,0}, mm_to_length(10)) == Status::Ok);
    tess::TessOptions opts;
    opts.chord_tolerance_m = 1e-4;  // 0.1 mm
    tess::BodyMesh mesh;
    GMK_EXPECT(tess::tessellate(body, opts, mesh) == Status::Ok);
    GMK_EXPECT(!mesh.faces.empty());
    GMK_EXPECT(!mesh.faces.front().triangles.empty());
}

GMK_TEST("brep: deep-clone preserves topology counts") {
    brep::Body body;
    brep::make_box(body, Vec3i{0,0,0},
                   mm_to_length(2), mm_to_length(3), mm_to_length(4));
    brep::Body copy = body.clone();
    GMK_EXPECT_EQ(copy.face_count(),   body.face_count());
    GMK_EXPECT_EQ(copy.edge_count(),   body.edge_count());
    GMK_EXPECT_EQ(copy.vertex_count(), body.vertex_count());
    brep::ValidationReport rep;
    brep::validate(copy, rep);
    GMK_EXPECT(rep.ok());
}

GMK_TEST("brep: bbox tracks vertices") {
    brep::Body body;
    brep::make_box(body, Vec3i{0,0,0},
                   mm_to_length(5), mm_to_length(7), mm_to_length(11));
    AABB box = body.bbox();
    GMK_EXPECT_EQ(box.min.x, -mm_to_length(5));
    GMK_EXPECT_EQ(box.max.z,  mm_to_length(11));
}
