#include "test_main.hpp"

#include <cmath>

#include "gmk/geom/analytic_curves.hpp"
#include "gmk/geom/analytic_surfaces.hpp"
#include "gmk/geom/nurbs_curve.hpp"
#include "gmk/geom/nurbs_surface.hpp"

using namespace gmk;

GMK_TEST("nurbs: linear B-spline evaluates as polyline") {
    // Open knot vector (0,0,1,2,2), degree 1, control points along x.
    NurbsCurve c;
    double knots[] = {0, 0, 1, 2, 2};
    Vec3d cp[]    = { Vec3d{0,0,0}, Vec3d{1,0,0}, Vec3d{2,0,0} };
    Status s = c.init(1, knots, 5, cp, nullptr, 3);
    GMK_EXPECT(s == Status::Ok);
    Vec3d p;
    GMK_EXPECT(c.point(0.0, p) == Status::Ok);
    GMK_EXPECT_NEAR(p.x, 0.0, 1e-12);
    GMK_EXPECT(c.point(1.5, p) == Status::Ok);
    GMK_EXPECT_NEAR(p.x, 1.5, 1e-12);
    GMK_EXPECT(c.point(2.0, p) == Status::Ok);
    GMK_EXPECT_NEAR(p.x, 2.0, 1e-12);
}

GMK_TEST("nurbs: rational quadratic circle has unit radius") {
    CircleCurve cc;
    GMK_EXPECT(cc.init(Vec3d{0,0,0}, Vec3d{0,0,1}, Vec3d{1,0,0}, 1.0) == Status::Ok);
    NurbsCurve nc;
    GMK_EXPECT(cc.to_nurbs(nc) == Status::Ok);
    // Sample around the circle. Each point should be at unit distance.
    int n_bad = 0;
    for (int i = 0; i < 32; ++i) {
        double u = (2.0 * PI) * i / 32.0;
        Vec3d p;
        nc.point(u, p);
        double r = std::sqrt(p.x*p.x + p.y*p.y);
        if (std::fabs(r - 1.0) > 1e-9) ++n_bad;
    }
    GMK_EXPECT_EQ(n_bad, 0);
}

GMK_TEST("nurbs: curve derivative matches finite difference") {
    NurbsCurve c;
    double knots[] = {0, 0, 0, 1, 2, 3, 3, 3};
    Vec3d cp[]    = { Vec3d{0,0,0}, Vec3d{1,1,0}, Vec3d{2,-1,0},
                       Vec3d{3,2,0}, Vec3d{4,0,0} };
    c.init(2, knots, 8, cp, nullptr, 5);
    Vec3d p, t;
    double u = 1.25;
    c.point_and_tangent(u, p, t);
    Vec3d p1, p0;
    double h = 1e-5;
    c.point(u + h, p1); c.point(u - h, p0);
    Vec3d fd{ (p1.x - p0.x)/(2*h), (p1.y - p0.y)/(2*h), (p1.z - p0.z)/(2*h) };
    GMK_EXPECT_NEAR(t.x, fd.x, 1e-3);
    GMK_EXPECT_NEAR(t.y, fd.y, 1e-3);
    GMK_EXPECT_NEAR(t.z, fd.z, 1e-3);
}

GMK_TEST("nurbs: surface partials match finite difference") {
    // Bilinear patch: degree (1,1), 2x2 control net.
    NurbsSurface s;
    double ku[] = {0,0,1,1};
    double kv[] = {0,0,1,1};
    Vec3d cp[]  = {
        Vec3d{0,0,0}, Vec3d{1,0,1},
        Vec3d{0,1,2}, Vec3d{1,1,3},
    };
    GMK_EXPECT(s.init(1, 1, ku, 4, kv, 4, cp, nullptr, 2, 2) == Status::Ok);
    Vec3d p, du, dv;
    s.point_and_partials(0.3, 0.7, p, du, dv);
    Vec3d a, b;
    double h = 1e-6;
    s.point(0.3 + h, 0.7, a); s.point(0.3 - h, 0.7, b);
    GMK_EXPECT_NEAR(du.x, (a.x - b.x)/(2*h), 1e-4);
    s.point(0.3, 0.7 + h, a); s.point(0.3, 0.7 - h, b);
    GMK_EXPECT_NEAR(dv.x, (a.x - b.x)/(2*h), 1e-4);
}

GMK_TEST("nurbs: sphere normal points outward") {
    SphereSurface sp;
    GMK_EXPECT(sp.init(Vec3d{1,2,3}, Vec3d{0,0,1}, Vec3d{1,0,0}, 2.0) == Status::Ok);
    Vec3d n;
    GMK_EXPECT(sp.normal(0.0, 0.0, n) == Status::Ok);
    GMK_EXPECT_NEAR(std::sqrt(n.x*n.x + n.y*n.y + n.z*n.z), 1.0, 1e-9);
}

GMK_TEST("nurbs: knot insertion preserves shape") {
    NurbsCurve c;
    double knots[] = {0, 0, 0, 1, 2, 3, 3, 3};
    Vec3d cp[]    = { Vec3d{0,0,0}, Vec3d{1,1,0}, Vec3d{2,-1,0},
                       Vec3d{3,2,0}, Vec3d{4,0,0} };
    c.init(2, knots, 8, cp, nullptr, 5);
    Vec3d p_before, p_after;
    double u = 1.7;
    c.point(u, p_before);
    GMK_EXPECT(c.insert_knot(1.5, 1) == Status::Ok);
    c.point(u, p_after);
    GMK_EXPECT_NEAR(p_before.x, p_after.x, 1e-9);
    GMK_EXPECT_NEAR(p_before.y, p_after.y, 1e-9);
}
