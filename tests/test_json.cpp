#include "test_main.hpp"

#include <string>

#include "gmk/server/json.hpp"

using namespace gmk;

GMK_TEST("json: parse atoms") {
    auto r = json::parse("null");        GMK_EXPECT(r && r->is_null());
    auto t = json::parse("true");        GMK_EXPECT(t && t->as_bool());
    auto f = json::parse("false");       GMK_EXPECT(f && !f->as_bool());
    auto i = json::parse("123");         GMK_EXPECT(i && i->as_int() == 123);
    auto d = json::parse("3.14");        GMK_EXPECT(d && std::abs(d->as_double() - 3.14) < 1e-12);
    auto s = json::parse("\"hi\\n\"");
    GMK_EXPECT(s && s->as_string() == "hi\n");
}

GMK_TEST("json: parse object") {
    auto v = json::parse("{\"x\":1,\"y\":[2,3,4]}");
    GMK_EXPECT(v && v->is_object());
    auto* x = v->find("x"); GMK_EXPECT(x && x->as_int() == 1);
    auto* y = v->find("y"); GMK_EXPECT(y && y->is_array() && y->as_array().size() == 3);
}

GMK_TEST("json: round trip preserves shape") {
    auto v = json::parse("{\"a\":1,\"b\":[2,3,\"x\"],\"c\":null}");
    std::string text = json::write(*v, /*compact=*/true);
    auto v2 = json::parse(text);
    GMK_EXPECT(v2 && v2->is_object());
    GMK_EXPECT(v2->find("a") && v2->find("a")->as_int() == 1);
    GMK_EXPECT(v2->find("c") && v2->find("c")->is_null());
}

GMK_TEST("json: malformed input is detected") {
    std::size_t off = 0;
    auto v = json::parse("{ \"a\": ", &off);
    GMK_EXPECT(!v);
}
