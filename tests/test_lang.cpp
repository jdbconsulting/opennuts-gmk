#include "test_main.hpp"

#include <string>
#include <variant>
#include <vector>

#include "gmk/brep/body.hpp"
#include "gmk/lang/eval.hpp"
#include "gmk/lang/parser.hpp"

using namespace gmk;

GMK_TEST("lang: empty source parses") {
    lang::Parser p("");
    auto prog = p.parse();
    GMK_EXPECT(prog.items.empty());
    GMK_EXPECT(p.diagnostics().empty());
}

GMK_TEST("lang: parse unit + body + primitives") {
    const char* src =
        "unit mm;\n"
        "body widget {\n"
        "    box(width: 10, depth: 20, height: 5);\n"
        "    sphere(radius: 2) at (5, 10, 5);\n"
        "}\n";
    lang::Parser p(src);
    auto prog = p.parse();
    GMK_EXPECT_EQ(prog.items.size(), 2u);
    GMK_EXPECT(p.diagnostics().empty());

    // First item: unit directive (mm).
    auto* u = std::get_if<lang::ast::UnitDirective>(&prog.items[0]);
    GMK_EXPECT(u != nullptr);
    GMK_EXPECT(u && u->unit == lang::ast::UnitKind::Mm);

    auto* b = std::get_if<lang::ast::BodyDef>(&prog.items[1]);
    GMK_EXPECT(b != nullptr);
    GMK_EXPECT(b && b->name == "widget");
    GMK_EXPECT(b && b->statements.size() == 2u);
}

GMK_TEST("lang: missing semicolon produces a diagnostic") {
    const char* src = "unit mm\n";
    lang::Parser p(src);
    p.parse();
    bool found = false;
    for (const auto& d : p.diagnostics()) {
        if (d.message.find("';'") != std::string::npos) found = true;
    }
    GMK_EXPECT(found);
}

GMK_TEST("lang: line comments are skipped") {
    const char* src =
        "// hello\n"
        "unit m; // top-level\n";
    lang::Parser p(src);
    auto prog = p.parse();
    GMK_EXPECT_EQ(prog.items.size(), 1u);
}

GMK_TEST("lang: unknown unit triggers diagnostic") {
    const char* src = "unit furlongs;";
    lang::Parser p(src);
    p.parse();
    bool found = false;
    for (const auto& d : p.diagnostics()) {
        if (d.message.find("unknown unit") != std::string::npos) found = true;
    }
    GMK_EXPECT(found);
}

namespace {
struct VecBodyStore final : public gmk::lang::BodyStore {
    std::vector<gmk::brep::Body> bodies;
    std::int64_t add(gmk::brep::Body b) override {
        bodies.push_back(std::move(b));
        return static_cast<std::int64_t>(bodies.size());  // 1-based, non-zero
    }
};
}

GMK_TEST("lang: eval builds bodies for primitives") {
    const char* src =
        "unit mm;\n"
        "body widget {\n"
        "    box(width: 10, depth: 20, height: 5);\n"
        "    sphere(radius: 2) at (5, 10, 5);\n"
        "}\n";
    VecBodyStore store;
    std::vector<lang::EvalBody>   bodies;
    std::vector<lang::Diagnostic> diags;
    GMK_EXPECT(lang::eval_program(src, store, bodies, diags) == Status::Ok);
    GMK_EXPECT(diags.empty());
    GMK_EXPECT_EQ(bodies.size(), 2u);
    GMK_EXPECT_EQ(store.bodies.size(), 2u);
    // First body is the box: 6 faces.
    GMK_EXPECT_EQ(store.bodies[0].face_count(), 6u);
}

GMK_TEST("lang: eval reports missing argument as diagnostic") {
    const char* src = "body w { box(width: 1, depth: 2); }";  // missing height
    VecBodyStore store;
    std::vector<lang::EvalBody>   bodies;
    std::vector<lang::Diagnostic> diags;
    lang::eval_program(src, store, bodies, diags);
    bool found = false;
    for (const auto& d : diags) {
        if (d.message.find("height") != std::string::npos) found = true;
    }
    GMK_EXPECT(found);
}
