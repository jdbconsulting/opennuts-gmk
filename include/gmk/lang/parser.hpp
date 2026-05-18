#pragma once
//
// Recursive-descent parser for OpenNuts MCAD.
//
// The parser is *recovering*: on a syntax error it records a diagnostic
// and resyncs to the next ``;`` or ``}`` rather than aborting. This is
// what the LSP wants -- partial ASTs from a broken file still drive
// hover/completion sensibly.
//

#include <string>
#include <string_view>
#include <vector>

#include "gmk/lang/ast.hpp"
#include "gmk/lang/lexer.hpp"
#include "gmk/lang/token.hpp"

namespace gmk::lang {

struct Diagnostic {
    enum class Severity : std::uint8_t { Hint, Info, Warning, Error };
    Severity     severity = Severity::Error;
    SourcePos    pos{};
    std::uint32_t length = 1;
    std::string  message;
    std::string  code;     // optional diagnostic code
};

class Parser {
public:
    explicit Parser(std::string_view source);

    // Run the full parse. Always succeeds; diagnostics live in
    // ``diagnostics()``.
    ast::Program parse();

    const std::vector<Diagnostic>& diagnostics() const noexcept { return diags_; }

private:
    bool   accept(TokKind k);
    bool   expect(TokKind k, const char* what);
    void   error(SourcePos pos, std::string msg, std::string code = "");
    void   sync_to(TokKind a, TokKind b);

    ast::TopLevel parse_top_level();
    ast::UnitDirective       parse_unit();
    ast::ToleranceDirective  parse_tolerance();
    ast::BodyDef             parse_body();
    ast::BodyStatement       parse_body_statement();
    ast::Primitive           parse_primitive(ast::Primitive::Kind k);
    ast::BooleanOp           parse_boolean(ast::BooleanOp::Kind k);
    bool                     parse_arg_list(std::vector<ast::Arg>& out);
    bool                     parse_vec3(ast::Vec3Literal& out);

    Lexer                   lex_;
    Token                   tok_;
    std::vector<Diagnostic> diags_;

    void take();
};

}  // namespace gmk::lang
