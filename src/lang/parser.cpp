#include "gmk/lang/parser.hpp"

#include <utility>

namespace gmk::lang {

Parser::Parser(std::string_view src) : lex_{src}, tok_{} {
    take();
}

void Parser::take() {
    tok_ = lex_.next();
}

bool Parser::accept(TokKind k) {
    if (tok_.kind == k) { take(); return true; }
    return false;
}

bool Parser::expect(TokKind k, const char* what) {
    if (tok_.kind == k) { take(); return true; }
    std::string msg = "expected ";
    msg += what ? what : tok_name(k);
    msg += ", got ";
    msg += tok_name(tok_.kind);
    error(tok_.pos, std::move(msg), "P001");
    return false;
}

void Parser::error(SourcePos pos, std::string msg, std::string code) {
    diags_.push_back(Diagnostic{
        Diagnostic::Severity::Error,
        pos,
        1,
        std::move(msg),
        std::move(code),
    });
}

void Parser::sync_to(TokKind a, TokKind b) {
    while (tok_.kind != TokKind::Eof && tok_.kind != a && tok_.kind != b) {
        take();
    }
    if (tok_.kind == a || tok_.kind == b) take();
}

ast::Program Parser::parse() {
    ast::Program prog;
    while (tok_.kind != TokKind::Eof) {
        Token saved = tok_;
        prog.items.push_back(parse_top_level());
        // Forward-progress guarantee: if the parser failed to consume any
        // input, eat one token before retrying. Otherwise a syntax-error
        // on the very first token would infinite-loop.
        if (tok_.kind == saved.kind && tok_.pos.offset == saved.pos.offset) {
            take();
        }
    }
    return prog;
}

ast::TopLevel Parser::parse_top_level() {
    if (tok_.kind == TokKind::KwUnit)      return parse_unit();
    if (tok_.kind == TokKind::KwTolerance) return parse_tolerance();
    if (tok_.kind == TokKind::KwBody)      return parse_body();
    error(tok_.pos,
          std::string("unexpected top-level token ") + tok_name(tok_.kind),
          "P002");
    take();
    // Return an empty body so the variant always has a sensible value.
    return ast::BodyDef{};
}

ast::UnitDirective Parser::parse_unit() {
    ast::UnitDirective u;
    u.span.begin = tok_.pos;
    take();  // 'unit'
    if (tok_.kind != TokKind::Ident) {
        error(tok_.pos, "expected unit name (mm, cm, m, in, ft)", "U001");
    } else {
        const std::string& n = tok_.text;
        if      (n == "mm" || n == "millimeter") u.unit = ast::UnitKind::Mm;
        else if (n == "cm" || n == "centimeter") u.unit = ast::UnitKind::Cm;
        else if (n == "m"  || n == "meter")      u.unit = ast::UnitKind::M;
        else if (n == "in" || n == "inch")       u.unit = ast::UnitKind::Inch;
        else if (n == "ft" || n == "foot")       u.unit = ast::UnitKind::Foot;
        else error(tok_.pos, "unknown unit '" + n + "'", "U002");
        take();
    }
    u.span.end = tok_.pos;
    expect(TokKind::Semicolon, "';'");
    return u;
}

ast::ToleranceDirective Parser::parse_tolerance() {
    ast::ToleranceDirective t;
    t.span.begin = tok_.pos;
    take();  // 'tolerance'
    if (tok_.kind != TokKind::Number) {
        error(tok_.pos, "expected numeric tolerance value", "T001");
    } else {
        t.value_mm = tok_.number_value;
        take();
    }
    t.span.end = tok_.pos;
    expect(TokKind::Semicolon, "';'");
    return t;
}

ast::BodyDef Parser::parse_body() {
    ast::BodyDef b;
    b.span.begin = tok_.pos;
    take();  // 'body'
    if (tok_.kind != TokKind::Ident) {
        error(tok_.pos, "expected body name", "B001");
    } else {
        b.name = tok_.text;
        take();
    }
    if (!expect(TokKind::LBrace, "'{'")) {
        return b;
    }
    while (tok_.kind != TokKind::RBrace && tok_.kind != TokKind::Eof) {
        Token saved = tok_;
        b.statements.push_back(parse_body_statement());
        if (tok_.kind == saved.kind && tok_.pos.offset == saved.pos.offset) {
            take();
        }
    }
    b.span.end = tok_.pos;
    expect(TokKind::RBrace, "'}'");
    return b;
}

ast::BodyStatement Parser::parse_body_statement() {
    switch (tok_.kind) {
        case TokKind::KwBox:       return parse_primitive(ast::Primitive::Kind::Box);
        case TokKind::KwSphere:    return parse_primitive(ast::Primitive::Kind::Sphere);
        case TokKind::KwCylinder:  return parse_primitive(ast::Primitive::Kind::Cylinder);
        case TokKind::KwCone:      return parse_primitive(ast::Primitive::Kind::Cone);
        case TokKind::KwTorus:     return parse_primitive(ast::Primitive::Kind::Torus);
        case TokKind::KwUnion:     return parse_boolean(ast::BooleanOp::Kind::Union);
        case TokKind::KwIntersect: return parse_boolean(ast::BooleanOp::Kind::Intersect);
        case TokKind::KwSubtract:  return parse_boolean(ast::BooleanOp::Kind::Subtract);
        default:
            error(tok_.pos,
                  std::string("expected primitive or boolean op, got ") + tok_name(tok_.kind),
                  "S001");
            sync_to(TokKind::Semicolon, TokKind::RBrace);
            return ast::Primitive{};
    }
}

ast::Primitive Parser::parse_primitive(ast::Primitive::Kind k) {
    ast::Primitive prim;
    prim.span.begin = tok_.pos;
    prim.kind = k;
    take();  // primitive keyword
    if (!expect(TokKind::LParen, "'('")) {
        sync_to(TokKind::Semicolon, TokKind::RBrace);
        return prim;
    }
    parse_arg_list(prim.args);
    expect(TokKind::RParen, "')'");

    // Optional placement clauses: at (...), axis (...).
    while (tok_.kind == TokKind::KwAt || tok_.kind == TokKind::KwAxis) {
        bool is_at = tok_.kind == TokKind::KwAt;
        take();
        ast::Vec3Literal v;
        v.span.begin = tok_.pos;
        if (!expect(TokKind::LParen, "'('")) break;
        if (!parse_vec3(v)) break;
        expect(TokKind::RParen, "')'");
        v.span.end = tok_.pos;
        if (is_at) { prim.at_pos = v; prim.has_at = true; }
        else       { prim.axis_vec = v; prim.has_axis = true; }
    }
    prim.span.end = tok_.pos;
    expect(TokKind::Semicolon, "';'");
    return prim;
}

bool Parser::parse_arg_list(std::vector<ast::Arg>& out) {
    if (tok_.kind == TokKind::RParen) return true;  // empty
    for (;;) {
        ast::Arg a;
        a.span.begin = tok_.pos;
        // Optional ``name:`` prefix.
        if (tok_.kind == TokKind::Ident) {
            // Peek ahead by reading the next token; we don't have a true
            // two-token lookahead, so we use the lexer's peek.
            Token saved_ident = tok_;
            take();
            if (tok_.kind == TokKind::Colon) {
                a.name = saved_ident.text;
                take();
            } else {
                // Rewinding the lexer cleanly is not supported, so we
                // record the ident itself as a value-less arg with a
                // diagnostic. In practice OpenNuts is "named-args first"
                // so unkeyed identifiers are an error.
                error(saved_ident.pos,
                      "unexpected identifier '" + saved_ident.text + "' in argument list", "A001");
            }
        }
        if (tok_.kind != TokKind::Number) {
            error(tok_.pos, "expected numeric value", "A002");
            sync_to(TokKind::Comma, TokKind::RParen);
            return false;
        }
        a.value         = tok_.number_value;
        a.is_int        = tok_.number_is_int;
        a.integer_value = tok_.integer_value;
        a.span.end      = tok_.pos;
        take();
        out.push_back(std::move(a));
        if (tok_.kind == TokKind::Comma) { take(); continue; }
        break;
    }
    return true;
}

bool Parser::parse_vec3(ast::Vec3Literal& v) {
    if (tok_.kind != TokKind::Number) {
        error(tok_.pos, "expected number in vector literal", "V001");
        return false;
    }
    v.x = tok_.number_value; take();
    if (!expect(TokKind::Comma, "','")) return false;
    if (tok_.kind != TokKind::Number) {
        error(tok_.pos, "expected number in vector literal", "V001");
        return false;
    }
    v.y = tok_.number_value; take();
    if (!expect(TokKind::Comma, "','")) return false;
    if (tok_.kind != TokKind::Number) {
        error(tok_.pos, "expected number in vector literal", "V001");
        return false;
    }
    v.z = tok_.number_value; take();
    return true;
}

ast::BooleanOp Parser::parse_boolean(ast::BooleanOp::Kind k) {
    ast::BooleanOp op;
    op.span.begin = tok_.pos;
    op.kind = k;
    take();
    if (!expect(TokKind::LParen, "'('")) {
        sync_to(TokKind::Semicolon, TokKind::RBrace);
        return op;
    }
    while (tok_.kind != TokKind::RParen && tok_.kind != TokKind::Eof) {
        if (tok_.kind != TokKind::Ident) {
            error(tok_.pos, "expected body identifier", "BO1");
            break;
        }
        op.operands.push_back(tok_.text);
        take();
        if (tok_.kind == TokKind::Comma) { take(); continue; }
        break;
    }
    expect(TokKind::RParen, "')'");
    op.span.end = tok_.pos;
    expect(TokKind::Semicolon, "';'");
    return op;
}

}  // namespace gmk::lang
