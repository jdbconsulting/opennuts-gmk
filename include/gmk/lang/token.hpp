#pragma once
//
// Tokens for the OpenNuts MCAD language.
//
// OpenNuts is a small declarative language for describing parametric
// mechanical assemblies. A program is a sequence of top-level statements
// that either set a global directive (``unit``, ``tolerance``) or define
// a body (``body NAME { ... }``).  Inside a body, statements are
// primitive constructors (``box``, ``sphere``, ``cylinder``, ``cone``,
// ``torus``) optionally followed by placement clauses (``at (x,y,z)``,
// ``axis (i,j,k)``).
//
// A worked example::
//
//     unit mm;
//
//     body widget {
//         box(width: 50, depth: 30, height: 10);
//         sphere(radius: 5) at (25, 15, 10);
//         cylinder(radius: 2, height: 12) at (10, 10, 0) axis (0, 0, 1);
//     }
//
// This is a small but real superset of SchemLang's directive style,
// adapted to MCAD.
//

#include <cstdint>
#include <string>
#include <string_view>

namespace gmk::lang {

enum class TokKind : std::uint8_t {
    Eof,
    Newline,           // significant only for diagnostics
    Ident,
    Number,            // either int or float; payload determines
    String,
    LParen,            // (
    RParen,            // )
    LBrace,            // {
    RBrace,            // }
    LBracket,          // [
    RBracket,          // ]
    Comma,
    Semicolon,
    Colon,
    Equals,
    // Keywords.
    KwUnit,
    KwTolerance,
    KwBody,
    KwAt,
    KwAxis,
    KwTrue,
    KwFalse,
    // Primitive constructors are also keywords for syntactic clarity.
    KwBox,
    KwSphere,
    KwCylinder,
    KwCone,
    KwTorus,
    // Boolean operations.
    KwUnion,
    KwIntersect,
    KwSubtract,
    // Errors.
    Invalid,
};

struct SourcePos {
    std::uint32_t line   = 1;
    std::uint32_t column = 1;
    std::uint32_t offset = 0;
};

struct Token {
    TokKind        kind   = TokKind::Eof;
    SourcePos      pos{};
    std::uint32_t  length = 0;        // bytes consumed
    // Lexeme payload.
    std::string    text;              // raw text (only set for diagnostics + identifiers)
    double         number_value = 0.0; // valid iff kind == Number
    bool           number_is_int = true;
    std::int64_t   integer_value = 0;
};

constexpr const char* tok_name(TokKind k) noexcept {
    switch (k) {
        case TokKind::Eof:        return "<eof>";
        case TokKind::Newline:    return "<newline>";
        case TokKind::Ident:      return "identifier";
        case TokKind::Number:     return "number";
        case TokKind::String:     return "string";
        case TokKind::LParen:     return "'('";
        case TokKind::RParen:     return "')'";
        case TokKind::LBrace:     return "'{'";
        case TokKind::RBrace:     return "'}'";
        case TokKind::LBracket:   return "'['";
        case TokKind::RBracket:   return "']'";
        case TokKind::Comma:      return "','";
        case TokKind::Semicolon:  return "';'";
        case TokKind::Colon:      return "':'";
        case TokKind::Equals:     return "'='";
        case TokKind::KwUnit:     return "unit";
        case TokKind::KwTolerance:return "tolerance";
        case TokKind::KwBody:     return "body";
        case TokKind::KwAt:       return "at";
        case TokKind::KwAxis:     return "axis";
        case TokKind::KwTrue:     return "true";
        case TokKind::KwFalse:    return "false";
        case TokKind::KwBox:      return "box";
        case TokKind::KwSphere:   return "sphere";
        case TokKind::KwCylinder: return "cylinder";
        case TokKind::KwCone:     return "cone";
        case TokKind::KwTorus:    return "torus";
        case TokKind::KwUnion:    return "union";
        case TokKind::KwIntersect:return "intersect";
        case TokKind::KwSubtract: return "subtract";
        case TokKind::Invalid:    return "<invalid>";
    }
    return "<unknown>";
}

}  // namespace gmk::lang
