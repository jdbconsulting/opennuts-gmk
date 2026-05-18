#pragma once
//
// AST nodes for OpenNuts MCAD source. All position-aware; every node
// carries the source span where it appeared, so diagnostics can be
// attributed precisely.
//

#include <memory>
#include <string>
#include <variant>
#include <vector>

#include "gmk/lang/token.hpp"

namespace gmk::lang::ast {

struct Span {
    SourcePos begin{};
    SourcePos end{};
};

// A single (named or positional) argument inside a constructor call.
struct Arg {
    Span            span{};
    std::string     name;        // empty if positional
    double          value = 0.0; // currently only numeric args supported
    bool            is_int = true;
    std::int64_t    integer_value = 0;
};

struct Vec3Literal {
    Span   span{};
    double x = 0, y = 0, z = 0;
};

// A primitive constructor with optional placement clauses.
struct Primitive {
    enum class Kind {
        Box, Sphere, Cylinder, Cone, Torus,
    };
    Span                  span{};
    Kind                  kind;
    std::vector<Arg>      args;
    bool                  has_at = false;
    Vec3Literal           at_pos{};
    bool                  has_axis = false;
    Vec3Literal           axis_vec{};
};

struct BooleanOp {
    enum class Kind { Union, Intersect, Subtract };
    Span                  span{};
    Kind                  kind;
    std::vector<std::string> operands;  // identifiers of bodies in scope
};

using BodyStatement = std::variant<Primitive, BooleanOp>;

struct BodyDef {
    Span                       span{};
    std::string                name;
    std::vector<BodyStatement> statements;
};

enum class UnitKind { Mm, Cm, M, Inch, Foot };

struct UnitDirective {
    Span     span{};
    UnitKind unit{UnitKind::Mm};
};

struct ToleranceDirective {
    Span   span{};
    double value_mm = 0.001;
};

using TopLevel = std::variant<UnitDirective, ToleranceDirective, BodyDef>;

struct Program {
    std::vector<TopLevel> items;
};

}  // namespace gmk::lang::ast
