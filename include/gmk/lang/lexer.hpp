#pragma once
//
// gmk::lang::Lexer -- a simple, recoverable lexer for OpenNuts.
//
// The lexer is single-pass and never throws. Unknown characters produce a
// TokKind::Invalid token; the caller can treat that as a diagnostic and
// keep going.
//

#include <cstdint>
#include <string_view>

#include "gmk/lang/token.hpp"

namespace gmk::lang {

class Lexer {
public:
    explicit Lexer(std::string_view src) noexcept;

    // Read the next token. Skips whitespace and comments.
    Token next();

    // Peek without consuming.
    Token peek();

    std::string_view source() const noexcept { return src_; }

private:
    void   advance();
    void   skip_ws_and_comments();
    Token  ident_or_keyword();
    Token  number();
    Token  string_lit();
    Token  punctuation();

    std::string_view src_;
    std::uint32_t    line_   = 1;
    std::uint32_t    column_ = 1;
    std::uint32_t    pos_    = 0;
    bool             have_peek_ = false;
    Token            peeked_{};
};

}  // namespace gmk::lang
