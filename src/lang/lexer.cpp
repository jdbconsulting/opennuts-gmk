#include "gmk/lang/lexer.hpp"

#include <cstdlib>
#include <cstring>

namespace gmk::lang {

namespace {

bool is_alpha(char c) {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_';
}
bool is_digit(char c)       { return c >= '0' && c <= '9'; }
bool is_alnum(char c)       { return is_alpha(c) || is_digit(c); }

struct KW { std::string_view name; TokKind kind; };
const KW kKeywords[] = {
    {"unit",       TokKind::KwUnit},
    {"tolerance",  TokKind::KwTolerance},
    {"body",       TokKind::KwBody},
    {"at",         TokKind::KwAt},
    {"axis",       TokKind::KwAxis},
    {"true",       TokKind::KwTrue},
    {"false",      TokKind::KwFalse},
    {"box",        TokKind::KwBox},
    {"sphere",     TokKind::KwSphere},
    {"cylinder",   TokKind::KwCylinder},
    {"cone",       TokKind::KwCone},
    {"torus",      TokKind::KwTorus},
    {"union",      TokKind::KwUnion},
    {"intersect",  TokKind::KwIntersect},
    {"subtract",   TokKind::KwSubtract},
};

}  // namespace

Lexer::Lexer(std::string_view src) noexcept : src_{src} {}

void Lexer::advance() {
    if (pos_ < src_.size()) {
        char c = src_[pos_++];
        if (c == '\n') { ++line_; column_ = 1; }
        else           { ++column_; }
    }
}

void Lexer::skip_ws_and_comments() {
    for (;;) {
        if (pos_ >= src_.size()) return;
        char c = src_[pos_];
        if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
            advance();
            continue;
        }
        // // line comment
        if (c == '/' && pos_ + 1 < src_.size() && src_[pos_ + 1] == '/') {
            while (pos_ < src_.size() && src_[pos_] != '\n') advance();
            continue;
        }
        // /* block comment */
        if (c == '/' && pos_ + 1 < src_.size() && src_[pos_ + 1] == '*') {
            advance(); advance();
            while (pos_ + 1 < src_.size() &&
                   !(src_[pos_] == '*' && src_[pos_ + 1] == '/')) {
                advance();
            }
            if (pos_ + 1 < src_.size()) { advance(); advance(); }
            continue;
        }
        return;
    }
}

Token Lexer::ident_or_keyword() {
    SourcePos start{line_, column_, pos_};
    std::uint32_t begin = pos_;
    while (pos_ < src_.size() && is_alnum(src_[pos_])) advance();
    std::string_view text = src_.substr(begin, pos_ - begin);

    Token t;
    t.pos = start;
    t.length = pos_ - begin;
    t.text   = std::string(text);
    for (const auto& kw : kKeywords) {
        if (kw.name == text) {
            t.kind = kw.kind;
            return t;
        }
    }
    t.kind = TokKind::Ident;
    return t;
}

Token Lexer::number() {
    SourcePos start{line_, column_, pos_};
    std::uint32_t begin = pos_;
    bool is_float = false;
    if (src_[pos_] == '-') advance();
    while (pos_ < src_.size() && is_digit(src_[pos_])) advance();
    if (pos_ < src_.size() && src_[pos_] == '.') {
        is_float = true;
        advance();
        while (pos_ < src_.size() && is_digit(src_[pos_])) advance();
    }
    if (pos_ < src_.size() && (src_[pos_] == 'e' || src_[pos_] == 'E')) {
        is_float = true;
        advance();
        if (pos_ < src_.size() && (src_[pos_] == '+' || src_[pos_] == '-')) advance();
        while (pos_ < src_.size() && is_digit(src_[pos_])) advance();
    }
    Token t;
    t.kind   = TokKind::Number;
    t.pos    = start;
    t.length = pos_ - begin;
    std::string tok(src_.substr(begin, pos_ - begin));
    if (is_float) {
        t.number_is_int = false;
        t.number_value  = std::strtod(tok.c_str(), nullptr);
    } else {
        t.number_is_int = true;
        t.integer_value = static_cast<std::int64_t>(std::strtoll(tok.c_str(), nullptr, 10));
        t.number_value  = static_cast<double>(t.integer_value);
    }
    t.text = std::move(tok);
    return t;
}

Token Lexer::string_lit() {
    SourcePos start{line_, column_, pos_};
    std::uint32_t begin = pos_;
    advance();  // opening "
    std::string out;
    while (pos_ < src_.size() && src_[pos_] != '"') {
        if (src_[pos_] == '\\' && pos_ + 1 < src_.size()) {
            char esc = src_[pos_ + 1];
            char dec;
            switch (esc) {
                case '"': dec = '"';  break;
                case '\\': dec = '\\'; break;
                case 'n': dec = '\n'; break;
                case 't': dec = '\t'; break;
                case 'r': dec = '\r'; break;
                default:  dec = esc;
            }
            out.push_back(dec);
            advance(); advance();
            continue;
        }
        out.push_back(src_[pos_]);
        advance();
    }
    if (pos_ < src_.size()) advance();  // closing "
    Token t;
    t.kind   = TokKind::String;
    t.pos    = start;
    t.length = pos_ - begin;
    t.text   = std::move(out);
    return t;
}

Token Lexer::punctuation() {
    SourcePos start{line_, column_, pos_};
    char c = src_[pos_];
    Token t;
    t.pos    = start;
    t.length = 1;
    switch (c) {
        case '(': t.kind = TokKind::LParen;    break;
        case ')': t.kind = TokKind::RParen;    break;
        case '{': t.kind = TokKind::LBrace;    break;
        case '}': t.kind = TokKind::RBrace;    break;
        case '[': t.kind = TokKind::LBracket;  break;
        case ']': t.kind = TokKind::RBracket;  break;
        case ',': t.kind = TokKind::Comma;     break;
        case ';': t.kind = TokKind::Semicolon; break;
        case ':': t.kind = TokKind::Colon;     break;
        case '=': t.kind = TokKind::Equals;    break;
        default:
            t.kind = TokKind::Invalid;
            t.text.push_back(c);
            break;
    }
    advance();
    return t;
}

Token Lexer::next() {
    if (have_peek_) { have_peek_ = false; return std::move(peeked_); }
    skip_ws_and_comments();
    if (pos_ >= src_.size()) {
        Token t;
        t.kind = TokKind::Eof;
        t.pos  = SourcePos{line_, column_, pos_};
        return t;
    }
    char c = src_[pos_];
    if (is_alpha(c))   return ident_or_keyword();
    if (is_digit(c))   return number();
    if (c == '-' && pos_ + 1 < src_.size() && is_digit(src_[pos_ + 1])) return number();
    if (c == '"')      return string_lit();
    return punctuation();
}

Token Lexer::peek() {
    if (!have_peek_) {
        peeked_     = next();
        have_peek_  = true;
    }
    return peeked_;
}

}  // namespace gmk::lang
