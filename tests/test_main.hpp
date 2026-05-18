#pragma once
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace gmk_test {

struct Context {
    const char*               name;
    std::vector<std::string>  messages;
    bool                      failed = false;

    void fail(const std::string& msg) {
        failed = true;
        messages.push_back(msg);
    }
};

using TestFn = void(*)(Context&);

struct TestRegistry {
    const char*    name;
    TestFn         fn;
    TestRegistry*  next;
};

void register_test(TestRegistry* node) noexcept;

}  // namespace gmk_test

#define GMK_TEST_CAT2(a, b) a##b
#define GMK_TEST_CAT(a, b)  GMK_TEST_CAT2(a, b)

#define GMK_TEST(NAME)                                                       \
    static void GMK_TEST_CAT(_gmk_test_, __LINE__)(gmk_test::Context&);      \
    static gmk_test::TestRegistry GMK_TEST_CAT(_gmk_test_reg_, __LINE__) = { \
        NAME, &GMK_TEST_CAT(_gmk_test_, __LINE__), nullptr};                 \
    static int GMK_TEST_CAT(_gmk_test_init_, __LINE__) = [](){               \
        gmk_test::register_test(&GMK_TEST_CAT(_gmk_test_reg_, __LINE__));    \
        return 0;                                                            \
    }();                                                                     \
    static void GMK_TEST_CAT(_gmk_test_, __LINE__)(gmk_test::Context& ctx)

#define GMK_EXPECT(cond)                                                     \
    do { if (!(cond)) ctx.fail(std::string(#cond) +                          \
        " (" + __FILE__ + ":" + std::to_string(__LINE__) + ")"); } while (0)

#define GMK_EXPECT_EQ(a, b)                                                  \
    do { if (!((a) == (b))) {                                                \
        ctx.fail(std::string(#a) + " != " + #b +                             \
                 " (" + __FILE__ + ":" + std::to_string(__LINE__) + ")");    \
    } } while (0)

#define GMK_EXPECT_NEAR(a, b, eps)                                           \
    do { double _a = (a), _b = (b), _e = static_cast<double>(eps);           \
         if (std::fabs(_a - _b) > _e) {                                      \
        char buf[160];                                                       \
        std::snprintf(buf, sizeof(buf), "%s ~ %s  |%.6g - %.6g|=%.3g > %.3g (%s:%d)", \
                      #a, #b, _a, _b, std::fabs(_a - _b), _e,                \
                      __FILE__, __LINE__);                                   \
        ctx.fail(buf);                                                       \
    } } while (0)
