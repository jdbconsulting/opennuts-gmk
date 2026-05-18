// Tiny test harness. We deliberately avoid pulling in gtest/catch -- the
// kernel has no dependencies and the tests stay small enough that a
// hand-rolled runner suffices.
//
// Each test file registers tests with the macro ``GMK_TEST(name)``. The
// runner walks the registry, prints PASS/FAIL, and exits with the number
// of failures.

#include "test_main.hpp"

#include <cstdio>
#include <cstring>

namespace gmk_test {

static TestRegistry* g_head = nullptr;

void register_test(TestRegistry* node) noexcept {
    node->next = g_head;
    g_head     = node;
}

}  // namespace gmk_test

int main(int argc, char** argv) {
    const char* filter = nullptr;
    for (int i = 1; i < argc; ++i) {
        if (std::strncmp(argv[i], "--filter=", 9) == 0) filter = argv[i] + 9;
    }
    int total = 0, failed = 0;
    for (auto* n = gmk_test::g_head; n; n = n->next) {
        if (filter && std::strstr(n->name, filter) == nullptr) continue;
        ++total;
        gmk_test::Context ctx{n->name};
        n->fn(ctx);
        if (ctx.failed) {
            ++failed;
            std::printf("[FAIL] %s\n", n->name);
            for (const auto& m : ctx.messages) std::printf("  %s\n", m.c_str());
        } else {
            std::printf("[ OK ] %s\n", n->name);
        }
    }
    std::printf("%d test(s), %d failure(s)\n", total, failed);
    return failed;
}
