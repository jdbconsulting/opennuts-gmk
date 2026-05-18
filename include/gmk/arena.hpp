#pragma once
//
// gmk::Arena -- a bump allocator. The kernel uses this in routines that
// would otherwise call ``new`` repeatedly (e.g. tessellation working sets,
// NURBS evaluation tables) so that hot paths stay within a single up-front
// allocation and overall memory pressure stays predictable.
//
// Arenas are intentionally non-owning of object lifetimes; everything
// allocated from an arena is reclaimed when the arena is reset. Callers
// must not place objects with non-trivial destructors into an arena --
// in practice every allocation is a POD work-buffer.
//

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <new>
#include <type_traits>

namespace gmk {

class Arena {
public:
    explicit Arena(std::size_t capacity_bytes)
        : capacity_{capacity_bytes},
          base_{std::make_unique<unsigned char[]>(capacity_bytes)} {}

    Arena(const Arena&)            = delete;
    Arena& operator=(const Arena&) = delete;
    Arena(Arena&&)                 = default;
    Arena& operator=(Arena&&)      = default;

    void reset() noexcept { used_ = 0; }

    std::size_t used()       const noexcept { return used_; }
    std::size_t capacity()   const noexcept { return capacity_; }
    std::size_t remaining()  const noexcept { return capacity_ - used_; }

    // Snapshot the cursor so a later restore() can rewind transient work.
    std::size_t save()                    const noexcept { return used_; }
    void        restore(std::size_t m)          noexcept { used_ = m; }

    // Allocates ``n`` bytes aligned to ``align``. Returns nullptr if the
    // arena does not have room.
    void* allocate(std::size_t n,
                   std::size_t align = alignof(std::max_align_t)) noexcept {
        std::uintptr_t base = reinterpret_cast<std::uintptr_t>(base_.get()) + used_;
        std::uintptr_t aligned = (base + align - 1) & ~(align - 1);
        std::size_t   pad = static_cast<std::size_t>(aligned - base);
        if (used_ + pad + n > capacity_) return nullptr;
        used_ += pad + n;
        return reinterpret_cast<void*>(aligned);
    }

    template <typename T>
    T* alloc_array(std::size_t count) noexcept {
        static_assert(std::is_trivially_destructible_v<T>,
                      "Arena objects must be trivially destructible.");
        void* p = allocate(sizeof(T) * count, alignof(T));
        if (!p) return nullptr;
        std::memset(p, 0, sizeof(T) * count);
        return std::launder(static_cast<T*>(p));
    }

private:
    std::size_t capacity_{0};
    std::size_t used_{0};
    std::unique_ptr<unsigned char[]> base_;
};

// Scope guard that rewinds an arena to its cursor on destruction. Useful
// for transient working memory inside a function.
class ArenaScope {
public:
    explicit ArenaScope(Arena& a) noexcept : a_{&a}, mark_{a.save()} {}
    ArenaScope(const ArenaScope&)            = delete;
    ArenaScope& operator=(const ArenaScope&) = delete;
    ~ArenaScope() noexcept { if (a_) a_->restore(mark_); }
    void dismiss() noexcept { a_ = nullptr; }
private:
    Arena*      a_;
    std::size_t mark_;
};

}  // namespace gmk
