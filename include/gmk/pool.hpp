#pragma once
//
// gmk::Pool<T, Tag> -- a versioned, dense object pool. B-rep entities live
// inside pools keyed by ``Handle<Tag>``. The handle stores the slot index
// AND a 16-bit generation so that stale handles can be detected after a
// free() + alloc() recycle.
//
// State convention: a slot's generation is *odd* when the slot is live and
// *even* when it is free. The sentinel slot 0 has generation 0 (free) and
// represents the null handle.
//

#include <cstdint>
#include <type_traits>
#include <utility>
#include <vector>

#include "gmk/result.hpp"

namespace gmk {

template <typename Tag>
struct Handle {
    std::uint32_t index{0};
    std::uint16_t generation{0};
    std::uint16_t _pad{0};

    constexpr bool valid() const noexcept { return generation != 0; }
    constexpr bool operator==(const Handle& o) const noexcept {
        return index == o.index && generation == o.generation;
    }
    constexpr bool operator!=(const Handle& o) const noexcept {
        return !(*this == o);
    }

    static constexpr Handle null() noexcept { return Handle{0, 0, 0}; }
};

template <typename T, typename Tag>
class Pool {
public:
    using value_type  = T;
    using handle_type = Handle<Tag>;

    Pool() {
        data_.emplace_back();
        gen_.push_back(0);   // sentinel slot 0 = null
    }

    handle_type alloc() {
        std::uint32_t slot;
        if (free_head_ != 0) {
            slot = free_head_;
            free_head_ = next_free_[slot];
            data_[slot] = T{};
        } else {
            slot = static_cast<std::uint32_t>(data_.size());
            data_.emplace_back();
            gen_.push_back(0);
            next_free_.push_back(0);
        }
        // Move to next odd generation (live).
        std::uint16_t g = static_cast<std::uint16_t>(gen_[slot] + 1);
        if ((g & 1) == 0) g = static_cast<std::uint16_t>(g + 1);
        if (g == 0)       g = 1;        // never produce gen 0
        gen_[slot] = g;
        ++live_count_;
        return handle_type{slot, g, 0};
    }

    Status free(handle_type h) {
        if (!h.valid())                              return Status::InvalidArgument;
        if (h.index == 0 || h.index >= data_.size()) return Status::NotFound;
        if (gen_[h.index] != h.generation)           return Status::NotFound;
        data_[h.index] = T{};
        // Move to next even generation (free).
        std::uint16_t g = static_cast<std::uint16_t>(gen_[h.index] + 1);
        if ((g & 1) == 1) g = static_cast<std::uint16_t>(g + 1);
        gen_[h.index] = g;
        next_free_[h.index] = free_head_;
        free_head_ = h.index;
        --live_count_;
        return Status::Ok;
    }

    bool alive(handle_type h) const noexcept {
        return h.valid() && h.index != 0 && h.index < data_.size() &&
               gen_[h.index] == h.generation;
    }

    T* get(handle_type h) noexcept {
        return alive(h) ? &data_[h.index] : nullptr;
    }
    const T* get(handle_type h) const noexcept {
        return alive(h) ? &data_[h.index] : nullptr;
    }

    std::size_t   slot_count() const noexcept { return data_.size(); }
    std::uint16_t generation_at(std::size_t i) const noexcept { return gen_[i]; }
    bool is_live_slot(std::size_t i) const noexcept {
        return i != 0 && i < data_.size() && (gen_[i] & 1u);
    }
    T&       at_slot(std::size_t i)       noexcept { return data_[i]; }
    const T& at_slot(std::size_t i) const noexcept { return data_[i]; }
    handle_type handle_of_slot(std::size_t i) const noexcept {
        return handle_type{ static_cast<std::uint32_t>(i), gen_[i], 0 };
    }

    std::size_t size()  const noexcept { return live_count_; }
    bool        empty() const noexcept { return live_count_ == 0; }

    void clear() {
        data_.clear();
        gen_.clear();
        next_free_.clear();
        data_.emplace_back();
        gen_.push_back(0);
        free_head_  = 0;
        live_count_ = 0;
    }

    template <typename F>
    void for_each(F&& fn) {
        for (std::size_t i = 1; i < data_.size(); ++i) {
            if (gen_[i] & 1u) fn(handle_of_slot(i), data_[i]);
        }
    }
    template <typename F>
    void for_each(F&& fn) const {
        for (std::size_t i = 1; i < data_.size(); ++i) {
            if (gen_[i] & 1u) fn(handle_of_slot(i), data_[i]);
        }
    }

private:
    std::vector<T>             data_;
    std::vector<std::uint16_t> gen_;
    std::vector<std::uint32_t> next_free_;
    std::uint32_t              free_head_ = 0;
    std::size_t                live_count_ = 0;
};

}  // namespace gmk
