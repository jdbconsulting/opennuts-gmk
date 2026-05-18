#pragma once
//
// Axis-aligned bounding box in kernel integer units.
//

#include <algorithm>

#include "gmk/math/vec.hpp"
#include "gmk/units.hpp"

namespace gmk {

struct AABB {
    Vec3i min{ LENGTH_MAX,  LENGTH_MAX,  LENGTH_MAX};
    Vec3i max{LENGTH_MIN, LENGTH_MIN, LENGTH_MIN};

    constexpr bool empty() const noexcept {
        return min.x > max.x || min.y > max.y || min.z > max.z;
    }

    void include(Vec3i p) noexcept {
        if (p.x < min.x) min.x = p.x;
        if (p.y < min.y) min.y = p.y;
        if (p.z < min.z) min.z = p.z;
        if (p.x > max.x) max.x = p.x;
        if (p.y > max.y) max.y = p.y;
        if (p.z > max.z) max.z = p.z;
    }
    void include(const AABB& o) noexcept {
        include(o.min); include(o.max);
    }

    Vec3i extents() const noexcept {
        if (empty()) return Vec3i{};
        return Vec3i{ max.x - min.x, max.y - min.y, max.z - min.z };
    }
    Vec3i center() const noexcept {
        if (empty()) return Vec3i{};
        return Vec3i{ (min.x + max.x) / 2,
                      (min.y + max.y) / 2,
                      (min.z + max.z) / 2 };
    }
};

}  // namespace gmk
