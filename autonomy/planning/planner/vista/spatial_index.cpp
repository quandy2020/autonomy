/* Copyright 2026 The Openbot Authors. */
/** @file spatial_index.cpp
 * @brief Checked quantization, transactional replacement and bounded spatial queries.
 */
#include "autonomy/planning/planner/vista/spatial_index.hpp"
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

namespace autonomy::planning::planner::vista {
namespace {
constexpr std::size_t kMaximumBuckets = 1000000;  ///< Maximum memberships for one indexed box.
/** @brief Valid closed rectangle expressed as a bounded count of signed buckets. */
struct BucketRange {
    std::int64_t x = 0;  ///< First X bucket.
    std::int64_t y = 0;  ///< First Y bucket.
    std::size_t width = 0;  ///< Number of X buckets, including closed endpoints.
    std::size_t height = 0;  ///< Number of Y buckets, including closed endpoints.
    /** @brief Test membership without overflowing signed endpoint subtraction. */
    bool Contains(std::int64_t px, std::int64_t py) const {
        return px >= x && py >= y && px <= x + static_cast<std::int64_t>(width - 1) &&
               py <= y + static_cast<std::int64_t>(height - 1);
    }
};
/** @brief Reject invalid world bounds before any floating-to-integer conversion. */
void Validate(const Bounds2d& b) {
    if (!std::isfinite(b.min_x) || !std::isfinite(b.min_y) ||
        !std::isfinite(b.max_x) || !std::isfinite(b.max_y) ||
        b.min_x > b.max_x || b.min_y > b.max_y)
        throw std::invalid_argument("invalid edge spatial bounds");
}
/** @brief Return only ranges with representable coordinates and bounded bucket count. */
std::optional<BucketRange> Quantize(const Bounds2d& b, double size) {
    const long double x0 = std::floor(static_cast<long double>(b.min_x) / size);
    const long double y0 = std::floor(static_cast<long double>(b.min_y) / size);
    const long double x1 = std::floor(static_cast<long double>(b.max_x) / size);
    const long double y1 = std::floor(static_cast<long double>(b.max_y) / size);
    // Half the int64 range remains exact even on platforms where long double
    // has double precision, and also leaves room for bounded loop offsets.
    constexpr auto limit = std::numeric_limits<std::int64_t>::max() / 2;
    for (long double q : {x0, y0, x1, y1})
        if (!std::isfinite(q) || q < -static_cast<long double>(limit) ||
            q >= static_cast<long double>(limit)) return std::nullopt;
    const long double width = x1 - x0 + 1, height = y1 - y0 + 1;
    if (width < 1 || height < 1 || width > kMaximumBuckets || height > kMaximumBuckets)
        return std::nullopt;
    const auto w = static_cast<std::size_t>(width), h = static_cast<std::size_t>(height);
    if (w > kMaximumBuckets / h) return std::nullopt;
    return BucketRange{static_cast<std::int64_t>(x0), static_cast<std::int64_t>(y0), w, h};
}
/** @brief Enumerate a prevalidated range without signed endpoint increments. */
template <typename Operation> void Visit(const BucketRange& range, Operation operation) {
    for (std::size_t dy = 0; dy < range.height; ++dy)
        for (std::size_t dx = 0; dx < range.width; ++dx)
            operation(std::make_pair(range.x + static_cast<std::int64_t>(dx),
                                     range.y + static_cast<std::int64_t>(dy)));
}
}  // namespace
EdgeSpatialIndex::EdgeSpatialIndex(double size) : bucket_size_(size) {
    if (!std::isfinite(size) || size <= 0) throw std::invalid_argument("invalid edge bucket size");
}
void EdgeSpatialIndex::Clear() { buckets_.clear(); bounds_.clear(); }
void EdgeSpatialIndex::Insert(const VisibilityEdge& edge) {
    Validate(edge.bounds);
    const auto range = Quantize(edge.bounds, bucket_size_);
    if (!range) throw std::length_error("edge spatial extent exceeds index limit");
    const auto old = bounds_.find(edge.id);
    const auto old_range = old == bounds_.end() ? std::optional<BucketRange>{} :
                                               Quantize(old->second, bucket_size_);
    auto remove_membership = [&](const auto& key) {
        auto bucket = buckets_.find(key);
        if (bucket == buckets_.end()) return;
        bucket->second.erase(edge.id);
        if (bucket->second.empty()) buckets_.erase(bucket);
    };
    try {
        // Add first. Both invalid input and allocation failures preserve the old
        // memberships; rollback needs neither a whole-index copy nor allocation.
        Visit(*range, [&](const auto& key) { buckets_[key].insert(edge.id); });
        if (old == bounds_.end()) bounds_.emplace(edge.id, edge.bounds);
        else old->second = edge.bounds;
    } catch (...) {
        Visit(*range, [&](const auto& key) {
            if (!old_range || !old_range->Contains(key.first, key.second)) remove_membership(key);
        });
        throw;
    }
    if (old_range) Visit(*old_range, [&](const auto& key) {
        if (!range->Contains(key.first, key.second)) remove_membership(key);
    });
}
void EdgeSpatialIndex::Erase(const EdgeId& edge) {
    auto b = bounds_.find(edge);
    if (b == bounds_.end()) return;
    Visit(*Quantize(b->second, bucket_size_), [&](const auto& key) {
        auto bucket = buckets_.find(key);
        if (bucket == buckets_.end()) return;
        bucket->second.erase(edge);
        if (bucket->second.empty()) buckets_.erase(bucket);
    });
    bounds_.erase(b);
}
std::set<EdgeId> EdgeSpatialIndex::Query(const Bounds2d& region) const {
    Validate(region);
    std::set<EdgeId> result;
    const auto range = Quantize(region, bucket_size_);
    if (!range || range->width * range->height > buckets_.size()) {
        for (const auto& entry : bounds_)
            if (entry.second.Intersects(region)) result.insert(entry.first);
        return result;
    }
    Visit(*range, [&](const auto& key) {
        auto bucket = buckets_.find(key);
        if (bucket == buckets_.end()) return;
        for (const auto& edge : bucket->second)
            if (bounds_.at(edge).Intersects(region)) result.insert(edge);
    });
    return result;
}
}  // namespace autonomy::planning::planner::vista
