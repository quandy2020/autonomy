/* Copyright 2026 The Openbot Authors. */
#include "autonomy/planning/planner/vista/spatial_index.hpp"
#include <cmath>
#include <stdexcept>

namespace autonomy::planning::planner::vista {
EdgeSpatialIndex::EdgeSpatialIndex(double size) : bucket_size_(size) {
    if (!std::isfinite(size) || size <= 0) throw std::invalid_argument("invalid edge bucket size");
}
void EdgeSpatialIndex::Clear() { buckets_.clear(); bounds_.clear(); }
void EdgeSpatialIndex::Insert(const VisibilityEdge& edge) {
    Erase(edge.id); bounds_[edge.id] = edge.bounds;
    int x0 = static_cast<int>(std::floor(edge.bounds.min_x/bucket_size_));
    int x1 = static_cast<int>(std::floor(edge.bounds.max_x/bucket_size_));
    int y0 = static_cast<int>(std::floor(edge.bounds.min_y/bucket_size_));
    int y1 = static_cast<int>(std::floor(edge.bounds.max_y/bucket_size_));
    if (static_cast<long long>(x1-x0+1)*(y1-y0+1) > 1000000)
        throw std::length_error("edge spatial extent exceeds index limit");
    for (int y = y0; y <= y1; ++y) for (int x = x0; x <= x1; ++x)
        buckets_[{x,y}].insert(edge.id);
}
void EdgeSpatialIndex::Erase(const EdgeId& edge) {
    auto b = bounds_.find(edge); if (b == bounds_.end()) return;
    int x0 = static_cast<int>(std::floor(b->second.min_x/bucket_size_));
    int x1 = static_cast<int>(std::floor(b->second.max_x/bucket_size_));
    int y0 = static_cast<int>(std::floor(b->second.min_y/bucket_size_));
    int y1 = static_cast<int>(std::floor(b->second.max_y/bucket_size_));
    for (int y = y0; y <= y1; ++y) for (int x = x0; x <= x1; ++x) {
        auto bucket = buckets_.find({x,y}); if (bucket == buckets_.end()) continue;
        bucket->second.erase(edge); if (bucket->second.empty()) buckets_.erase(bucket);
    }
    bounds_.erase(b);
}
std::set<EdgeId> EdgeSpatialIndex::Query(const Bounds2d& region) const {
    std::set<EdgeId> result;
    int x0 = static_cast<int>(std::floor(region.min_x/bucket_size_));
    int x1 = static_cast<int>(std::floor(region.max_x/bucket_size_));
    int y0 = static_cast<int>(std::floor(region.min_y/bucket_size_));
    int y1 = static_cast<int>(std::floor(region.max_y/bucket_size_));
    for (int y = y0; y <= y1; ++y) for (int x = x0; x <= x1; ++x) {
        auto bucket = buckets_.find({x,y}); if (bucket == buckets_.end()) continue;
        for (const auto& edge : bucket->second)
            if (bounds_.at(edge).Intersects(region)) result.insert(edge);
    }
    return result;
}
}  // namespace autonomy::planning::planner::vista
