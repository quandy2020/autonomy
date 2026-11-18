/* Copyright 2026 The Openbot Authors. */
/** @file spatial_index.hpp
 * @brief Bounded world-space edge lookup with checked bucket coordinates.
 */
#pragma once
#include <cstdint>
#include <map>
#include <set>
#include "autonomy/planning/planner/vista/visibility_graph.hpp"

namespace autonomy::planning::planner::vista {
/** @brief Exact closed-box intersections accelerated by a sparse bucket registry. */
class EdgeSpatialIndex {
public:
    /** @brief Configure a positive finite bucket width in meters. */
    explicit EdgeSpatialIndex(double bucket_size = 2.0);
    /** @brief Insert or replace an edge; rejected input preserves its previous entry.
     * @throws std::invalid_argument Bounds are nonfinite or inverted.
     * @throws std::length_error Coordinates or bucket count exceed the index budget.
     */
    void Insert(const VisibilityEdge& edge);
    /** @brief Remove an existing edge and all its bucket memberships. */
    void Erase(const EdgeId& edge);
    /** @brief Return exact intersections; wide regions scan stored boxes in bounded time. */
    std::set<EdgeId> Query(const Bounds2d& region) const;
    /** @brief Release all edge bounds and bucket memberships. */
    void Clear();
private:
    double bucket_size_;  ///< Positive finite world-space width of one bucket.
    std::map<std::pair<std::int64_t,std::int64_t>, std::set<EdgeId>> buckets_;  ///< Sparse memberships.
    std::map<EdgeId, Bounds2d> bounds_;  ///< Validated exact boxes for final intersection filtering.
};
}  // namespace autonomy::planning::planner::vista
