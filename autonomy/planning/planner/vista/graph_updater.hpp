/* Copyright 2026 The Openbot Authors. */
/** @file graph_updater.hpp
 * @brief Persistent contour vertex and visibility edge maintenance.
 */
#pragma once
#include "autonomy/planning/planner/vista/obstacle_contour.hpp"
#include "autonomy/planning/planner/vista/spatial_index.hpp"

namespace autonomy::planning::planner::vista {
/** @brief Update affected graph regions, preserving matching contour identities. */
class GraphUpdater {
public:
    /** @brief Bound contour vertex count and connection length in meters. */
    explicit GraphUpdater(std::size_t maximum_nodes = 4096, double maximum_edge_length = 30.0);
    /** @brief Apply a view; missing revisions or differences require a full refresh.
     * @param full_refresh Force all current and previous bounds to be reconsidered.
     * @details The owning planner serializes mutation and clears state on exceptions.
     */
    GraphUpdateStats Update(const perception::MapReadView& map,
                            const EdgeValidator& validator, VisibilityGraph& graph,
                            bool full_refresh = false);
    /** @brief Discard spatial state after a failed mutation, retaining monotonic node IDs. */
    void Clear();
private:
    std::size_t maximum_nodes_;  ///< Maximum retained contour vertices.
    double maximum_edge_length_;  ///< Maximum chord length, meters.
    NodeId next_id_ = 1;  ///< Monotonic IDs excluding reserved search endpoints.
    EdgeSpatialIndex index_;  ///< Spatial membership of valid and blocked edges.
    std::optional<Bounds2d> previous_extent_;  ///< Prior bounds needed to remove vanished vertices.
};
}  // namespace autonomy::planning::planner::vista
