/* Copyright 2026 The Openbot Authors. */
#pragma once
#include "autonomy/planning/planner/vista/obstacle_contour.hpp"
#include "autonomy/planning/planner/vista/spatial_index.hpp"

namespace autonomy::planning::planner::vista {
class GraphUpdater {
public:
    explicit GraphUpdater(std::size_t maximum_nodes = 4096, double maximum_edge_length = 30.0);
    GraphUpdateStats Update(const perception::MapReadView& map,
                            const EdgeValidator& validator, VisibilityGraph& graph);
private:
    std::size_t maximum_nodes_;
    double maximum_edge_length_;
    NodeId next_id_ = 1;
    EdgeSpatialIndex index_;
};
}  // namespace autonomy::planning::planner::vista
