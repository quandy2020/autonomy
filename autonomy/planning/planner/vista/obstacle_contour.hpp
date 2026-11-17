/* Copyright 2026 The Openbot Authors. */
#pragma once
#include "autonomy/planning/planner/vista/edge_validator.hpp"

namespace autonomy::planning::planner::vista {
class ObstacleContour {
public:
    // Grid-resolution contour turning candidates. Raw obstacles are never inflated here.
    std::vector<Pose2d> Extract(const perception::MapReadView& map,
                               const std::vector<Bounds2d>& regions,
                               const EdgeValidator& validator) const;
};
}  // namespace autonomy::planning::planner::vista
