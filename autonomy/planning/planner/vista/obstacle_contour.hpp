/* Copyright 2026 The Openbot Authors. */
/** @file obstacle_contour.hpp
 * @brief Grid contour candidate extraction inside bounded world regions.
 */
#pragma once
#include "autonomy/planning/planner/vista/edge_validator.hpp"

namespace autonomy::planning::planner::vista {
/** @brief Generate safe corner and doorway poses without mutating source geometry. */
class ObstacleContour {
public:
    /** @brief Extract grid-resolution contour vertices from clipped update regions.
     * @details Raw obstacles are not inflated; EdgeValidator applies the footprint once.
     */
    std::vector<Pose2d> Extract(const perception::MapReadView& map,
                               const std::vector<Bounds2d>& regions,
                               const EdgeValidator& validator) const;
};
}  // namespace autonomy::planning::planner::vista
