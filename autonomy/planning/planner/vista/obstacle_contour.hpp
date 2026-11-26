/* Copyright 2026 The Openbot Authors. */
/** @file obstacle_contour.hpp
 * @brief Grid contour candidate extraction inside bounded world regions.
 */
#pragma once
#include "autonomy/planning/planner/vista/edge_validator.hpp"

namespace autonomy::planning::planner::vista {
/** @brief One extracted contour vertex with FAR-compatible classification. */
struct ContourVertex {
    Pose2d pose;  ///< Candidate vertex in the world grid frame.
    ContourDirect direct = ContourDirect::kUnknown;  ///< Obstacle free direction.
    std::uint64_t contour_id = 0;  ///< Obstacle contour identity (minimum component cell).
    bool frontier = false;  ///< Vertex touches unmapped space.
    bool boundary = false;  ///< Vertex lies on the mapped-area boundary.
};
/** @brief Generate safe corner, doorway, pillar and frontier poses without mutating source geometry. */
class ObstacleContour {
public:
    /** @brief Extract grid-resolution contour vertices from clipped update regions.
     * @details Raw obstacles are not inflated; EdgeValidator applies the footprint once.
     */
    std::vector<ContourVertex> Extract(const perception::MapReadView& map,
                                      const std::vector<Bounds2d>& regions,
                                      const EdgeValidator& validator) const;
};
}  // namespace autonomy::planning::planner::vista
