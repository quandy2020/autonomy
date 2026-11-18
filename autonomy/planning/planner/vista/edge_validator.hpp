/* Copyright 2026 The Openbot Authors. */
/** @file edge_validator.hpp
 * @brief Snapshot-bound route geometry checks with one robot footprint inflation.
 */
#pragma once
#include "autonomy/planning/planner/vista/visibility_graph.hpp"
#include "opt/navigation/safety.h"

namespace autonomy::planning::planner::vista {
/** @brief Validate poses and chords against one immutable route-model snapshot. */
class EdgeValidator {
public:
    /** @brief Build geometry checks, compensating for already inflated source grids. */
    EdgeValidator(const perception::MapReadView& map, opt::navigation::SafetyOptions options);
    /** @brief Check pose footprint and available height evidence. */
    bool PoseSafe(const Pose2d& pose) const;
    /** @brief Certify both chord directions and record source revision and influence bounds. */
    VisibilityEdge Validate(NodeId a, const Pose2d& from, NodeId b, const Pose2d& to) const;
    /** @brief Expose the same geometry model to independent grid routing. */
    const opt::navigation::GeometrySafety& safety() const { return safety_; }
private:
    opt::navigation::GeometrySafety safety_;  ///< Snapshot geometry and effective footprint.
    std::uint64_t revision_;  ///< Source geometric validation revision.
    double influence_padding_;  ///< Dirty-region influence radius in meters.
};
}  // namespace autonomy::planning::planner::vista
