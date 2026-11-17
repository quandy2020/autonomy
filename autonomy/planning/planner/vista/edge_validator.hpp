/* Copyright 2026 The Openbot Authors. */
#pragma once
#include "autonomy/planning/planner/vista/visibility_graph.hpp"
#include "opt/navigation/safety.h"

namespace autonomy::planning::planner::vista {
class EdgeValidator {
public:
    EdgeValidator(const perception::MapReadView& map, opt::navigation::SafetyOptions options);
    bool PoseSafe(const Pose2d& pose) const;
    VisibilityEdge Validate(NodeId a, const Pose2d& from, NodeId b, const Pose2d& to) const;
    const opt::navigation::GeometrySafety& safety() const { return safety_; }
private:
    opt::navigation::GeometrySafety safety_;
    std::uint64_t revision_;
    double influence_padding_;
};
}  // namespace autonomy::planning::planner::vista
