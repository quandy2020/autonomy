/* Copyright 2026 The Openbot Authors. */
/** @file edge_validator.cpp
 * @brief Conservative route-model geometry and padded edge influence boxes.
 */
#include "autonomy/planning/planner/vista/edge_validator.hpp"

namespace autonomy::planning::planner::vista {
namespace {
/** @brief Avoid repeating footprint inflation already performed by the map producer. */
opt::navigation::SafetyOptions EffectiveOptions(const perception::MapReadView& map,
                                                 opt::navigation::SafetyOptions options) {
    if (map.footprint_inflated) { options.robot_radius = 1e-9; options.horizontal_margin = 0; }
  if (map.startup_support) options.startup_support = map.startup_support;
    return options;
}
}  // namespace
EdgeValidator::EdgeValidator(const perception::MapReadView& map,
                              opt::navigation::SafetyOptions options)
    : safety_(map.grid,EffectiveOptions(map,options)), revision_(map.geometry_revision),
      influence_padding_(options.robot_radius+options.horizontal_margin+map.grid->resolution()) {}
bool EdgeValidator::PoseSafe(const Pose2d& pose) const { return safety_.VerifyPose(pose).safe; }
VisibilityEdge EdgeValidator::Validate(NodeId a, const Pose2d& from, NodeId b, const Pose2d& to) const {
    auto checked = safety_.VerifySegment(from,to);
    if (checked.safe) {
        // Graph edges are undirected. At cell boundaries conservative interval
        // checks can differ with floating-point interpolation order, so admit
        // an edge only when the same checker certifies both traversal directions.
        const auto reverse = safety_.VerifySegment(to,from);
        checked.safe = reverse.safe;
        checked.clearance = std::min(checked.clearance,reverse.clearance);
    }
    return {EdgeKey(a,b),opt::navigation::Distance(from,to),checked.clearance,
            {std::min(from.x,to.x)-influence_padding_,std::min(from.y,to.y)-influence_padding_,
             std::max(from.x,to.x)+influence_padding_,std::max(from.y,to.y)+influence_padding_},
            checked.safe,revision_};
}
}  // namespace autonomy::planning::planner::vista
