/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/planning/common/planner_interface.hpp"

#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"

namespace autonomy {
namespace planning {
namespace common {

GlobalPlanner::GlobalPlanner(
    const proto::PlannerOptions& options, std::string name,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap)
    : options_(options),
      name_(std::move(name)),
      costmap_(std::move(costmap)) {}

}  // namespace common
}  // namespace planning
}  // namespace autonomy
