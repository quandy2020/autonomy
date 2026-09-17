/*
 * Copyright 2026 The Openbot Authors
 *
 * OMPL RRT-Connect planner (compiled only with AUTONOMY_HAS_OMPL).
 */

#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "autonomy/manipulation/constraints/constraint_sampler_manager.hpp"
#include "autonomy/manipulation/planner/ompl/ompl_planning_config.hpp"
#include "autonomy/manipulation/common/planner_interface.hpp"

namespace autonomy {
namespace manipulation {
namespace planner {

/**
 * @brief OMPL-backed joint-space planner (enabled with AUTONOMY_HAS_OMPL).
 *
 * Selects RRTConnect / RRT / RRTstar / KPIECE / PRM from @p planner_id.
 * Loads `ompl_planning.conf` on Init for named config overrides.
 */
class OmplPlanner : public common::PlannerInterface {
 public:
  bool Init(const std::string& planner_id) override;
  ::autonomy::manipulation::proto::MotionPlanResponse Plan(const MotionPlanRequest& request) override;

 private:
  std::string planner_id_;
  std::vector<OmplPlannerConfig> configs_;
  std::unordered_map<std::string, OmplPlannerConfig> by_name_;
  constraints::ConstraintSamplerManager sampler_manager_;
};

/**
 * @brief Factory for OmplPlanner (or stub when OMPL is unavailable).
 * @return Shared planner instance.
 */
common::PlannerInterface::SharedPtr CreateOmplPlanner();

}  // namespace planner
}  // namespace manipulation
}  // namespace autonomy
