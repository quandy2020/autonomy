/*
 * Copyright 2026 The Openbot Authors
 *
 * OMPL RRT-Connect planner (compiled only with AUTONOMY_HAS_OMPL).
 */

#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "autonomy/manipulation/planner/constraint_samplers/constraint_sampler_manager.hpp"
#include "autonomy/manipulation/planner/ompl/ompl_planning_config.hpp"
#include "autonomy/manipulation/common/planner_interface.hpp"

namespace autonomy {
namespace manipulation {
namespace planning {

/**
 * @brief OMPL-backed joint-space planner (enabled with AUTONOMY_HAS_OMPL).
 *
 * Selects RRTConnect / RRT / RRTstar / KPIECE / PRM from @p planner_id.
 * Loads `ompl_planning.conf` on Init for named config overrides.
 */
class OmplPlanner : public PlannerBase {
 public:
  bool Init(const std::string& planner_id) override;
  MotionPlanResponse Plan(const MotionPlanRequest& request) override;

 private:
  std::string planner_id_;
  std::vector<OmplPlannerConfig> configs_;
  std::unordered_map<std::string, OmplPlannerConfig> by_name_;
  constraint_samplers::ConstraintSamplerManager sampler_manager_;
};

/**
 * @brief Factory for OmplPlanner (or stub when OMPL is unavailable).
 * @return Shared planner instance.
 */
std::shared_ptr<PlannerBase> CreateOmplPlanner();

}  // namespace planning
}  // namespace manipulation
}  // namespace autonomy
