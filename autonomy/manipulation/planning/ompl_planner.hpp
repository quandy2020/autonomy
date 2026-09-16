/*
 * Copyright 2026 The Openbot Authors
 *
 * OMPL RRT-Connect planner (compiled only with AUTONOMY_HAS_OMPL).
 */

#pragma once

#include "autonomy/manipulation/planning/planner_base.hpp"

namespace autonomy {
namespace manipulation {
namespace planning {

/**
 * @brief OMPL-backed RRT-Connect planner (enabled with AUTONOMY_HAS_OMPL).
 *
 * Without OMPL, CreateOmplPlanner() returns a stub that reports failure.
 */
class OmplPlanner : public PlannerBase {
 public:
  /**
   * @brief Store the planner id.
   * @param[in] planner_id Registry name.
   * @return true.
   */
  bool Init(const std::string& planner_id) override;

  /**
   * @brief Plan with OMPL RRT-Connect in joint space.
   * @param[in] request Start / goal and optional collision scene.
   * @return Trajectory or planning failure.
   */
  MotionPlanResponse Plan(const MotionPlanRequest& request) override;

 private:
  std::string planner_id_;
};

/**
 * @brief Factory for OmplPlanner (or stub when OMPL is unavailable).
 * @return Shared planner instance.
 */
std::shared_ptr<PlannerBase> CreateOmplPlanner();

}  // namespace planning
}  // namespace manipulation
}  // namespace autonomy
