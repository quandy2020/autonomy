/*
 * Copyright 2026 The Openbot Authors
 *
 * Linear joint-space interpolation planner (no OMPL dependency).
 */

#pragma once

#include "autonomy/manipulation/planning/planner_base.hpp"

namespace autonomy {
namespace manipulation {
namespace planning {

/**
 * @brief Linear joint-space interpolation from start to goal.
 *
 * Suitable for short, collision-free motions; does not search around obstacles.
 */
class JointInterpolationPlanner : public PlannerBase {
 public:
  /**
   * @brief Store the planner id.
   * @param[in] planner_id Registry name.
   * @return true.
   */
  bool Init(const std::string& planner_id) override;

  /**
   * @brief Interpolate start_state → goal_state in joint space.
   * @param[in] request Must provide matching DOF start / goal.
   * @return Densified trajectory or error.
   */
  MotionPlanResponse Plan(const MotionPlanRequest& request) override;

  /**
   * @brief Set number of interpolation waypoints (clamped to ≥ 2).
   * @param[in] steps Desired waypoint count.
   */
  void SetNumSteps(int steps) { num_steps_ = steps > 1 ? steps : 2; }

 private:
  std::string planner_id_;
  int num_steps_ = 20;
};

}  // namespace planning
}  // namespace manipulation
}  // namespace autonomy
