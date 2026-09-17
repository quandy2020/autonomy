/*
 * Copyright 2026 The Openbot Authors
 *
 * Cartesian linear path via stepwise IK.
 */

#pragma once

#include <string>

#include "autonomy/manipulation/common/planner_interface.hpp"

namespace autonomy {
namespace manipulation {
namespace planning {

/**
 * @brief Cartesian straight-line path via stepwise IK.
 *
 * Requires a kinematics solver on the request; fails if any IK sample fails.
 */
class CartesianPlanner : public PlannerBase {
 public:
  /**
   * @brief Store the planner id.
   * @param[in] planner_id Registry name.
   * @return true.
   */
  bool Init(const std::string& planner_id) override;

  /**
   * @brief Plan a linear Cartesian path to goal_pose / waypoints.
   * @param[in] request Must provide kinematics and a Cartesian goal.
   * @return Joint trajectory or IK / collision error.
   */
  MotionPlanResponse Plan(const MotionPlanRequest& request) override;

  /**
   * @brief Set number of Cartesian samples along the path.
   * @param[in] n Sample count.
   */
  void SetNumSteps(int n) { num_steps_ = n; }

 private:
  std::string planner_id_;
  int num_steps_ = 20;
};

}  // namespace planning
}  // namespace manipulation
}  // namespace autonomy
