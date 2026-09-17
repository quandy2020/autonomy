/*
 * Copyright 2026 The Openbot Authors
 *
 * Simple RRT-Connect in joint space with collision checking.
 */

#pragma once

#include <string>

#include "autonomy/manipulation/common/planner_interface.hpp"

namespace autonomy {
namespace manipulation {
namespace planning {

/**
 * @brief Lightweight joint-space RRT-Connect with optional collision checks.
 *
 * Built-in sampler (no OMPL). Uses the request scene when available.
 */
class RrtConnectPlanner : public PlannerBase {
 public:
  /**
   * @brief Store the planner id.
   * @param[in] planner_id Registry name.
   * @return true.
   */
  bool Init(const std::string& planner_id) override;

  /**
   * @brief Search a collision-free joint path with RRT-Connect.
   * @param[in] request Start / goal joint states and optional scene.
   * @return Trajectory or planning failure.
   */
  MotionPlanResponse Plan(const MotionPlanRequest& request) override;

  /**
   * @brief Cap RRT expand / connect iterations.
   * @param[in] n Maximum iterations.
   */
  void SetMaxIterations(int n) { max_iters_ = n; }

 private:
  std::string planner_id_;
  int max_iters_ = 2000;
};

}  // namespace planning
}  // namespace manipulation
}  // namespace autonomy
