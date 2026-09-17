/*
 * Copyright 2026 The Openbot Authors
 *
 * STOMP planner plugin (stochastic trajectory optimize).
 */

#pragma once

#include <string>

#include "autonomy/manipulation/common/planner_interface.hpp"

namespace autonomy {
namespace manipulation {
namespace planning {

class StompPlanner : public PlannerBase {
 public:
  bool Init(const std::string& planner_id) override;
  MotionPlanResponse Plan(const MotionPlanRequest& request) override;

 private:
  std::string planner_id_;
};

}  // namespace planning
}  // namespace manipulation
}  // namespace autonomy
