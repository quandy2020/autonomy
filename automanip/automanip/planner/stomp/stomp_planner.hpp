/*
 * Copyright 2026 The Openbot Authors
 *
 * STOMP planner plugin (stochastic trajectory optimize).
 */

#pragma once

#include <string>

#include "automanip/common/planner_interface.hpp"

namespace automanip {
namespace planner {

class StompPlanner : public common::PlannerInterface {
 public:
  bool Init(const std::string& planner_id) override;
  ::automanip::proto::MotionPlanResponse Plan(const MotionPlanRequest& request) override;

 private:
  std::string planner_id_;
};

}  // namespace planner
}  // namespace automanip
