/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file fix_start_state_path_constraints_adapter.hpp
 * @brief Pre-plan start-state path constraints adapter.
 */

#pragma once

#include "autonomy/manipulation/pipeline/planning_request_adapter.hpp"

namespace autonomy {
namespace manipulation {
namespace planner {

/**
 * @brief Pre-plan: project start_state onto joint + Cartesian path constraints.
 *
 * MoveIt FixStartStatePathConstraints lite: clamp joints then IK-project tip.
 */
class FixStartStatePathConstraintsAdapter : public PlanningRequestAdapter {
 public:
  std::string GetName() const override {
    return "fix_start_state_path_constraints";
  }
  bool Adapt(MotionPlanRequest* request,
             ::autonomy::manipulation::proto::MotionPlanResponse* response) const override;
};

}  // namespace planner
}  // namespace manipulation
}  // namespace autonomy
