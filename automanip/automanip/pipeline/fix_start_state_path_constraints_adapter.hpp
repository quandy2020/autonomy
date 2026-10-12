/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file fix_start_state_path_constraints_adapter.hpp
 * @brief Pre-plan start-state path constraints adapter.
 */

#pragma once

#include "automanip/pipeline/planning_request_adapter.hpp"

namespace automanip {
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
             ::automanip::proto::MotionPlanResponse* response) const override;
};

}  // namespace planner
}  // namespace automanip
