/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file validate_trajectory_path_adapter.hpp
 * @brief Post-plan trajectory path validation adapter.
 */

#pragma once

#include "autonomy/manipulation/pipeline/planning_request_adapter.hpp"

namespace autonomy {
namespace manipulation {
namespace planner {

/** @brief Post-plan: reject trajectories that collide or violate scene validity. */
class ValidateTrajectoryPathAdapter : public PlanningRequestAdapter {
 public:
  std::string GetName() const override { return "validate_path"; }
  bool Adapt(MotionPlanRequest* request,
             ::autonomy::manipulation::proto::MotionPlanResponse* response) const override;
};

}  // namespace planner
}  // namespace manipulation
}  // namespace autonomy
