/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file validate_trajectory_path_adapter.hpp
 * @brief Post-plan trajectory path validation adapter.
 */

#pragma once

#include "automanip/pipeline/planning_request_adapter.hpp"

namespace automanip {
namespace planner {

/** @brief Post-plan: reject trajectories that collide or violate scene validity. */
class ValidateTrajectoryPathAdapter : public PlanningRequestAdapter {
 public:
  std::string GetName() const override { return "validate_path"; }
  bool Adapt(MotionPlanRequest* request,
             ::automanip::proto::MotionPlanResponse* response) const override;
};

}  // namespace planner
}  // namespace automanip
