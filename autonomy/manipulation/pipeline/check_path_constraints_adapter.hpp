/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file check_path_constraints_adapter.hpp
 * @brief Post-plan path constraints check adapter.
 */

#pragma once

#include "autonomy/manipulation/pipeline/planning_request_adapter.hpp"

namespace autonomy {
namespace manipulation {
namespace planner {

/** @brief Post-plan: verify joint path constraints along the trajectory. */
class CheckPathConstraintsAdapter : public PlanningRequestAdapter {
 public:
  std::string GetName() const override { return "check_constraints"; }
  bool Adapt(MotionPlanRequest* request,
             ::autonomy::manipulation::proto::MotionPlanResponse* response) const override;
};

}  // namespace planner
}  // namespace manipulation
}  // namespace autonomy
