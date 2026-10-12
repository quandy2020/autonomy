/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file check_path_constraints_adapter.hpp
 * @brief Post-plan path constraints check adapter.
 */

#pragma once

#include "automanip/pipeline/planning_request_adapter.hpp"

namespace automanip {
namespace planner {

/** @brief Post-plan: verify joint path constraints along the trajectory. */
class CheckPathConstraintsAdapter : public PlanningRequestAdapter {
 public:
  std::string GetName() const override { return "check_constraints"; }
  bool Adapt(MotionPlanRequest* request,
             ::automanip::proto::MotionPlanResponse* response) const override;
};

}  // namespace planner
}  // namespace automanip
