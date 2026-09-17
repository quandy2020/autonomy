/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file fix_start_state_bounds_adapter.hpp
 * @brief Pre-plan start-state joint bounds adapter.
 */

#pragma once

#include "autonomy/manipulation/pipeline/planning_request_adapter.hpp"

namespace autonomy {
namespace manipulation {
namespace planner {

/** @brief Pre-plan: clamp start_state into model joint limits when model is set. */
class FixStartStateBoundsAdapter : public PlanningRequestAdapter {
 public:
  std::string GetName() const override { return "fix_start_state_bounds"; }
  bool Adapt(MotionPlanRequest* request,
             ::autonomy::manipulation::proto::MotionPlanResponse* response) const override;
};

}  // namespace planner
}  // namespace manipulation
}  // namespace autonomy
