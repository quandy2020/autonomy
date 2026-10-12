/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file densify_joint_trajectory_adapter.hpp
 * @brief Post-plan joint trajectory densification adapter.
 */

#pragma once

#include "automanip/pipeline/planning_request_adapter.hpp"

namespace automanip {
namespace planner {

/**
 * @brief Post-plan: densify joint waypoints to a maximum joint-space step.
 */
class DensifyJointTrajectoryAdapter : public PlanningRequestAdapter {
 public:
  /**
   * @brief Construct with maximum joint-space step between waypoints.
   * @param[in] max_joint_step Max |Δq| between consecutive waypoints.
   */
  explicit DensifyJointTrajectoryAdapter(double max_joint_step = 0.1)
      : max_joint_step_(max_joint_step) {}
  std::string GetName() const override { return "dense_sample"; }
  bool Adapt(MotionPlanRequest* request,
             ::automanip::proto::MotionPlanResponse* response) const override;

 private:
  double max_joint_step_;
};

}  // namespace planner
}  // namespace automanip
