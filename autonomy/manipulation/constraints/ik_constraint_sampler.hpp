/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <string>

#include "autonomy/manipulation/constraints/constraint_sampler.hpp"
#include "autonomy/manipulation/constraints/constraint_samplers.hpp"
#include "autonomy/manipulation/common/planner_interface.hpp"

namespace autonomy {
namespace manipulation {
namespace constraints {

/** @brief IK region sampler for Cartesian path/goal constraints. */
class IkConstraintSampler : public ConstraintSampler {
 public:
  IkConstraintSampler(std::string group, planner::MotionPlanRequest req)
      : group_(std::move(group)), req_(std::move(req)) {}

  const std::string& GetName() const override {
    static const std::string kName = "IkConstraintSampler";
    return kName;
  }
  const std::string& GetGroupName() const override { return group_; }

  bool Sample(const automsgs::msgs::sensor_msgs::JointState& seed,
              automsgs::msgs::sensor_msgs::JointState* state, std::mt19937* rng,
              int max_attempts = 32) const override {
    return SampleIkConstrainedState(req_, seed, state, rng, max_attempts);
  }

  bool Project(automsgs::msgs::sensor_msgs::JointState* state) const override {
    return ProjectOntoCartesianConstraints(req_, state);
  }

 private:
  std::string group_;
  planner::MotionPlanRequest req_;
};

}  // namespace constraints
}  // namespace manipulation
}  // namespace autonomy
