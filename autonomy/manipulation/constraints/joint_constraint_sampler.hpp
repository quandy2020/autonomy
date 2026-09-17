/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <algorithm>
#include <string>

#include "autonomy/manipulation/constraints/constraint_sampler.hpp"
#include "autonomy/manipulation/constraints/constraint_samplers.hpp"
#include "autonomy/manipulation/common/planner_interface.hpp"

namespace autonomy {
namespace manipulation {
namespace constraints {

/** @brief Joint-limit / joint-constraint region sampler. */
class JointConstraintSampler : public ConstraintSampler {
 public:
  JointConstraintSampler(std::string group, planner::MotionPlanRequest req)
      : group_(std::move(group)), req_(std::move(req)) {}

  const std::string& GetName() const override {
    static const std::string kName = "JointConstraintSampler";
    return kName;
  }
  const std::string& GetGroupName() const override { return group_; }

  bool Sample(const automsgs::msgs::sensor_msgs::JointState& seed,
              automsgs::msgs::sensor_msgs::JointState* state, std::mt19937* rng,
              int max_attempts = 32) const override {
    if (!state || !rng) {
      return false;
    }
    for (int a = 0; a < std::max(1, max_attempts); ++a) {
      *state = seed;
      if (SampleJointConstrainedState(req_, state, rng, /*near_seed=*/a > 0)) {
        return true;
      }
    }
    return false;
  }

  bool Project(automsgs::msgs::sensor_msgs::JointState* state) const override {
    if (!state) {
      return false;
    }
    if (SatisfiesJointConstraints(req_, *state)) {
      return true;
    }
    return ClampToJointConstraints(req_, state);
  }

 private:
  std::string group_;
  planner::MotionPlanRequest req_;
};

}  // namespace constraints
}  // namespace manipulation
}  // namespace autonomy
