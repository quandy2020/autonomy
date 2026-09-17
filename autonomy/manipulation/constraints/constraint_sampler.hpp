/*
 * Copyright 2026 The Openbot Authors
 *
 * Polymorphic constraint sampler base (MoveIt ConstraintSampler lite).
 */

#pragma once

#include <memory>
#include <random>
#include <string>

#include "autonomy/common/macros.hpp"

#include <automsgs/msgs/sensor_msgs/joint_state.pb.h>

namespace autonomy {
namespace manipulation {
namespace constraints {

/**
 * @brief Base sampler object (MoveIt ConstraintSampler).
 *
 * Industrial lite: samples / projects joint states for a planning group.
 */
class ConstraintSampler {
 public:
  AUTONOMY_SMART_PTR_DEFINITIONS(ConstraintSampler)

  virtual ~ConstraintSampler() = default;

  virtual const std::string& GetName() const = 0;
  virtual const std::string& GetGroupName() const = 0;

  virtual bool Sample(const automsgs::msgs::sensor_msgs::JointState& seed,
                      automsgs::msgs::sensor_msgs::JointState* state,
                      std::mt19937* rng, int max_attempts = 32) const = 0;

  virtual bool Project(automsgs::msgs::sensor_msgs::JointState* state) const = 0;
};

}  // namespace constraints
}  // namespace manipulation
}  // namespace autonomy
