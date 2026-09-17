/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <string>
#include <vector>

#include "autonomy/manipulation/constraints/constraint_sampler.hpp"

namespace autonomy {
namespace manipulation {
namespace constraints {

/**
 * @brief Union of samplers (MoveIt UnionConstraintSampler lite).
 *
 * Samples with the first sampler, then projects through the remaining ones.
 */
class UnionConstraintSampler : public ConstraintSampler {
 public:
  UnionConstraintSampler(std::string group,
                         std::vector<ConstraintSampler::SharedPtr> samplers)
      : group_(std::move(group)), samplers_(std::move(samplers)) {}

  const std::string& GetName() const override {
    static const std::string kName = "UnionConstraintSampler";
    return kName;
  }
  const std::string& GetGroupName() const override { return group_; }

  bool Sample(const automsgs::msgs::sensor_msgs::JointState& seed,
              automsgs::msgs::sensor_msgs::JointState* state, std::mt19937* rng,
              int max_attempts = 32) const override {
    if (!state || !rng || samplers_.empty()) {
      return false;
    }
    for (int a = 0; a < max_attempts; ++a) {
      if (!samplers_.front()->Sample(seed, state, rng, 1)) {
        continue;
      }
      bool ok = true;
      for (std::size_t i = 1; i < samplers_.size(); ++i) {
        if (!samplers_[i]->Project(state)) {
          ok = false;
          break;
        }
      }
      if (ok) {
        return true;
      }
    }
    return false;
  }

  bool Project(automsgs::msgs::sensor_msgs::JointState* state) const override {
    if (!state) {
      return false;
    }
    for (const auto& s : samplers_) {
      if (s && !s->Project(state)) {
        return false;
      }
    }
    return true;
  }

 private:
  std::string group_;
  std::vector<ConstraintSampler::SharedPtr> samplers_;
};

}  // namespace constraints
}  // namespace manipulation
}  // namespace autonomy
