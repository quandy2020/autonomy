/*
 * Copyright 2026 The Openbot Authors
 *
 * Polymorphic constraint sampler (MoveIt ConstraintSampler lite).
 */

#pragma once

#include <algorithm>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "autonomy/manipulation/planner/constraint_samplers/constraint_samplers.hpp"
#include "autonomy/manipulation/common/planner_interface.hpp"
#include "autonomy/manipulation/motion/scene/planning_scene.hpp"

namespace autonomy {
namespace manipulation {
namespace constraint_samplers {

/**
 * @brief Base sampler object (MoveIt ConstraintSampler).
 *
 * Industrial lite: samples / projects joint states for a planning group.
 */
class ConstraintSampler {
 public:
  virtual ~ConstraintSampler() = default;

  virtual const std::string& GetName() const = 0;
  virtual const std::string& GetGroupName() const = 0;

  virtual bool Sample(const core::JointState& seed, core::JointState* state,
                      std::mt19937* rng, int max_attempts = 32) const = 0;

  virtual bool Project(core::JointState* state) const = 0;
};

/** @brief Joint-limit / joint-constraint region sampler. */
class JointConstraintSampler : public ConstraintSampler {
 public:
  JointConstraintSampler(std::string group, planning::MotionPlanRequest req)
      : group_(std::move(group)), req_(std::move(req)) {}

  const std::string& GetName() const override {
    static const std::string kName = "JointConstraintSampler";
    return kName;
  }
  const std::string& GetGroupName() const override { return group_; }

  bool Sample(const core::JointState& seed, core::JointState* state,
              std::mt19937* rng, int max_attempts = 32) const override {
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

  bool Project(core::JointState* state) const override {
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
  planning::MotionPlanRequest req_;
};

/** @brief IK region sampler for Cartesian path/goal constraints. */
class IkConstraintSampler : public ConstraintSampler {
 public:
  IkConstraintSampler(std::string group, planning::MotionPlanRequest req)
      : group_(std::move(group)), req_(std::move(req)) {}

  const std::string& GetName() const override {
    static const std::string kName = "IkConstraintSampler";
    return kName;
  }
  const std::string& GetGroupName() const override { return group_; }

  bool Sample(const core::JointState& seed, core::JointState* state,
              std::mt19937* rng, int max_attempts = 32) const override {
    return SampleIkConstrainedState(req_, seed, state, rng, max_attempts);
  }

  bool Project(core::JointState* state) const override {
    return ProjectOntoCartesianConstraints(req_, state);
  }

 private:
  std::string group_;
  planning::MotionPlanRequest req_;
};

/**
 * @brief Union of samplers (MoveIt UnionConstraintSampler lite).
 *
 * Samples with the first sampler, then projects through the remaining ones.
 */
class UnionConstraintSampler : public ConstraintSampler {
 public:
  UnionConstraintSampler(
      std::string group,
      std::vector<std::shared_ptr<ConstraintSampler>> samplers)
      : group_(std::move(group)), samplers_(std::move(samplers)) {}

  const std::string& GetName() const override {
    static const std::string kName = "UnionConstraintSampler";
    return kName;
  }
  const std::string& GetGroupName() const override { return group_; }

  bool Sample(const core::JointState& seed, core::JointState* state,
              std::mt19937* rng, int max_attempts = 32) const override {
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

  bool Project(core::JointState* state) const override {
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
  std::vector<std::shared_ptr<ConstraintSampler>> samplers_;
};

}  // namespace constraint_samplers
}  // namespace manipulation
}  // namespace autonomy
