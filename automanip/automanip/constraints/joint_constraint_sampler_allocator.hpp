/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include "automanip/constraints/constraint_sampler_allocator.hpp"
#include "automanip/constraints/joint_constraint_sampler.hpp"

namespace automanip {
namespace constraints {

/** @brief Serves joint-only constraints. */
class JointConstraintSamplerAllocator : public ConstraintSamplerAllocator {
 public:
  bool CanService(const scene::PlanningScene* /*scene*/,
                  const std::string& /*group*/,
                  const planner::MotionPlanRequest& req) const override {
    return !(req.pb.joint_constraints_size() == 0) &&
           (req.pb.position_constraints_size() == 0) &&
           (req.pb.orientation_constraints_size() == 0);
  }

  ConstraintSampler::SharedPtr Alloc(
      const scene::PlanningScene* /*scene*/, const std::string& group,
      const planner::MotionPlanRequest& req) const override {
    return std::make_shared<JointConstraintSampler>(group, req);
  }
};

}  // namespace constraints
}  // namespace automanip
