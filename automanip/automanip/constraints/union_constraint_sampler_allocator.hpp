/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <vector>

#include "automanip/constraints/constraint_sampler_allocator.hpp"
#include "automanip/constraints/ik_constraint_sampler.hpp"
#include "automanip/constraints/joint_constraint_sampler.hpp"
#include "automanip/constraints/union_constraint_sampler.hpp"

namespace automanip {
namespace constraints {

/**
 * @brief Serves joint + Cartesian together via UnionConstraintSampler.
 *
 * Registered first so mixed constraints do not collapse to IK-only.
 */
class UnionConstraintSamplerAllocator : public ConstraintSamplerAllocator {
 public:
  bool CanService(const scene::PlanningScene* /*scene*/,
                  const std::string& /*group*/,
                  const planner::MotionPlanRequest& req) const override {
    const bool has_cart = req.kinematics &&
                          (!(req.pb.position_constraints_size() == 0) ||
                           !(req.pb.orientation_constraints_size() == 0));
    return has_cart && !(req.pb.joint_constraints_size() == 0);
  }

  ConstraintSampler::SharedPtr Alloc(
      const scene::PlanningScene* /*scene*/, const std::string& group,
      const planner::MotionPlanRequest& req) const override {
    std::vector<ConstraintSampler::SharedPtr> parts;
    parts.push_back(std::make_shared<IkConstraintSampler>(group, req));
    parts.push_back(std::make_shared<JointConstraintSampler>(group, req));
    return std::make_shared<UnionConstraintSampler>(group, std::move(parts));
  }
};

}  // namespace constraints
}  // namespace automanip
