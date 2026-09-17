/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include "autonomy/manipulation/constraints/constraint_sampler_allocator.hpp"
#include "autonomy/manipulation/constraints/ik_constraint_sampler.hpp"

namespace autonomy {
namespace manipulation {
namespace constraints {

/** @brief Serves Cartesian position/orientation constraints via IK. */
class IkConstraintSamplerAllocator : public ConstraintSamplerAllocator {
 public:
  bool CanService(const scene::PlanningScene* /*scene*/,
                  const std::string& /*group*/,
                  const planner::MotionPlanRequest& req) const override {
    return req.kinematics && (!(req.pb.position_constraints_size() == 0) ||
                              !(req.pb.orientation_constraints_size() == 0)) &&
           (req.pb.joint_constraints_size() == 0);
  }

  ConstraintSampler::SharedPtr Alloc(
      const scene::PlanningScene* /*scene*/, const std::string& group,
      const planner::MotionPlanRequest& req) const override {
    return std::make_shared<IkConstraintSampler>(group, req);
  }
};

}  // namespace constraints
}  // namespace manipulation
}  // namespace autonomy
