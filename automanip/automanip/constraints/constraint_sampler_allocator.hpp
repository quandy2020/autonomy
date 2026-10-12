/*
 * Copyright 2026 The Openbot Authors
 *
 * ConstraintSamplerAllocator base (MoveIt constraint_samplers lite).
 */

#pragma once

#include <memory>
#include <string>

#include "autonomy/common/macros.hpp"
#include "automanip/constraints/constraint_sampler.hpp"
#include "automanip/common/planner_interface.hpp"
#include "automanip/motion/scene/planning_scene.hpp"

namespace automanip {
namespace constraints {

/**
 * @brief Factory for ConstraintSampler (MoveIt ConstraintSamplerAllocator).
 */
class ConstraintSamplerAllocator {
 public:
  AUTONOMY_SMART_PTR_DEFINITIONS(ConstraintSamplerAllocator)

  virtual ~ConstraintSamplerAllocator() = default;

  virtual bool CanService(const scene::PlanningScene* /*scene*/,
                          const std::string& /*group*/,
                          const planner::MotionPlanRequest& req) const = 0;

  virtual ConstraintSampler::SharedPtr Alloc(
      const scene::PlanningScene* scene, const std::string& group,
      const planner::MotionPlanRequest& req) const = 0;
};

}  // namespace constraints
}  // namespace automanip
