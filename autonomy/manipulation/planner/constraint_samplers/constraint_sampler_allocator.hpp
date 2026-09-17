/*
 * Copyright 2026 The Openbot Authors
 *
 * ConstraintSamplerAllocator (MoveIt constraint_samplers lite).
 */

#pragma once

#include <memory>
#include <string>

#include "autonomy/manipulation/planner/constraint_samplers/constraint_sampler.hpp"
#include "autonomy/manipulation/common/planner_interface.hpp"
#include "autonomy/manipulation/motion/scene/planning_scene.hpp"

namespace autonomy {
namespace manipulation {
namespace constraint_samplers {

/**
 * @brief Factory for ConstraintSampler (MoveIt ConstraintSamplerAllocator).
 */
class ConstraintSamplerAllocator {
 public:
  virtual ~ConstraintSamplerAllocator() = default;

  virtual bool CanService(const scene::PlanningScene* /*scene*/,
                          const std::string& /*group*/,
                          const planning::MotionPlanRequest& req) const = 0;

  virtual std::shared_ptr<ConstraintSampler> Alloc(
      const scene::PlanningScene* scene, const std::string& group,
      const planning::MotionPlanRequest& req) const = 0;
};

/** @brief Serves joint-only constraints. */
class JointConstraintSamplerAllocator : public ConstraintSamplerAllocator {
 public:
  bool CanService(const scene::PlanningScene* /*scene*/,
                  const std::string& /*group*/,
                  const planning::MotionPlanRequest& req) const override {
    return !req.joint_constraints.empty() &&
           req.position_constraints.empty() &&
           req.orientation_constraints.empty();
  }

  std::shared_ptr<ConstraintSampler> Alloc(
      const scene::PlanningScene* /*scene*/, const std::string& group,
      const planning::MotionPlanRequest& req) const override {
    return std::make_shared<JointConstraintSampler>(group, req);
  }
};

/** @brief Serves Cartesian position/orientation constraints via IK. */
class IkConstraintSamplerAllocator : public ConstraintSamplerAllocator {
 public:
  bool CanService(const scene::PlanningScene* /*scene*/,
                  const std::string& /*group*/,
                  const planning::MotionPlanRequest& req) const override {
    return req.kinematics && (!req.position_constraints.empty() ||
                              !req.orientation_constraints.empty()) &&
           req.joint_constraints.empty();
  }

  std::shared_ptr<ConstraintSampler> Alloc(
      const scene::PlanningScene* /*scene*/, const std::string& group,
      const planning::MotionPlanRequest& req) const override {
    return std::make_shared<IkConstraintSampler>(group, req);
  }
};

/**
 * @brief Serves joint + Cartesian together via UnionConstraintSampler.
 *
 * Registered first so mixed constraints do not collapse to IK-only.
 */
class UnionConstraintSamplerAllocator : public ConstraintSamplerAllocator {
 public:
  bool CanService(const scene::PlanningScene* /*scene*/,
                  const std::string& /*group*/,
                  const planning::MotionPlanRequest& req) const override {
    const bool has_cart = req.kinematics &&
                          (!req.position_constraints.empty() ||
                           !req.orientation_constraints.empty());
    return has_cart && !req.joint_constraints.empty();
  }

  std::shared_ptr<ConstraintSampler> Alloc(
      const scene::PlanningScene* /*scene*/, const std::string& group,
      const planning::MotionPlanRequest& req) const override {
    std::vector<std::shared_ptr<ConstraintSampler>> parts;
    parts.push_back(std::make_shared<IkConstraintSampler>(group, req));
    parts.push_back(std::make_shared<JointConstraintSampler>(group, req));
    return std::make_shared<UnionConstraintSampler>(group, std::move(parts));
  }
};

}  // namespace constraint_samplers
}  // namespace manipulation
}  // namespace autonomy
