/*
 * Copyright 2026 The Openbot Authors
 *
 * ConstraintSamplerManager (MoveIt constraint_samplers lite).
 */

#pragma once

#include <memory>
#include <random>
#include <string>
#include <vector>

#include "autonomy/manipulation/constraints/constraint_sampler_types.hpp"
#include "autonomy/manipulation/common/planner_interface.hpp"

namespace autonomy {
namespace manipulation {
namespace constraints {

/**
 * @brief Selects samplers via registered Allocators (MoveIt Manager lite).
 *
 * Falls back to default Joint / IK selection when no allocator matches.
 */
class ConstraintSamplerManager {
 public:
  ConstraintSamplerManager() { LoadRegisteredAllocators(); }

  void SetDefaultSeed(unsigned seed) { seed_ = seed; }

  void RegisterAllocator(ConstraintSamplerAllocator::SharedPtr alloc) {
    if (alloc) {
      allocators_.push_back(std::move(alloc));
    }
  }

  /**
   * @brief Reload allocators from Autolink PluginManager (dynamic registration).
   *
   * Falls back to built-in Union → IK → Joint if no plugins resolve.
   */
  void LoadRegisteredAllocators();

  /**
   * @brief Load external plugin description files then reload allocators.
   * @param[in] plugins_list_file Text file: one description path per line.
   * @return Number of description files successfully loaded.
   */
  int LoadExternalPluginDescriptions(const std::string& plugins_list_file);

  /** @brief Pick first allocator that CanService, else default heuristic. */
  ConstraintSampler::SharedPtr SelectSampler(
      const scene::PlanningScene* scene, const std::string& group,
      const planner::MotionPlanRequest& req) const {
    for (const auto& a : allocators_) {
      if (a && a->CanService(scene, group, req)) {
        return a->Alloc(scene, group, req);
      }
    }
    return SelectDefaultSampler(scene, group, req);
  }

  ConstraintSampler::SharedPtr SelectDefaultSampler(
      const scene::PlanningScene* /*scene*/, const std::string& group,
      const planner::MotionPlanRequest& req) const {
    const bool has_cart = req.kinematics &&
                          (!(req.pb.position_constraints_size() == 0) ||
                           !(req.pb.orientation_constraints_size() == 0));
    if (has_cart && !(req.pb.joint_constraints_size() == 0)) {
      std::vector<ConstraintSampler::SharedPtr> parts;
      parts.push_back(std::make_shared<IkConstraintSampler>(group, req));
      parts.push_back(std::make_shared<JointConstraintSampler>(group, req));
      return std::make_shared<UnionConstraintSampler>(group, std::move(parts));
    }
    if (has_cart) {
      return std::make_shared<IkConstraintSampler>(group, req);
    }
    return std::make_shared<JointConstraintSampler>(group, req);
  }

  /**
   * @brief Sample one valid state for @p req into @p state.
   */
  bool Sample(const planner::MotionPlanRequest& req,
              const automsgs::msgs::sensor_msgs::JointState& seed,
              automsgs::msgs::sensor_msgs::JointState* state,
              int max_attempts = 32) const {
    if (!state) {
      return false;
    }
    auto sampler = SelectSampler(req.scene.get(), req.pb.group(), req);
    if (!sampler) {
      return false;
    }
    std::mt19937 rng(seed_);
    return sampler->Sample(seed, state, &rng, max_attempts);
  }

  /** @brief Project @p state onto constraints when possible. */
  bool Project(const planner::MotionPlanRequest& req,
               automsgs::msgs::sensor_msgs::JointState* state) const {
    if (!state) {
      return false;
    }
    auto sampler = SelectSampler(req.scene.get(), req.pb.group(), req);
    if (!sampler) {
      return false;
    }
    return sampler->Project(state);
  }

 private:
  unsigned seed_ = 42;
  std::vector<ConstraintSamplerAllocator::SharedPtr> allocators_;
};

}  // namespace constraints
}  // namespace manipulation
}  // namespace autonomy
