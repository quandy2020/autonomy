/*
 * Copyright 2026 The Openbot Authors
 *
 * FCL collision detector (FEATURES fcl).
 */

#pragma once

#include <memory>
#include <string>

#include "autonomy/manipulation/collision/collision_detector.hpp"

namespace autonomy {
namespace manipulation {
namespace collision {

/**
 * @brief FCL-backed CollisionDetector (built when FEATURES includes fcl).
 *
 * Uses simple EE / link spheres against scene geometry; falls back or stubs
 * when FCL is unavailable at build time.
 */
class FclCollisionDetector : public CollisionDetector {
 public:
  /**
   * @brief Store plugin id @p id.
   * @return true on success.
   */
  bool Init(const std::string& id) override;

  /**
   * @brief Robot–world collision via FCL (or stub equivalent).
   * @return First contact pair when a hit is found.
   */
  CollisionResult CheckRobotWorld(
      const core::JointState& state,
      const scene::PlanningScene& scene) const override;

  /**
   * @brief Robot self-collision query (may be limited / stubbed).
   * @return CollisionResult for the self-check.
   */
  CollisionResult CheckRobotSelf(const core::JointState& state) const override;

  /** @brief Nominal base→EE length for simplified robot geometry. */
  void SetLinkLength(double length) { link_length_ = length; }

  /** @brief End-effector sphere radius. */
  void SetEeRadius(double r) { ee_radius_ = r; }

 private:
  std::string id_;
  double link_length_ = 0.3;
  double ee_radius_ = 0.05;
};

/**
 * @brief Factory for an FCL detector instance (shared_ptr).
 * @return New FclCollisionDetector (or stub implementation when FCL is off).
 */
std::shared_ptr<CollisionDetector> CreateFclCollisionDetector();

}  // namespace collision
}  // namespace manipulation
}  // namespace autonomy
