/*
 * Copyright 2026 The Openbot Authors
 *
 * FCL collision detector (FEATURES fcl) — full-chain LinkFk + ACM + URDF mesh.
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "autonomy/manipulation/common/collision_interface.hpp"
#include "autonomy/manipulation/motion/collision/link_collision_geometry.hpp"
#include "autonomy/manipulation/model/link_fk.hpp"

namespace autonomy {
namespace manipulation {
namespace collision {

/**
 * @brief FCL-backed CollisionDetector with per-link FK geometry.
 *
 * Prefers URDF collision shapes (sphere / box / cylinder / convex hull of mesh).
 * Falls back to uniform link spheres, then planar EE-sphere proxy.
 * World meshes use convex hull (or AABB box if hull fails).
 */
class FclCollisionDetector : public CollisionDetector {
 public:
  bool Init(const std::string& id) override;

  CollisionResult CheckRobotWorld(
      const core::JointState& state,
      const scene::PlanningScene& scene) const override;

  CollisionResult CheckRobotSelf(
      const core::JointState& state,
      const scene::PlanningScene* scene = nullptr) const override;

  DistanceResult DistanceRobotWorld(
      const core::JointState& state,
      const scene::PlanningScene& scene) const override;

  void SetPadding(double padding) override {
    padding_ = std::max(0.0, padding);
  }

  void SetLinkTree(std::shared_ptr<const core::LinkFkTree> tree) override {
    link_tree_ = std::move(tree);
  }

  void SetLinkRadius(double r) override { link_radius_ = r; }

  void SetLinkCollisionModel(
      std::shared_ptr<const LinkCollisionModel> model) override {
    link_shapes_ = std::move(model);
  }

  /** @brief Nominal base→EE length for EE-proxy fallback. */
  void SetLinkLength(double length) { link_length_ = length; }

  /** @brief End-effector sphere radius for EE-proxy fallback. */
  void SetEeRadius(double r) { ee_radius_ = r; }

 private:
  std::string id_;
  double link_length_ = 0.3;
  double ee_radius_ = 0.05;
  double link_radius_ = 0.04;
  std::shared_ptr<const core::LinkFkTree> link_tree_;
  std::shared_ptr<const LinkCollisionModel> link_shapes_;
};

/**
 * @brief Factory for an FCL detector instance.
 * @return New FclCollisionDetector.
 */
std::shared_ptr<CollisionDetector> CreateFclCollisionDetector();

}  // namespace collision
}  // namespace manipulation
}  // namespace autonomy
