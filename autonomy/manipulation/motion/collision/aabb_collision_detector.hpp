/*
 * Copyright 2026 The Openbot Authors
 *
 * Axis-aligned sphere / box / cylinder / mesh-AABB world collision.
 * Optional LinkFkTree enables approximate self-collision between link spheres.
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "autonomy/manipulation/common/collision_interface.hpp"
#include "autonomy/manipulation/model/link_fk.hpp"

namespace autonomy {
namespace manipulation {
namespace collision {

/**
 * @brief Lightweight AABB / sphere collision backend (no FCL dependency).
 *
 * World checks use EE / link spheres against scene boxes, spheres, cylinders,
 * and mesh AABBs. When a LinkFkTree is set, self-checks compare link spheres.
 */
class AabbCollisionDetector : public CollisionDetector {
 public:
  /**
   * @brief Store plugin id @p id (no heavyweight setup).
   * @return Always true.
   */
  bool Init(const std::string& id) override;

  /**
   * @brief Sphere–primitive collision of robot vs world / attached objects.
   * @return First contact pair when a hit is found.
   */
  CollisionResult CheckRobotWorld(
      const core::JointState& state,
      const scene::PlanningScene& scene) const override;

  /**
   * @brief Approximate self-collision between link spheres (requires LinkFkTree).
   * @return Collision with empty contacts if no tree is set or no hit.
   */
  CollisionResult CheckRobotSelf(
      const core::JointState& state,
      const scene::PlanningScene* scene = nullptr) const override;

  DistanceResult DistanceRobotWorld(
      const core::JointState& state,
      const scene::PlanningScene& scene) const override;

  /** @brief Nominal base→EE length used when no LinkFkTree is available. */
  void SetLinkLength(double length) { link_length_ = length; }

  /** @brief End-effector sphere radius for world / occupancy style queries. */
  void SetEeRadius(double r) { ee_radius_ = r; }

  /** @brief Link capsule / sphere radius for self-collision. */
  void SetLinkRadius(double r) override { link_radius_ = r; }

  /** @brief FK tree enabling per-link poses for self-collision. */
  void SetLinkTree(std::shared_ptr<const core::LinkFkTree> tree) override {
    link_tree_ = std::move(tree);
  }

 private:
  std::string id_;
  double link_length_ = 0.3;
  double ee_radius_ = 0.05;
  double link_radius_ = 0.04;
  std::shared_ptr<const core::LinkFkTree> link_tree_;
};

}  // namespace collision
}  // namespace manipulation
}  // namespace autonomy
