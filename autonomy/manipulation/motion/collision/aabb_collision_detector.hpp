/*
 * Copyright 2026 The Openbot Authors
 *
 * Axis-aligned sphere / box / cylinder / mesh-AABB world collision.
 * Optional LinkForwardKinematicsTree enables approximate self-collision between link spheres.
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "autonomy/manipulation/common/collision_interface.hpp"
#include "autonomy/manipulation/model/link_forward_kinematics.hpp"

namespace autonomy {
namespace manipulation {
namespace collision {

/**
 * @brief Lightweight AABB / sphere collision backend (no FCL dependency).
 *
 * World checks use EE / link spheres against scene boxes, spheres, cylinders,
 * and mesh AABBs. When a LinkForwardKinematicsTree is set, self-checks compare link spheres.
 */
class AabbCollisionDetector : public common::CollisionInterface {
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
  ::autonomy::manipulation::proto::CollisionResult CheckRobotWorld(
      const automsgs::msgs::sensor_msgs::JointState& state,
      const scene::PlanningScene& scene) const override;

  /**
   * @brief Approximate self-collision between link spheres (requires LinkForwardKinematicsTree).
   * @return Collision with empty contacts if no tree is set or no hit.
   */
  ::autonomy::manipulation::proto::CollisionResult CheckRobotSelf(
      const automsgs::msgs::sensor_msgs::JointState& state,
      const scene::PlanningScene* scene = nullptr) const override;

  ::autonomy::manipulation::proto::DistanceResult DistanceRobotWorld(
      const automsgs::msgs::sensor_msgs::JointState& state,
      const scene::PlanningScene& scene) const override;

  /** @brief Nominal base→EE length used when no LinkForwardKinematicsTree is available. */
  void SetLinkLength(double link_length_m) { link_length_ = link_length_m; }

  /** @brief End-effector sphere radius for world / occupancy style queries. */
  void SetEndEffectorRadius(double end_effector_radius_m) { end_effector_radius_ = end_effector_radius_m; }

  /** @brief Link capsule / sphere radius for self-collision. */
  void SetLinkRadius(double nominal_link_radius_m) override {
    link_radius_ = nominal_link_radius_m;
  }

  /** @brief FK tree enabling per-link poses for self-collision. */
  void SetLinkTree(
      std::shared_ptr<const model::LinkForwardKinematicsTree> link_fk_tree) override {
    link_tree_ = std::move(link_fk_tree);
  }

 private:
  std::string id_;
  double link_length_ = 0.3;
  double end_effector_radius_ = 0.05;
  double link_radius_ = 0.04;
  std::shared_ptr<const model::LinkForwardKinematicsTree> link_tree_;
};

}  // namespace collision
}  // namespace manipulation
}  // namespace autonomy
