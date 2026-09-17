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
#include "autonomy/manipulation/model/link_forward_kinematics.hpp"

namespace autonomy {
namespace manipulation {
namespace collision {

/**
 * @brief FCL-backed CollisionInterface with per-link FK geometry.
 *
 * Prefers URDF collision shapes (sphere / box / cylinder / convex hull of mesh).
 * Falls back to uniform link spheres, then planar EE-sphere proxy.
 * World meshes use convex hull (or AABB box if hull fails).
 */
class FclCollisionDetector : public common::CollisionInterface {
 public:
  bool Init(const std::string& id) override;

  ::autonomy::manipulation::proto::CollisionResult CheckRobotWorld(
      const automsgs::msgs::sensor_msgs::JointState& state,
      const scene::PlanningScene& scene) const override;

  ::autonomy::manipulation::proto::CollisionResult CheckRobotSelf(
      const automsgs::msgs::sensor_msgs::JointState& state,
      const scene::PlanningScene* scene = nullptr) const override;

  ::autonomy::manipulation::proto::DistanceResult DistanceRobotWorld(
      const automsgs::msgs::sensor_msgs::JointState& state,
      const scene::PlanningScene& scene) const override;

  void SetPadding(double contact_padding_m) override {
    contact_padding_m_ = std::max(0.0, contact_padding_m);
  }

  void SetLinkTree(
      std::shared_ptr<const model::LinkForwardKinematicsTree> link_fk_tree) override {
    link_tree_ = std::move(link_fk_tree);
  }

  void SetLinkRadius(double nominal_link_radius_m) override {
    link_radius_ = nominal_link_radius_m;
  }

  void SetLinkCollisionModel(
      std::shared_ptr<const LinkCollisionModel> link_collision_model) override {
    link_shapes_ = std::move(link_collision_model);
  }

  /** @brief Nominal base→EE length for EE-proxy fallback. */
  void SetLinkLength(double link_length_m) { link_length_ = link_length_m; }

  /** @brief End-effector sphere radius for EE-proxy fallback. */
  void SetEndEffectorRadius(double end_effector_radius_m) { end_effector_radius_ = end_effector_radius_m; }

 private:
  std::string id_;
  double link_length_ = 0.3;
  double end_effector_radius_ = 0.05;
  double link_radius_ = 0.04;
  std::shared_ptr<const model::LinkForwardKinematicsTree> link_tree_;
  std::shared_ptr<const LinkCollisionModel> link_shapes_;
};

/**
 * @brief Factory for an FCL detector instance.
 * @return New FclCollisionDetector.
 */
common::CollisionInterface::SharedPtr CreateFclCollisionDetector();

}  // namespace collision
}  // namespace manipulation
}  // namespace autonomy
