/*
 * Copyright 2026 The Openbot Authors
 *
 * Collision detector plugin interface (common).
 */

#pragma once

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "autonomy/manipulation/motion/collision/link_collision_geometry.hpp"
#include "autonomy/manipulation/model/link_fk.hpp"
#include "autonomy/manipulation/model/robot_model.hpp"
#include "autonomy/manipulation/motion/scene/planning_scene.hpp"

namespace autonomy {
namespace manipulation {
namespace collision {

/**
 * @brief Result of a collision query.
 *
 * When @p collision is true, @p contact_body_a / @p contact_body_b name the
 * first contacting pair when the backend provides them (may be empty).
 */
struct CollisionResult {
  bool collision = false;
  std::string contact_body_a;
  std::string contact_body_b;
};

/**
 * @brief Minimum distance query (MoveIt CollisionEnv::distanceRobot analogue).
 *
 * @p distance is signed when the backend supports it (negative = penetration);
 * otherwise ≥0 with collision ⇒ 0.
 */
struct DistanceResult {
  double distance = 1e9;
  bool collision = false;
  std::string nearest_body_a;
  std::string nearest_body_b;
  double nearest_x = 0.0;
  double nearest_y = 0.0;
  double nearest_z = 0.0;
};

/**
 * @brief Collision-detector plugin interface (robot–world and robot–self).
 *
 * Concrete backends (AABB, FCL, …) are created via the manipulation plugin hub
 * and installed on a PlanningScene.
 */
class CollisionDetector {
 public:
  virtual ~CollisionDetector() = default;

  /**
   * @brief Initialize the detector with plugin / config id @p id.
   * @return true on success.
   */
  virtual bool Init(const std::string& id) = 0;

  /**
   * @brief Check robot links / EE against world (and attached) objects in @p scene.
   * @param[in] state Robot configuration to test.
   * @param[in] scene Source of world / attached geometry and ACM.
   * @return CollisionResult with collision flag and optional contact names.
   */
  virtual CollisionResult CheckRobotWorld(
      const core::JointState& state,
      const scene::PlanningScene& scene) const = 0;

  /**
   * @brief Check approximate robot self-collision at @p state.
   * @param[in] state Robot configuration.
   * @param[in] scene Optional ACM source (SRDF disable_collisions); may be null.
   * @return CollisionResult with collision flag and optional contact names.
   */
  virtual CollisionResult CheckRobotSelf(
      const core::JointState& state,
      const scene::PlanningScene* scene = nullptr) const = 0;

  /**
   * @brief Minimum robot–world distance at @p state (optional; default ≈ collision).
   * FCL backend returns true nearest-pair distance; AABB uses sphere proxies.
   */
  virtual DistanceResult DistanceRobotWorld(
      const core::JointState& state,
      const scene::PlanningScene& scene) const {
    DistanceResult d;
    const auto c = CheckRobotWorld(state, scene);
    d.collision = c.collision;
    d.distance = c.collision ? 0.0 : 1e3;
    d.nearest_body_a = c.contact_body_a;
    d.nearest_body_b = c.contact_body_b;
    return d;
  }

  /**
   * @brief Optional FK tree for per-link poses (full-chain backends).
   * @param[in] tree Shared LinkFkTree; may be null to clear.
   */
  virtual void SetLinkTree(std::shared_ptr<const core::LinkFkTree> /*tree*/) {}

  /**
   * @brief Nominal link capsule / sphere radius (meters).
   * @param[in] r Radius used when URDF collision geometry is absent.
   */
  virtual void SetLinkRadius(double /*r*/) {}

  /**
   * @brief Optional per-link URDF collision geometry (primitives / convex hulls).
   * @param[in] model Shared model; null clears to uniform-radius spheres.
   */
  virtual void SetLinkCollisionModel(
      std::shared_ptr<const LinkCollisionModel> /*model*/) {}

  /**
   * @brief Collision padding / contact distance (meters), MoveIt padding analogue.
   * Inflates primitives and enables FCL contact-distance checks when > 0.
   */
  virtual void SetPadding(double padding) { padding_ = std::max(0.0, padding); }

  double GetPadding() const { return padding_; }

 protected:
  double padding_ = 0.0;
};

}  // namespace collision
}  // namespace manipulation
}  // namespace autonomy
