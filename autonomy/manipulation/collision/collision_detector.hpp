/*
 * Copyright 2026 The Openbot Authors
 *
 * Collision detector plugin base.
 */

#pragma once

#include <string>
#include <vector>

#include "autonomy/manipulation/core/robot_model.hpp"
#include "autonomy/manipulation/scene/planning_scene.hpp"

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
   * @return CollisionResult with collision flag and optional contact names.
   */
  virtual CollisionResult CheckRobotSelf(
      const core::JointState& state) const = 0;
};

}  // namespace collision
}  // namespace manipulation
}  // namespace autonomy
