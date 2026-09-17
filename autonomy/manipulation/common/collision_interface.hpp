/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <algorithm>
#include <memory>
#include <string>

#include <automsgs/msgs/sensor_msgs/joint_state.pb.h>

#include "autonomy/common/macros.hpp"
#include "autonomy/manipulation/model/link_forward_kinematics.hpp"
#include "autonomy/manipulation/motion/collision/link_collision_geometry.hpp"
#include "autonomy/manipulation/motion/scene/planning_scene.hpp"
#include "autonomy/manipulation/proto/collision_query.pb.h"

namespace autonomy {
namespace manipulation {
namespace common {

/**
 * @class CollisionInterface
 * @brief Collision-detector plugin interface (robot–world and robot–self).
 *
 * Concrete backends (AABB, FCL, …) are created via the manipulation plugin hub
 * and installed on a PlanningScene.
 */
class CollisionInterface
{
public:
  /**
   * @brief Define CollisionInterface::SharedPtr type
   */
  AUTONOMY_SMART_PTR_DEFINITIONS(CollisionInterface)

  /**
   * @brief Destructor for CollisionInterface
   */
  virtual ~CollisionInterface() = default;

  /**
   * @brief Initialize the detector with plugin / config id.
   * @param plugin_id Registry or config identifier.
   * @return true on success.
   */
  virtual bool Init(const std::string& plugin_id) = 0;

  /**
   * @brief Check robot links / EE against world (and attached) objects.
   * @param joint_state Robot configuration to test.
   * @param planning_scene Source of world / attached geometry and ACM.
   * @return Collision result (binary flag and optional contact bodies).
   */
  virtual ::autonomy::manipulation::proto::CollisionResult CheckRobotWorld(
      const automsgs::msgs::sensor_msgs::JointState& joint_state,
      const scene::PlanningScene& planning_scene) const = 0;

  /**
   * @brief Check approximate robot self-collision.
   * @param joint_state Robot configuration.
   * @param planning_scene Optional ACM source (SRDF disable_collisions).
   * @return Collision result (binary flag and optional contact bodies).
   */
  virtual ::autonomy::manipulation::proto::CollisionResult CheckRobotSelf(
      const automsgs::msgs::sensor_msgs::JointState& joint_state,
      const scene::PlanningScene* planning_scene = nullptr) const = 0;

  /**
   * @brief Minimum robot–world clearance (optional; default ≈ binary collision).
   * @param joint_state Robot configuration.
   * @param planning_scene World / attached geometry source.
   * @return Distance result with nearest bodies when available.
   */
  virtual ::autonomy::manipulation::proto::DistanceResult DistanceRobotWorld(
      const automsgs::msgs::sensor_msgs::JointState& joint_state,
      const scene::PlanningScene& planning_scene) const {
    ::autonomy::manipulation::proto::DistanceResult distance_result;
    distance_result.set_distance(1e9);
    const ::autonomy::manipulation::proto::CollisionResult collision_result =
        CheckRobotWorld(joint_state, planning_scene);
    distance_result.set_collision(collision_result.collision());
    distance_result.set_distance(collision_result.collision() ? 0.0 : 1e3);
    distance_result.set_nearest_body_a(collision_result.contact_body_a());
    distance_result.set_nearest_body_b(collision_result.contact_body_b());
    return distance_result;
  }

  /**
   * @brief Optional FK tree for per-link poses (full-chain backends).
   * @param link_fk_tree Shared LinkForwardKinematicsTree; may be null to clear.
   */
  virtual void SetLinkTree(
      std::shared_ptr<const model::LinkForwardKinematicsTree> /*link_fk_tree*/) {}

  /**
   * @brief Nominal link capsule / sphere radius when URDF geometry is absent.
   * @param nominal_link_radius_m Radius in meters.
   */
  virtual void SetLinkRadius(double /*nominal_link_radius_m*/) {}

  /**
   * @brief Optional per-link URDF collision geometry (primitives / convex hulls).
   * @param link_collision_model Shared model; null clears to uniform spheres.
   */
  virtual void SetLinkCollisionModel(
      std::shared_ptr<const collision::LinkCollisionModel>
          /*link_collision_model*/) {}

  /**
   * @brief Collision padding / contact distance (MoveIt padding analogue).
   * @param contact_padding_m Inflates primitives; meters, clamped ≥ 0.
   */
  virtual void SetPadding(double contact_padding_m) {
    contact_padding_m_ = std::max(0.0, contact_padding_m);
  }

  /**
   * @brief Current contact padding in meters.
   * @return Contact / collision padding applied to robot geometry.
   */
  double GetPadding() const { return contact_padding_m_; }

protected:
  /**
   * @brief Default constructor for plugin registration only.
   */
  CollisionInterface() = default;

  /** Contact / collision padding applied to robot geometry (meters). */
  double contact_padding_m_ = 0.0;
};

}  // namespace common
}  // namespace manipulation
}  // namespace autonomy
