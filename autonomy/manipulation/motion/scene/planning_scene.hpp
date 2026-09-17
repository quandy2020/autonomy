/*
 * Copyright 2026 The Openbot Authors
 *
 * Planning scene (MoveIt planning_scene analogue).
 */

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/manipulation/model/link_forward_kinematics.hpp"
#include "autonomy/manipulation/model/robot_model.hpp"
#include "autonomy/manipulation/motion/scene/collision_object_helpers.hpp"
#include "autonomy/manipulation/proto/collision_query.pb.h"

namespace autonomy {
namespace manipulation {

namespace common {
class CollisionInterface;
}  // namespace common

namespace scene {

/** @brief Occupied voxel / point sample used for coarse occupancy checks. */
struct OccupiedPoint {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

/**
 * @brief Fast pairwise allowed-collision lookup (runtime cache).
 *
 * Wire format is @ref automsgs::msgs::moveit_msgs::AllowedCollisionMatrix (moveit_msgs); use
 * Get/SetAllowedCollisionMatrix for protobuf I/O.
 */
using AllowedCollisionLookup =
    std::unordered_map<std::string, std::unordered_set<std::string>>;

/**
 * @brief Planning-scene interface: robot state, world objects, ACM, occupancy.
 *
 * World / attached geometry uses automsgs::msgs::moveit_msgs::CollisionObject (no POD proxy).
 */
class PlanningScene {
 public:
  /**
   * @brief Define PlanningScene::SharedPtr type
   */
  AUTONOMY_SMART_PTR_DEFINITIONS(PlanningScene)

  virtual ~PlanningScene() = default;

  /** @brief Replace the current robot joint state snapshot. */
  virtual void SetCurrentState(const automsgs::msgs::sensor_msgs::JointState& state) = 0;

  /** @brief @return Copy of the last set robot joint state. */
  virtual automsgs::msgs::sensor_msgs::JointState GetCurrentState() const = 0;

  /** @brief Insert or replace a world collision object by @p object.id(). */
  virtual void AddCollisionObject(const automsgs::msgs::moveit_msgs::CollisionObject& object) = 0;

  /** @brief Remove a world object; no-op if @p id is unknown. */
  virtual void RemoveCollisionObject(const std::string& id) = 0;

  /**
   * @brief Attach @p attached.object() to @p attached.link_name().
   * Removes the same id from the world if present.
   */
  virtual void AttachObject(const automsgs::msgs::moveit_msgs::AttachedCollisionObject& attached) = 0;

  /** @brief Detach object @p object_id from the robot (does not re-add to world). */
  virtual void DetachObject(const std::string& object_id) = 0;

  /** @brief @return Snapshot of all currently attached objects. */
  virtual std::vector<automsgs::msgs::moveit_msgs::AttachedCollisionObject> GetAttachedObjects() const = 0;

  /**
   * @brief Whether @p state is considered valid for planning (no collision / occupancy).
   * @return true if valid.
   */
  virtual bool IsStateValid(const automsgs::msgs::sensor_msgs::JointState& state) const = 0;

  /**
   * @brief Collision query at @p state (robot–world and/or self per detector).
   * @return true if a collision is detected.
   */
  virtual bool CheckCollision(const automsgs::msgs::sensor_msgs::JointState& state) const = 0;

  /** @brief Collision query with optional contact body names. */
  using CollisionInfo = ::autonomy::manipulation::proto::CollisionResult;

  /** @brief Robot–world clearance (MoveIt distanceRobot lite). */
  using DistanceInfo = ::autonomy::manipulation::proto::DistanceResult;

  /**
   * @brief Detailed collision query (contacts when the backend provides them).
   * @param[in] state Robot configuration to test.
   * @return Collision flag and optional contact pair.
   */
  virtual CollisionInfo CheckCollisionDetailed(
      const automsgs::msgs::sensor_msgs::JointState& state) const {
    CollisionInfo info;
    info.set_collision(CheckCollision(state));
    return info;
  }

  /**
   * @brief Minimum robot–world distance (uses CollisionInterface when available).
   */
  virtual DistanceInfo DistanceRobotWorld(
      const automsgs::msgs::sensor_msgs::JointState& state) const {
    DistanceInfo d;
    d.set_distance(1e9);
    d.set_collision(CheckCollision(state));
    d.set_distance(d.collision() ? 0.0 : 1e3);
    return d;
  }

  /**
   * @brief Whether every waypoint of @p trajectory is collision-free / valid.
   * @return true if the path is valid.
   */
  virtual bool IsPathValid(const automsgs::msgs::trajectory_msgs::JointTrajectory& trajectory) const = 0;

  /** @brief @return Snapshot of world (non-attached) collision objects. */
  virtual std::vector<automsgs::msgs::moveit_msgs::CollisionObject> GetCollisionObjects() const = 0;

  /**
   * @brief Set whether bodies @p a and @p b may collide without reporting.
   * @param[in] allowed true to allow the pair, false to forbid.
   */
  virtual void SetAllowedCollision(const std::string& a, const std::string& b,
                                   bool allowed) = 0;

  /**
   * @brief Query the allowed-collision matrix for the unordered pair (@p a, @p b).
   * @return true if the pair is allowed.
   */
  virtual bool IsCollisionAllowed(const std::string& a,
                                  const std::string& b) const = 0;

  /** @brief @return Allowed-collision matrix as moveit_msgs. */
  virtual automsgs::msgs::moveit_msgs::AllowedCollisionMatrix GetAllowedCollisionMatrix() const = 0;

  /** @brief Replace the allowed-collision matrix from moveit_msgs. */
  virtual void SetAllowedCollisionMatrix(
      const automsgs::msgs::moveit_msgs::AllowedCollisionMatrix& matrix) = 0;

  /** @brief Install the collision backend used by CheckCollision / IsStateValid. */
  virtual void SetCollisionDetector(
      std::shared_ptr<common::CollisionInterface> detector) = 0;

  /** @brief Optional FK tree for attached-object / link-frame transforms. */
  virtual void SetLinkTree(
      std::shared_ptr<const model::LinkForwardKinematicsTree> tree) = 0;

  /** @brief @return Shared LinkForwardKinematicsTree if set (may be null). */
  virtual std::shared_ptr<const model::LinkForwardKinematicsTree> GetLinkTree() const {
    return nullptr;
  }

  /**
   * @brief Replace occupancy samples used for coarse environment checks.
   * @param[in] points Occupied centers in the world frame.
   * @param[in] resolution Voxel / query radius scale (meters).
   */
  virtual void SetOccupiedPoints(std::vector<OccupiedPoint> points,
                                 double resolution) = 0;

  /** @brief Clear occupancy samples. */
  virtual void ClearOccupiedPoints() = 0;

  /** @brief Remove all world (non-attached) collision objects. */
  virtual void ClearWorldObjects() = 0;

  /** @brief Detach and drop all attached collision objects. */
  virtual void ClearAttachedObjects() = 0;

  /** @brief @return Snapshot of occupied points. */
  virtual std::vector<OccupiedPoint> OccupiedPoints() const = 0;

  /** @brief @return Occupancy resolution last set with SetOccupiedPoints. */
  virtual double OccupancyResolution() const = 0;

  /**
   * @brief Dense path check with midpoint samples between waypoints.
   * Default: waypoint-only IsPathValid.
   */
  virtual bool IsPathValidDense(const automsgs::msgs::trajectory_msgs::JointTrajectory& trajectory,
                                int segments_per_edge = 4) const {
    (void)segments_per_edge;
    return IsPathValid(trajectory);
  }
};

}  // namespace scene
}  // namespace manipulation
}  // namespace autonomy
