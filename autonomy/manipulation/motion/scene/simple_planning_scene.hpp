/*
 * Copyright 2026 The Openbot Authors
 *
 * In-memory PlanningScene with shared-mutex state and optional occupancy.
 */

#pragma once

#include <memory>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "autonomy/manipulation/motion/scene/planning_scene.hpp"

namespace autonomy {
namespace manipulation {
namespace scene {

/**
 * @brief Thread-safe default PlanningScene: objects, ACM, detector, occupancy.
 *
 * Readers take a shared lock; mutators take an exclusive lock. Collision and
 * occupancy checks use the configured CollisionInterface / LinkForwardKinematicsTree when set.
 */
class SimplePlanningScene : public PlanningScene {
 public:
  /** @brief Replace the current robot joint state snapshot. */
  void SetCurrentState(const automsgs::msgs::sensor_msgs::JointState& state) override;

  /** @brief @return Copy of the last set robot joint state. */
  automsgs::msgs::sensor_msgs::JointState GetCurrentState() const override;

  /** @brief Insert or replace a world collision object by @p object.id. */
  void AddCollisionObject(const automsgs::msgs::moveit_msgs::CollisionObject& object) override;

  /** @brief Remove a world object; no-op if @p id is unknown. */
  void RemoveCollisionObject(const std::string& id) override;

  /** @brief Attach @p attached.object to @p attached.link_name. */
  void AttachObject(const automsgs::msgs::moveit_msgs::AttachedCollisionObject& attached) override;

  /** @brief Detach object @p object_id from the robot. */
  void DetachObject(const std::string& object_id) override;

  /** @brief @return Snapshot of all currently attached objects. */
  std::vector<automsgs::msgs::moveit_msgs::AttachedCollisionObject> GetAttachedObjects() const override;

  /**
   * @brief Whether @p state is free of detector collision and occupancy hits.
   * @return true if valid.
   */
  bool IsStateValid(const automsgs::msgs::sensor_msgs::JointState& state) const override;

  /**
   * @brief Run the configured detector (and occupancy) at @p state.
   * @return true if a collision / occupancy hit is detected.
   */
  bool CheckCollision(const automsgs::msgs::sensor_msgs::JointState& state) const override;

  /**
   * @brief Detailed collision query with contact body names when available.
   * @param[in] state Robot configuration to test.
   * @return Collision flag and optional contact pair.
   */
  CollisionInfo CheckCollisionDetailed(
      const automsgs::msgs::sensor_msgs::JointState& state) const override;

  DistanceInfo DistanceRobotWorld(const automsgs::msgs::sensor_msgs::JointState& state) const override;

  /**
   * @brief Validate every waypoint of @p trajectory via IsStateValid.
   * @return true if the path is valid.
   */
  bool IsPathValid(const automsgs::msgs::trajectory_msgs::JointTrajectory& trajectory) const override;

  /** @brief @return Snapshot of world (non-attached) collision objects. */
  std::vector<automsgs::msgs::moveit_msgs::CollisionObject> GetCollisionObjects() const override;

  /**
   * @brief Set whether bodies @p a and @p b may collide without reporting.
   * @param[in] allowed true to allow the pair, false to forbid.
   */
  void SetAllowedCollision(const std::string& a, const std::string& b,
                           bool allowed) override;

  /**
   * @brief Query the ACM for the unordered pair (@p a, @p b).
   * @return true if the pair is allowed.
   */
  bool IsCollisionAllowed(const std::string& a,
                          const std::string& b) const override;

  /** @brief @return Allowed-collision matrix as moveit_msgs. */
  automsgs::msgs::moveit_msgs::AllowedCollisionMatrix GetAllowedCollisionMatrix() const override;

  /** @brief Replace the allowed-collision matrix from moveit_msgs. */
  void SetAllowedCollisionMatrix(
      const automsgs::msgs::moveit_msgs::AllowedCollisionMatrix& matrix) override;

  /** @brief Install the collision backend used by CheckCollision / IsStateValid. */
  void SetCollisionDetector(
      std::shared_ptr<common::CollisionInterface> detector) override;

  /** @brief Optional FK tree for attached-object transforms. */
  void SetLinkTree(std::shared_ptr<const model::LinkForwardKinematicsTree> tree) override;

  std::shared_ptr<const model::LinkForwardKinematicsTree> GetLinkTree() const override;

  bool IsPathValidDense(const automsgs::msgs::trajectory_msgs::JointTrajectory& trajectory,
                        int segments_per_edge = 4) const override;

  /**
   * @brief Replace occupancy samples used for coarse environment checks.
   * @param[in] points Occupied centers in the world frame.
   * @param[in] resolution Voxel / query radius scale (meters).
   */
  void SetOccupiedPoints(std::vector<OccupiedPoint> points,
                         double resolution) override;

  /** @brief Clear occupancy samples. */
  void ClearOccupiedPoints() override;

  /** @brief Remove all world (non-attached) collision objects. */
  void ClearWorldObjects() override;

  /** @brief Detach and drop all attached collision objects. */
  void ClearAttachedObjects() override;

  /** @brief @return Snapshot of occupied points. */
  std::vector<OccupiedPoint> OccupiedPoints() const override;

  /** @brief @return Occupancy resolution last set with SetOccupiedPoints. */
  double OccupancyResolution() const override;

 private:
  /** @brief True if EE / link samples intersect occupied voxels at @p state. */
  bool CheckOccupancy(const automsgs::msgs::sensor_msgs::JointState& state) const;

  /**
   * @brief World objects plus attached objects posed into the world frame.
   * @return Combined collision object list for detector queries.
   */
  std::vector<automsgs::msgs::moveit_msgs::CollisionObject> GetWorldAndAttachedObjects(
      const automsgs::msgs::sensor_msgs::JointState& state) const;

  mutable std::shared_mutex mutex_;
  automsgs::msgs::sensor_msgs::JointState current_;
  std::unordered_map<std::string, automsgs::msgs::moveit_msgs::CollisionObject> objects_;
  std::unordered_map<std::string, automsgs::msgs::moveit_msgs::AttachedCollisionObject> attached_;
  AllowedCollisionLookup acm_;
  std::shared_ptr<common::CollisionInterface> detector_;
  std::shared_ptr<const model::LinkForwardKinematicsTree> link_tree_;
  std::vector<OccupiedPoint> occupied_;
  double occupancy_resolution_ = 0.05;
};

}  // namespace scene
}  // namespace manipulation
}  // namespace autonomy
