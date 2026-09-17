/*
 * Copyright 2026 The Openbot Authors
 *
 * Scene monitor: ApplySceneDiff + joint-state driven updates.
 */

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <automsgs/msgs/map_msgs/octomap_with_pose.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

#include "autonomy/manipulation/motion/execution/joint_state_subscriber.hpp"
#include "autonomy/manipulation/motion/scene/planning_scene.hpp"

namespace autonomy {
namespace manipulation {
namespace scene {

/**
 * @brief Incremental scene update: objects, attachments, state, occupancy.
 *
 * Flags (`has_robot_state`, `has_occupancy`, `clear_occupancy`) select which
 * fields SceneMonitor applies; empty vectors are no-ops for add/remove lists.
 */
struct SceneDiff {
  std::vector<CollisionObject> add;
  std::vector<std::string> remove;
  std::vector<AttachedCollisionObject> attach;
  std::vector<std::string> detach;
  core::JointState robot_state;
  bool has_robot_state = false;
  std::vector<OccupiedPoint> occupied;
  double occupancy_resolution = 0.05;
  bool has_occupancy = false;
  bool clear_occupancy = false;
  /** @brief If true, remove all world collision objects before add/remove lists. */
  bool clear_world = false;
  /** @brief If true, detach all attached objects before attach/detach lists. */
  bool clear_attached = false;
};

/**
 * @brief Decode OctomapWithPose.
 *
 * Prefer FEATURE `octomap` OcTree binary leaf decode when `binary=true`;
 * otherwise interpret packed int32 triples (ix,iy,iz) → world points via
 * origin+res (common bridge layout).
 */
bool OccupiedPointsFromOctomap(
    const automsgs::msgs::map_msgs::OctomapWithPose& msg,
    std::vector<OccupiedPoint>* points, double* resolution);

/**
 * @brief Owns a PlanningScene and applies diffs / octomap / joint updates.
 */
class SceneMonitor {
 public:
  void SetScene(std::shared_ptr<PlanningScene> scene);

  void SetJointStateSubscriber(
      std::shared_ptr<execution::JointStateSubscriber> sub);

  bool ApplySceneDiff(const SceneDiff& diff);

  bool ApplyOctomap(const automsgs::msgs::map_msgs::OctomapWithPose& msg);

  /**
   * @brief PointCloud2 → occupancy (MoveIt OccupancyMapUpdater cloud lite).
   */
  bool ApplyPointCloud(const automsgs::msgs::sensor_msgs::PointCloud2& cloud,
                       double resolution = 0.05,
                       std::size_t max_points = 5000);

  bool ClearOctomap();

  bool ClearScene(bool clear_attached = false);

  std::shared_ptr<PlanningScene> Scene() const;

 private:
  mutable std::mutex mutex_;
  std::shared_ptr<PlanningScene> scene_;
  std::shared_ptr<execution::JointStateSubscriber> joint_states_;
};

}  // namespace scene
}  // namespace manipulation
}  // namespace autonomy
