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

#include "autonomy/manipulation/execution/joint_state_subscriber.hpp"
#include "autonomy/manipulation/scene/planning_scene.hpp"

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
 * @brief Decode OctomapWithPose without liboctomap.
 *
 * Non-binary: packed int32 triples (ix,iy,iz) → world points via origin+res.
 * Binary: ignored (requires liboctomap); returns empty.
 *
 * @param[in] msg Octomap with pose header / resolution.
 * @param[out] points Decoded occupied centers (cleared then filled).
 * @param[out] resolution Map resolution in meters when decoding succeeds.
 * @return true if non-binary data was decoded (may still yield zero points).
 */
bool OccupiedPointsFromOctomap(
    const automsgs::msgs::map_msgs::OctomapWithPose& msg,
    std::vector<OccupiedPoint>* points, double* resolution);

/**
 * @brief Owns a PlanningScene and applies diffs / octomap / joint updates.
 *
 * Optionally mirrors JointStateSubscriber into the scene when a subscriber is
 * set; ApplySceneDiff remains the primary external mutation path.
 */
class SceneMonitor {
 public:
  /** @brief Install the scene instance subsequent Apply* calls mutate. */
  void SetScene(std::shared_ptr<PlanningScene> scene);

  /** @brief Optional joint-state source mirrored into the scene. */
  void SetJointStateSubscriber(
      std::shared_ptr<execution::JointStateSubscriber> sub);

  /**
   * @brief Apply @p diff to the configured scene under the monitor lock.
   * @return false if no scene is set; true otherwise.
   */
  bool ApplySceneDiff(const SceneDiff& diff);

  /**
   * @brief Decode @p msg and SetOccupiedPoints on the scene.
   * @return false if no scene is set or decode fails; true on success.
   */
  bool ApplyOctomap(const automsgs::msgs::map_msgs::OctomapWithPose& msg);

  /**
   * @brief Clear occupancy samples (MoveIt clear_octomap analogue).
   * @return false if no scene is set.
   */
  bool ClearOctomap();

  /**
   * @brief Clear world objects and optionally attached objects.
   * @param[in] clear_attached Also drop attachments when true.
   * @return false if no scene is set.
   */
  bool ClearScene(bool clear_attached = false);

  /** @brief @return Shared pointer to the configured PlanningScene (may be null). */
  std::shared_ptr<PlanningScene> Scene() const;

 private:
  mutable std::mutex mutex_;
  std::shared_ptr<PlanningScene> scene_;
  std::shared_ptr<execution::JointStateSubscriber> joint_states_;
};

}  // namespace scene
}  // namespace manipulation
}  // namespace autonomy
