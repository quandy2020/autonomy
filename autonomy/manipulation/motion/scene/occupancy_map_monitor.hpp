/*
 * Copyright 2026 The Openbot Authors
 *
 * Occupancy map monitor lite (MoveIt occupancy_map_monitor subset).
 */

#pragma once

#include <memory>
#include <string>

#include "autolink/node/node.hpp"
#include "autonomy/manipulation/model/simple_robot_model.hpp"
#include "autonomy/manipulation/motion/scene/perception.hpp"
#include "autonomy/manipulation/motion/scene/scene_monitor.hpp"

namespace autonomy {
namespace manipulation {
namespace scene {

/**
 * @brief Subscribes PointCloud2 / OctomapWithPose → SceneMonitor occupancy.
 *
 * Full MoveIt uses OccupancyMapUpdater plugins + Octomap; this lite keeps
 * voxel-downsampled points as OccupiedPoint[] for collision checks, with
 * optional robot self-filter around link origins.
 */
class OccupancyMapMonitor {
 public:
  void SetSceneMonitor(std::shared_ptr<SceneMonitor> monitor) {
    monitor_ = std::move(monitor);
  }

  void SetCloudOptions(perception::CloudToOccupancyOptions options) {
    cloud_options_ = std::move(options);
  }

  /** @brief Optional model for link-origin self-filter. */
  void SetRobotModel(std::shared_ptr<const core::SimpleRobotModel> model) {
    model_ = std::move(model);
  }

  /**
   * @brief Sphere radius (m) around each link origin to drop cloud points.
   * 0 disables self-filter.
   */
  void SetSelfFilterPadding(double padding) { self_filter_padding_ = padding; }

  /**
   * @brief Subscribe cloud + octomap topics on @p node.
   * @return true if at least one subscription succeeded.
   */
  bool Start(const std::shared_ptr<autolink::Node>& node,
             const std::string& cloud_topic,
             const std::string& octomap_topic);

  void Stop();

  /** @brief Inject PointCloud2 without a live subscription. */
  bool ApplyPointCloud(
      const automsgs::msgs::sensor_msgs::PointCloud2& cloud);

 private:
  void MaybeSelfFilter(std::vector<OccupiedPoint>* points) const;

  std::shared_ptr<SceneMonitor> monitor_;
  std::shared_ptr<const core::SimpleRobotModel> model_;
  perception::CloudToOccupancyOptions cloud_options_;
  double self_filter_padding_ = 0.0;
  std::shared_ptr<void> cloud_reader_;
  std::shared_ptr<void> octomap_reader_;
};

}  // namespace scene
}  // namespace manipulation
}  // namespace autonomy
