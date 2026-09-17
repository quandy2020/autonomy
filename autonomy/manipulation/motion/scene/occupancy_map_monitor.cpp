/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/motion/scene/occupancy_map_monitor.hpp"

#include <unordered_map>

#include <automsgs/msgs/map_msgs/octomap_with_pose.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

#include "autolink/node/reader.hpp"
#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace manipulation {
namespace scene {

void OccupancyMapMonitor::MaybeSelfFilter(
    std::vector<OccupiedPoint>* points) const {
  if (!points || !model_ || self_filter_padding_ <= 0.0 || !monitor_) {
    return;
  }
  const auto scene = monitor_->Scene();
  if (!scene) {
    return;
  }
  const core::JointState state = scene->GetCurrentState();
  std::unordered_map<std::string, core::Transform> poses;
  if (!model_->LinkTree().Compute(state, &poses) || poses.empty()) {
    return;
  }
  std::vector<OccupiedPoint> origins;
  origins.reserve(poses.size());
  for (const auto& kv : poses) {
    origins.push_back({kv.second.x, kv.second.y, kv.second.z});
  }
  const std::size_t removed = perception::FilterSelfOccupiedPoints(
      origins, self_filter_padding_, points);
  if (removed > 0) {
    AINFO << "OccupancyMapMonitor self-filter removed=" << removed
          << " padding=" << self_filter_padding_;
  }
}

bool OccupancyMapMonitor::ApplyPointCloud(
    const automsgs::msgs::sensor_msgs::PointCloud2& cloud) {
  if (!monitor_) {
    return false;
  }
  std::vector<OccupiedPoint> points;
  if (!perception::OccupiedPointsFromPointCloud2(cloud, &points,
                                                 cloud_options_)) {
    return false;
  }
  MaybeSelfFilter(&points);
  SceneDiff diff;
  diff.has_occupancy = true;
  diff.occupied = std::move(points);
  diff.occupancy_resolution = cloud_options_.resolution;
  return monitor_->ApplySceneDiff(diff);
}

bool OccupancyMapMonitor::Start(const std::shared_ptr<autolink::Node>& node,
                                const std::string& cloud_topic,
                                const std::string& octomap_topic) {
  if (!node || !monitor_) {
    return false;
  }
  bool ok = false;
  if (!cloud_topic.empty()) {
    auto reader = node->CreateReader<automsgs::msgs::sensor_msgs::PointCloud2>(
        cloud_topic,
        [this](const std::shared_ptr<automsgs::msgs::sensor_msgs::PointCloud2>&
                   msg) {
          if (msg) {
            ApplyPointCloud(*msg);
          }
        });
    if (reader) {
      cloud_reader_ = reader;
      ok = true;
      AINFO << "OccupancyMapMonitor PointCloud2 on " << cloud_topic;
    } else {
      AWARN << "OccupancyMapMonitor: PointCloud2 subscribe failed "
            << cloud_topic;
    }
  }
  if (!octomap_topic.empty()) {
    auto reader =
        node->CreateReader<automsgs::msgs::map_msgs::OctomapWithPose>(
            octomap_topic,
            [this](const std::shared_ptr<
                       automsgs::msgs::map_msgs::OctomapWithPose>& msg) {
              if (msg && monitor_) {
                monitor_->ApplyOctomap(*msg);
              }
            });
    if (reader) {
      octomap_reader_ = reader;
      ok = true;
      AINFO << "OccupancyMapMonitor OctomapWithPose on " << octomap_topic;
    } else {
      AWARN << "OccupancyMapMonitor: octomap subscribe failed "
            << octomap_topic;
    }
  }
  return ok;
}

void OccupancyMapMonitor::Stop() {
  cloud_reader_.reset();
  octomap_reader_.reset();
}

}  // namespace scene
}  // namespace manipulation
}  // namespace autonomy
