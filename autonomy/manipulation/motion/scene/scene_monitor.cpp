/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/motion/scene/scene_monitor.hpp"

#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "autonomy/manipulation/motion/scene/perception.hpp"

#ifdef AUTONOMY_HAS_OCTOMAP
#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>
#endif

namespace autonomy {
namespace manipulation {
namespace scene {
namespace {

bool OccupiedPointsFromCellIndices(
    const automsgs::msgs::map_msgs::OctomapWithPose& msg,
    std::vector<OccupiedPoint>* points, double* resolution) {
  const auto& om = msg.octomap();
  const double res = om.resolution() > 1e-9 ? om.resolution() : 0.05;
  if (resolution) {
    *resolution = res;
  }
  if (om.data_size() < 3) {
    return false;
  }
  const double ox = msg.origin().position().x();
  const double oy = msg.origin().position().y();
  const double oz = msg.origin().position().z();
  points->clear();
  for (int i = 0; i + 2 < om.data_size(); i += 3) {
    OccupiedPoint p;
    p.x = ox + (static_cast<double>(om.data(i)) + 0.5) * res;
    p.y = oy + (static_cast<double>(om.data(i + 1)) + 0.5) * res;
    p.z = oz + (static_cast<double>(om.data(i + 2)) + 0.5) * res;
    points->push_back(p);
  }
  return !points->empty();
}

#ifdef AUTONOMY_HAS_OCTOMAP
std::string PackOctomapBytes(
    const automsgs::msgs::map_msgs::Octomap& om) {
  // Prefer one-byte-per-int32 (common int8[] → int32 bridge).
  bool fits_byte = true;
  for (int v : om.data()) {
    if (v < -128 || v > 255) {
      fits_byte = false;
      break;
    }
  }
  std::string bytes;
  if (fits_byte) {
    bytes.reserve(static_cast<std::size_t>(om.data_size()));
    for (int v : om.data()) {
      bytes.push_back(static_cast<char>(static_cast<unsigned char>(v & 0xff)));
    }
    return bytes;
  }
  // Fallback: little-endian 4 bytes per int32.
  bytes.reserve(static_cast<std::size_t>(om.data_size()) * 4u);
  for (int32_t v : om.data()) {
    const auto u = static_cast<uint32_t>(v);
    bytes.push_back(static_cast<char>(u & 0xff));
    bytes.push_back(static_cast<char>((u >> 8) & 0xff));
    bytes.push_back(static_cast<char>((u >> 16) & 0xff));
    bytes.push_back(static_cast<char>((u >> 24) & 0xff));
  }
  return bytes;
}

bool OccupiedPointsFromOcTreeBinary(
    const automsgs::msgs::map_msgs::OctomapWithPose& msg,
    std::vector<OccupiedPoint>* points, double* resolution) {
  const auto& om = msg.octomap();
  if (!om.binary() || om.data_size() == 0 || !points) {
    return false;
  }
  const double res = om.resolution() > 1e-9 ? om.resolution() : 0.05;
  if (resolution) {
    *resolution = res;
  }
  const std::string bytes = PackOctomapBytes(om);
  std::stringstream ss(bytes);
  std::unique_ptr<octomap::OcTree> tree;
  try {
    if (!om.id().empty() && om.id() != "OcTree") {
      std::unique_ptr<octomap::AbstractOcTree> abs(
          octomap::AbstractOcTree::createTree(om.id(), res));
      if (!abs || !abs->readBinaryData(ss)) {
        return false;
      }
      tree.reset(dynamic_cast<octomap::OcTree*>(abs.release()));
      if (!tree) {
        return false;
      }
    } else {
      tree = std::make_unique<octomap::OcTree>(res);
      if (!tree->readBinaryData(ss)) {
        return false;
      }
    }
  } catch (...) {
    return false;
  }

  const double ox = msg.origin().position().x();
  const double oy = msg.origin().position().y();
  const double oz = msg.origin().position().z();
  // Origin pose: apply translation only (lite; full MoveIt uses Transform).
  points->clear();
  constexpr std::size_t kMaxLeaves = 200000;
  for (auto it = tree->begin_leafs(), end = tree->end_leafs(); it != end;
       ++it) {
    if (!tree->isNodeOccupied(*it)) {
      continue;
    }
    OccupiedPoint p;
    p.x = ox + it.getX();
    p.y = oy + it.getY();
    p.z = oz + it.getZ();
    points->push_back(p);
    if (points->size() >= kMaxLeaves) {
      break;
    }
  }
  if (resolution) {
    *resolution = tree->getResolution();
  }
  return !points->empty();
}
#endif

}  // namespace

bool OccupiedPointsFromOctomap(
    const automsgs::msgs::map_msgs::OctomapWithPose& msg,
    std::vector<OccupiedPoint>* points, double* resolution) {
  if (!points) {
    return false;
  }
#ifdef AUTONOMY_HAS_OCTOMAP
  if (msg.octomap().binary()) {
    if (OccupiedPointsFromOcTreeBinary(msg, points, resolution)) {
      return true;
    }
  }
#endif
  // Cell-index packing fallback (works for binary and non-binary bridges).
  return OccupiedPointsFromCellIndices(msg, points, resolution);
}

void SceneMonitor::SetScene(std::shared_ptr<PlanningScene> scene) {
  std::lock_guard<std::mutex> lock(mutex_);
  scene_ = std::move(scene);
}

void SceneMonitor::SetJointStateSubscriber(
    std::shared_ptr<execution::JointStateSubscriber> sub) {
  std::lock_guard<std::mutex> lock(mutex_);
  joint_states_ = std::move(sub);
  if (joint_states_ && scene_) {
    joint_states_->SetScene(scene_);
  }
}

bool SceneMonitor::ApplySceneDiff(const SceneDiff& diff) {
  std::shared_ptr<PlanningScene> scene;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    scene = scene_;
  }
  if (!scene) {
    return false;
  }
  if (diff.clear_world) {
    scene->ClearWorldObjects();
  }
  if (diff.clear_attached) {
    scene->ClearAttachedObjects();
  }
  for (const auto& id : diff.remove) {
    scene->RemoveCollisionObject(id);
  }
  for (const auto& obj : diff.add) {
    scene->AddCollisionObject(obj);
  }
  for (const auto& id : diff.detach) {
    scene->DetachObject(id);
  }
  for (const auto& att : diff.attach) {
    scene->AttachObject(att);
  }
  if (diff.has_robot_state) {
    scene->SetCurrentState(diff.robot_state);
  }
  if (diff.clear_occupancy) {
    scene->ClearOccupiedPoints();
  }
  if (diff.has_occupancy) {
    scene->SetOccupiedPoints(diff.occupied, diff.occupancy_resolution);
  }
  return true;
}

bool SceneMonitor::ApplyOctomap(
    const automsgs::msgs::map_msgs::OctomapWithPose& msg) {
  std::vector<OccupiedPoint> points;
  double resolution = 0.05;
  if (!OccupiedPointsFromOctomap(msg, &points, &resolution)) {
    return false;
  }
  SceneDiff diff;
  diff.has_occupancy = true;
  diff.occupied = std::move(points);
  diff.occupancy_resolution = resolution;
  return ApplySceneDiff(diff);
}

bool SceneMonitor::ApplyPointCloud(
    const automsgs::msgs::sensor_msgs::PointCloud2& cloud, double resolution,
    std::size_t max_points) {
  perception::CloudToOccupancyOptions opt;
  opt.resolution = resolution;
  opt.max_points = max_points;
  std::vector<OccupiedPoint> points;
  if (!perception::OccupiedPointsFromPointCloud2(cloud, &points, opt)) {
    return false;
  }
  SceneDiff diff;
  diff.has_occupancy = true;
  diff.occupied = std::move(points);
  diff.occupancy_resolution = resolution;
  return ApplySceneDiff(diff);
}

bool SceneMonitor::ClearOctomap() {
  SceneDiff diff;
  diff.clear_occupancy = true;
  return ApplySceneDiff(diff);
}

bool SceneMonitor::ClearScene(bool clear_attached) {
  SceneDiff diff;
  diff.clear_world = true;
  diff.clear_attached = clear_attached;
  return ApplySceneDiff(diff);
}

std::shared_ptr<PlanningScene> SceneMonitor::Scene() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return scene_;
}

}  // namespace scene
}  // namespace manipulation
}  // namespace autonomy
