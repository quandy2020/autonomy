/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/scene/scene_monitor.hpp"

namespace autonomy {
namespace manipulation {
namespace scene {

bool OccupiedPointsFromOctomap(
    const automsgs::msgs::map_msgs::OctomapWithPose& msg,
    std::vector<OccupiedPoint>* points, double* resolution) {
  if (!points) {
    return false;
  }
  points->clear();
  const auto& om = msg.octomap();
  const double res = om.resolution() > 1e-9 ? om.resolution() : 0.05;
  if (resolution) {
    *resolution = res;
  }
  if (om.binary() || om.data_size() < 3) {
    return false;
  }
  const double ox = msg.origin().position().x();
  const double oy = msg.origin().position().y();
  const double oz = msg.origin().position().z();
  // Interpret packed int32 as (ix, iy, iz) cell indices.
  for (int i = 0; i + 2 < om.data_size(); i += 3) {
    OccupiedPoint p;
    p.x = ox + (static_cast<double>(om.data(i)) + 0.5) * res;
    p.y = oy + (static_cast<double>(om.data(i + 1)) + 0.5) * res;
    p.z = oz + (static_cast<double>(om.data(i + 2)) + 0.5) * res;
    points->push_back(p);
  }
  return !points->empty();
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
