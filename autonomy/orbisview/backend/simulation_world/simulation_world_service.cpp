/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/orbisview/backend/simulation_world/simulation_world_service.hpp"

namespace autonomy {
namespace orbisview {
namespace core {

void SimulationWorldService::SetPose(double x, double y, double yaw) {
  std::lock_guard<std::mutex> lock(mutex_);
  pose_ = {x, y, yaw, true};
}

void SimulationWorldService::SetChassis(const WorldChassis& c) {
  std::lock_guard<std::mutex> lock(mutex_);
  chassis_ = c;
  chassis_.valid = true;
}

void SimulationWorldService::SetObstacles(std::vector<WorldObstacle> obs) {
  std::lock_guard<std::mutex> lock(mutex_);
  obstacles_ = std::move(obs);
}

void SimulationWorldService::SetPathPoses(std::vector<WorldPose> poses) {
  std::lock_guard<std::mutex> lock(mutex_);
  path_ = std::move(poses);
}

void SimulationWorldService::SetGoal(double x, double y, double yaw) {
  std::lock_guard<std::mutex> lock(mutex_);
  goal_ = {x, y, yaw, true};
}

void SimulationWorldService::ClearGoal() {
  std::lock_guard<std::mutex> lock(mutex_);
  goal_.valid = false;
}

std::string SimulationWorldService::ToJson() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::ostringstream oss;
  oss << "{\"robot\":";
  if (pose_.valid) {
    oss << "{\"x\":" << pose_.x << ",\"y\":" << pose_.y
        << ",\"yaw\":" << pose_.yaw << '}';
  } else {
    oss << "null";
  }
  oss << ",\"chassis\":";
  if (chassis_.valid) {
    oss << "{\"vx\":" << chassis_.vx << ",\"wz\":" << chassis_.wz
        << ",\"gear\":\"" << chassis_.gear << "\",\"throttle\":"
        << chassis_.throttle << ",\"brake\":" << chassis_.brake
        << ",\"steering\":" << chassis_.steering << ",\"driving_mode\":\""
        << chassis_.driving_mode << "\"}";
  } else {
    oss << "null";
  }
  oss << ",\"obstacles\":[";
  for (size_t i = 0; i < obstacles_.size(); ++i) {
    if (i) oss << ',';
    const auto& o = obstacles_[i];
    oss << "{\"id\":" << o.id << ",\"x\":" << o.x << ",\"y\":" << o.y
        << ",\"yaw\":" << o.yaw << ",\"length\":" << o.length
        << ",\"width\":" << o.width << ",\"type\":\"" << o.type
        << "\",\"vx\":" << o.vx << ",\"vy\":" << o.vy << '}';
  }
  oss << "],\"path\":[";
  for (size_t i = 0; i < path_.size(); ++i) {
    if (i) oss << ',';
    oss << "{\"x\":" << path_[i].x << ",\"y\":" << path_[i].y
        << ",\"yaw\":" << path_[i].yaw << '}';
  }
  oss << "],\"goal\":";
  if (goal_.valid) {
    oss << "{\"x\":" << goal_.x << ",\"y\":" << goal_.y
        << ",\"yaw\":" << goal_.yaw << '}';
  } else {
    oss << "null";
  }
  oss << '}';
  return oss.str();
}

}  // namespace core
}  // namespace orbisview
}  // namespace autonomy
