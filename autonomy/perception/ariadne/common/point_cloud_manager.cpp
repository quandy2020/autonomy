/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/perception/exploration/common/point_cloud_manager.hpp"

#include <cmath>

#include <automsgs/msgs/sensor_msgs/point_field_conversion.hpp>

#include "autonomy/perception/exploration/common/planning_env.hpp"

namespace autonomy::perception::exploration {
namespace {

using automsgs::msgs::sensor_msgs::PointCloud2;
using automsgs::msgs::sensor_msgs::PointField;

}  // namespace

PointCloudManager::PointCloudManager(const proto::ExplorationOptions& options) {
  SetOptions(options);
}

void PointCloudManager::SetOptions(const proto::ExplorationOptions& options) {
  options_ = options;
}

void PointCloudManager::Clear() {
  voxel_counts_.clear();
  planner_x_.clear();
  planner_y_.clear();
  planner_z_.clear();
  uncovered_x_.clear();
  uncovered_y_.clear();
  uncovered_z_.clear();
  covered_flags_.clear();
}

int64_t PointCloudManager::VoxelKey(int gx, int gy, int gz) {
  return (static_cast<int64_t>(gx) << 42) ^
         (static_cast<int64_t>(gy & 0xfffff) << 21) ^
         static_cast<int64_t>(gz & 0x1fffff);
}

void PointCloudManager::WorldToVoxel(double wx, double wy, double wz, int* gx,
                                     int* gy, int* gz) const {
  const double res = options_.point_cloud_voxel_m() > 0
                         ? options_.point_cloud_voxel_m()
                         : 0.2;
  *gx = static_cast<int>(std::floor(wx / res));
  *gy = static_cast<int>(std::floor(wy / res));
  *gz = static_cast<int>(std::floor(wz / res));
}

void PointCloudManager::Update(const PointCloud2& cloud, double robot_x,
                               double robot_y, double robot_z) {
  if (cloud.data().empty()) {
    return;
  }
  int ox = -1;
  int oy = -1;
  int oz = -1;
  for (int i = 0; i < cloud.fields_size(); ++i) {
    if (cloud.fields(i).name() == "x") {
      ox = static_cast<int>(cloud.fields(i).offset());
    } else if (cloud.fields(i).name() == "y") {
      oy = static_cast<int>(cloud.fields(i).offset());
    } else if (cloud.fields(i).name() == "z") {
      oz = static_cast<int>(cloud.fields(i).offset());
    }
  }
  if (ox < 0 || oy < 0 || oz < 0) {
    return;
  }
  const int count = static_cast<int>(cloud.width() * cloud.height());
  const int stride = std::max(options_.depth_stride(), 1);
  const double range = options_.sensor_range_m();
  for (int i = 0; i < count; i += stride) {
    const int idx = i * static_cast<int>(cloud.point_step());
    if (idx + oz + 8 > static_cast<int>(cloud.data().size())) {
      continue;
    }
    const float px = automsgs::msgs::sensor_msgs::readPointCloud2BufferValue<float>(
        reinterpret_cast<const unsigned char*>(cloud.data().data()) + idx + ox,
        PointField::FLOAT32);
    const float py = automsgs::msgs::sensor_msgs::readPointCloud2BufferValue<float>(
        reinterpret_cast<const unsigned char*>(cloud.data().data()) + idx + oy,
        PointField::FLOAT32);
    const float pz = automsgs::msgs::sensor_msgs::readPointCloud2BufferValue<float>(
        reinterpret_cast<const unsigned char*>(cloud.data().data()) + idx + oz,
        PointField::FLOAT32);
    if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) {
      continue;
    }
    if (std::hypot(px - robot_x, py - robot_y) > range) {
      continue;
    }
    int gx = 0;
    int gy = 0;
    int gz = 0;
    WorldToVoxel(px, py, pz, &gx, &gy, &gz);
    ++voxel_counts_[VoxelKey(gx, gy, gz)];
  }
  planner_x_.clear();
  planner_y_.clear();
  planner_z_.clear();
  const double res = options_.point_cloud_voxel_m() > 0
                         ? options_.point_cloud_voxel_m()
                         : 0.2;
  for (const auto& entry : voxel_counts_) {
    const int64_t key = entry.first;
    if (entry.second < options_.point_cloud_min_points_per_voxel()) {
      continue;
    }
    const int gz = static_cast<int>(key & 0x1fffff);
    const int gy = static_cast<int>((key >> 21) & 0xfffff);
    const int gx = static_cast<int>((key >> 42));
    planner_x_.push_back((static_cast<double>(gx) + 0.5) * res);
    planner_y_.push_back((static_cast<double>(gy) + 0.5) * res);
    planner_z_.push_back((static_cast<double>(gz) + 0.5) * res);
  }
  covered_flags_.assign(planner_x_.size(), 0);
  uncovered_x_ = planner_x_;
  uncovered_y_ = planner_y_;
  uncovered_z_ = planner_z_;
}

void PointCloudManager::MarkCovered(double x, double y, double z,
                                    double radius_m) {
  const double res = options_.point_cloud_voxel_m() > 0
                         ? options_.point_cloud_voxel_m()
                         : 0.2;
  const int r = static_cast<int>(std::ceil(radius_m / res));
  int cx = 0;
  int cy = 0;
  int cz = 0;
  WorldToVoxel(x, y, z, &cx, &cy, &cz);
  for (size_t i = 0; i < planner_x_.size(); ++i) {
    int gx = 0;
    int gy = 0;
    int gz = 0;
    WorldToVoxel(planner_x_[i], planner_y_[i], planner_z_[i], &gx, &gy, &gz);
    if (std::abs(gx - cx) <= r && std::abs(gy - cy) <= r &&
        std::abs(gz - cz) <= r) {
      covered_flags_[i] = 1;
    }
  }
}

void PointCloudManager::RefreshUncovered(const PlanningEnv& env) {
  uncovered_x_.clear();
  uncovered_y_.clear();
  uncovered_z_.clear();
  for (size_t i = 0; i < planner_x_.size(); ++i) {
    if (covered_flags_[i] != 0) {
      continue;
    }
    if (!env.IsInExplorationArea(planner_x_[i], planner_y_[i])) {
      continue;
    }
    uncovered_x_.push_back(planner_x_[i]);
    uncovered_y_.push_back(planner_y_[i]);
    uncovered_z_.push_back(planner_z_[i]);
  }
}

}  // namespace autonomy::perception::exploration
