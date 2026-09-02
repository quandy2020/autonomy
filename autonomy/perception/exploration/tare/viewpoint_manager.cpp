/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/perception/exploration/tare/viewpoint_manager.hpp"

#include <algorithm>
#include <cmath>

#include "autonomy/perception/exploration/common/planning_utilities.hpp"
#include "autonomy/perception/exploration/common/planning_utilities.hpp"

namespace autonomy::perception::exploration {
namespace {

constexpr double kTwoPi = 2.0 * M_PI;

}  // namespace

ViewpointManager::ViewpointManager(const proto::ExplorationOptions& options) {
  SetOptions(options);
}

void ViewpointManager::SetOptions(const proto::ExplorationOptions& options) {
  options_ = options;
  const int nx = std::max(options_.viewpoint_num_x(), 1);
  const int ny = std::max(options_.viewpoint_num_y(), 1);
  const int nz = std::max(options_.viewpoint_num_z(), 1);
  rolling_grid_.Resize(nx, ny, nz);
  lidar_model_.Configure(options_.lidar_h_res_deg() > 0 ? options_.lidar_h_res_deg()
                                                        : 2.0,
                         options_.lidar_v_res_deg() > 0 ? options_.lidar_v_res_deg()
                                                        : 5.0,
                         options_.sensor_range_m());
}

void ViewpointManager::RollIfNeeded(double robot_x, double robot_y,
                                    double robot_z) {
  if (!has_robot_) {
    has_robot_ = true;
    last_robot_x_ = robot_x;
    last_robot_y_ = robot_y;
    last_robot_z_ = robot_z;
    return;
  }
  const double res = options_.viewpoint_resolution_m() > 0
                         ? options_.viewpoint_resolution_m()
                         : 1.0;
  const double res_z = options_.viewpoint_resolution_z_m() > 0
                           ? options_.viewpoint_resolution_z_m()
                           : 0.5;
  const int dx = static_cast<int>(std::round(
      (robot_x - last_robot_x_) / res));
  const int dy = static_cast<int>(std::round(
      (robot_y - last_robot_y_) / res));
  const int dz = static_cast<int>(std::round(
      (robot_z - last_robot_z_) / res_z));
  if (dx != 0 || dy != 0 || dz != 0) {
  rolling_grid_.Roll(dx, dy, dz, nullptr, nullptr);
    last_robot_x_ = robot_x;
    last_robot_y_ = robot_y;
    last_robot_z_ = robot_z;
  }
}

void ViewpointManager::Update(const PlanningEnv& env,
                              const PointCloudManager* cloud_manager,
                              double robot_x, double robot_y, double robot_z) {
  RollIfNeeded(robot_x, robot_y, robot_z);
  BuildLattice(robot_x, robot_y, robot_z);
  FilterViewpoints(env);
  for (auto& vp : viewpoints_) {
    vp.score = ScoreViewpoint(env, cloud_manager, vp);
    vp.candidate = vp.score > 0.0 && !vp.collision && vp.line_of_sight &&
                   vp.connected;
  }
  std::sort(viewpoints_.begin(), viewpoints_.end(),
            [](const Viewpoint& a, const Viewpoint& b) {
              return a.score > b.score;
            });
}

void ViewpointManager::BuildLattice(double robot_x, double robot_y,
                                    double robot_z) {
  viewpoints_.clear();
  const int nx = std::max(options_.viewpoint_num_x(), 1);
  const int ny = std::max(options_.viewpoint_num_y(), 1);
  const int nz = std::max(options_.viewpoint_num_z(), 1);
  const double res = options_.viewpoint_resolution_m() > 0
                         ? options_.viewpoint_resolution_m()
                         : 1.0;
  const double res_z = options_.viewpoint_resolution_z_m() > 0
                           ? options_.viewpoint_resolution_z_m()
                           : 0.5;
  const int yaw_samples = std::max(options_.viewpoint_yaw_samples(), 1);
  const double half_x = 0.5 * static_cast<double>(nx) * res;
  const double half_y = 0.5 * static_cast<double>(ny) * res;
  const double half_z = 0.5 * static_cast<double>(nz) * res_z;
  int index = 0;
  for (int iz = 0; iz < nz; ++iz) {
    for (int ix = 0; ix < nx; ++ix) {
      for (int iy = 0; iy < ny; ++iy) {
        const double x =
            robot_x - half_x + (static_cast<double>(ix) + 0.5) * res;
        const double y =
            robot_y - half_y + (static_cast<double>(iy) + 0.5) * res;
        double z = robot_z - half_z + (static_cast<double>(iz) + 0.5) * res_z;
        if (options_.use_terrain_height() && terrain_map_ != nullptr) {
          z = terrain_map_->QueryGroundZ(x, y, robot_z) +
              options_.camera_height_m();
        } else if (options_.use_terrain_height()) {
          z = robot_z;
        }
        for (int iyaw = 0; iyaw < yaw_samples; ++iyaw) {
          Viewpoint vp;
          vp.x = x;
          vp.y = y;
          vp.z = z;
          vp.yaw = kTwoPi * static_cast<double>(iyaw) /
                   static_cast<double>(yaw_samples);
          vp.index = index++;
          viewpoints_.push_back(vp);
        }
      }
    }
  }
}

void ViewpointManager::FilterViewpoints(const PlanningEnv& env) {
  for (auto& vp : viewpoints_) {
    vp.collision = env.IsOccupied(vp.x, vp.y) || env.IsInNogoArea(vp.x, vp.y);
    vp.line_of_sight = LineOfSightChecker::HasLineOfSight(
        env, env.robot_x(), env.robot_y(), vp.x, vp.y);
    vp.connected = std::abs(vp.z - env.robot_z()) <
                   std::max(options_.viewpoint_resolution_z_m(), 0.5) * 2.0;
  }
}

double ViewpointManager::ScoreViewpoint(const PlanningEnv& env,
                                        const PointCloudManager* cloud,
                                        const Viewpoint& vp) const {
  if (vp.collision || !env.IsInExplorationArea(vp.x, vp.y)) {
    return 0.0;
  }
  int visible = 0;
  if (cloud != nullptr && !cloud->uncovered_x().empty()) {
    visible = lidar_model_.CountVisibleUnknown(
        vp.x, vp.y, vp.z, vp.yaw, cloud->uncovered_x(), cloud->uncovered_y(),
        cloud->uncovered_z());
  } else {
    visible = CountVisibleUnknown(env, vp);
  }
  const double dist =
      std::hypot(vp.x - env.robot_x(), vp.y - env.robot_y());
  const double range = std::max(options_.sensor_range_m(), 1.0);
  const double dist_penalty = 1.0 - std::min(dist / range, 1.0);
  return static_cast<double>(visible) * (0.5 + 0.5 * dist_penalty);
}

int ViewpointManager::CountVisibleUnknown(const PlanningEnv& env,
                                          const Viewpoint& vp) const {
  const double range = options_.sensor_range_m();
  const double res = env.costmap().getResolution();
  const int steps = static_cast<int>(range / std::max(res, 1e-3));
  const double fov = M_PI / 2.0;
  int count = 0;
  for (int i = -steps; i <= steps; ++i) {
    for (int j = 1; j <= steps; ++j) {
      const double angle =
          vp.yaw + (static_cast<double>(i) / std::max(steps, 1)) * fov;
      const double dist =
          range * static_cast<double>(j) / static_cast<double>(steps);
      const double wx = vp.x + dist * std::cos(angle);
      const double wy = vp.y + dist * std::sin(angle);
      if (!env.IsInExplorationArea(wx, wy)) {
        continue;
      }
      if (env.IsOccupied(wx, wy)) {
        break;
      }
      if (!env.IsCovered(wx, wy) && !env.IsFree(wx, wy)) {
        ++count;
      }
    }
  }
  return count;
}

void ViewpointManager::SelectTopViewpoints(int max_count) {
  selected_.clear();
  if (max_count <= 0) {
    return;
  }
  for (const auto& vp : viewpoints_) {
    if (!vp.candidate) {
      continue;
    }
    selected_.push_back(vp);
    if (static_cast<int>(selected_.size()) >= max_count) {
      break;
    }
  }
}

double ViewpointManager::TotalGain() const {
  double sum = 0.0;
  for (const auto& vp : selected_) {
    sum += vp.score;
  }
  return sum;
}

bool ViewpointManager::GetShortestPath(
    double from_x, double from_y, double to_x, double to_y,
    std::vector<automsgs::msgs::geometry_msgs::Point>* path) const {
  if (path == nullptr) {
    return false;
  }
  PlanningEnv env(options_);
  const double len =
      PathPlanner::Plan(env, from_x, from_y, to_x, to_y, path);
  return std::isfinite(len) && !path->empty();
}

}  // namespace autonomy::perception::exploration
