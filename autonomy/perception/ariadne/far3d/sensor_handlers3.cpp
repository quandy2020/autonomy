/*
 * Copyright 2026 The Openbot Authors
 */
#include "autonomy/perception/exploration/far3d/sensor_handlers3.hpp"
#include <cmath>
#include <cstdint>
#include <queue>
#include <unordered_map>
#include <vector>
#include <automsgs/msgs/sensor_msgs/point_field_conversion.hpp>

namespace autonomy::perception::exploration::far3d {

namespace {

using automsgs::msgs::sensor_msgs::PointField;

}  // namespace

MapHandler3::MapHandler3(const proto::ExplorationOptions& options) {
  SetOptions(options);
}

void MapHandler3::SetOptions(const proto::ExplorationOptions& options) {
  options_ = options;
}

void MapHandler3::Clear() {
  obs_cells_.clear();
  terrain_z_.clear();
}

int64_t MapHandler3::Key(int gx, int gy, int gz) {
  return (static_cast<int64_t>(gx) << 42) ^
         (static_cast<int64_t>(gy & 0xfffff) << 21) ^
         static_cast<int64_t>(gz & 0x1fffff);
}

void MapHandler3::Voxel(double x, double y, double z, int* gx, int* gy,
                        int* gz) const {
  const double res = options_.far3d_voxel_dim_m() > 0 ? options_.far3d_voxel_dim_m()
                                                      : 0.2;
  *gx = static_cast<int>(std::floor(x / res));
  *gy = static_cast<int>(std::floor(y / res));
  *gz = static_cast<int>(std::floor(z / res));
}

void MapHandler3::InsertObsCloud(
    const automsgs::msgs::sensor_msgs::PointCloud2& cloud) {
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
    int gx = 0;
    int gy = 0;
    int gz = 0;
    Voxel(px, py, pz, &gx, &gy, &gz);
    obs_cells_[Key(gx, gy, gz)] = 1;
    terrain_z_[Key(gx, gy, 0)] = pz;
  }
}

void MapHandler3::InsertFreeCloud(
    const automsgs::msgs::sensor_msgs::PointCloud2& cloud) {
  (void)cloud;
}

void MapHandler3::RemoveObsNear(double x, double y, double z, double radius_m) {
  const double res = options_.far3d_voxel_dim_m() > 0 ? options_.far3d_voxel_dim_m()
                                                      : 0.2;
  int cx = 0;
  int cy = 0;
  int cz = 0;
  Voxel(x, y, z, &cx, &cy, &cz);
  const int r = static_cast<int>(std::ceil(radius_m / res));
  for (int dx = -r; dx <= r; ++dx) {
    for (int dy = -r; dy <= r; ++dy) {
      for (int dz = -r; dz <= r; ++dz) {
        obs_cells_.erase(Key(cx + dx, cy + dy, cz + dz));
      }
    }
  }
}

bool MapHandler3::HasObsBetween(double ax, double ay, double az, double bx,
                                double by, double bz) const {
  const int steps = static_cast<int>(
      std::hypot(bx - ax, by - ay, bz - az) /
      std::max(options_.far3d_voxel_dim_m(), 0.1));
  for (int i = 1; i < steps; ++i) {
    const double t = static_cast<double>(i) / std::max(steps, 1);
    const double x = ax + t * (bx - ax);
    const double y = ay + t * (by - ay);
    const double z = az + t * (bz - az);
    int gx = 0;
    int gy = 0;
    int gz = 0;
    const_cast<MapHandler3*>(this)->Voxel(x, y, z, &gx, &gy, &gz);
    if (obs_cells_.find(Key(gx, gy, gz)) != obs_cells_.end()) {
      return true;
    }
  }
  return false;
}

double MapHandler3::TerrainHeightAt(double x, double y, double z_hint) const {
  int gx = 0;
  int gy = 0;
  int gz = 0;
  const_cast<MapHandler3*>(this)->Voxel(x, y, z_hint, &gx, &gy, &gz);
  const auto it = terrain_z_.find(Key(gx, gy, 0));
  return it != terrain_z_.end() ? static_cast<double>(it->second) : z_hint;
}


namespace {

using automsgs::msgs::sensor_msgs::PointField;

}  // namespace

ScanHandler3::ScanHandler3(const proto::ExplorationOptions& options) {
  SetOptions(options);
}

void ScanHandler3::SetOptions(const proto::ExplorationOptions& options) {
  options_ = options;
}

void ScanHandler3::ResetFrame() { cells_.clear(); }

int64_t ScanHandler3::Key(int gx, int gy, int gz) {
  return (static_cast<int64_t>(gx) << 42) ^
         (static_cast<int64_t>(gy & 0xfffff) << 21) ^
         static_cast<int64_t>(gz & 0x1fffff);
}

void ScanHandler3::Voxel(double x, double y, double z, int* gx, int* gy,
                         int* gz) const {
  const double res =
      options_.far_scan_voxel_m() > 0 ? options_.far_scan_voxel_m() : 0.2;
  *gx = static_cast<int>(std::floor(x / res));
  *gy = static_cast<int>(std::floor(y / res));
  *gz = static_cast<int>(std::floor(z / res));
}

void ScanHandler3::MarkScan(
    const automsgs::msgs::sensor_msgs::PointCloud2& cloud, double robot_x,
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
    int gx = 0;
    int gy = 0;
    int gz = 0;
    Voxel(px, py, pz, &gx, &gy, &gz);
    cells_[Key(gx, gy, gz)] |= kScan;
    MarkRay(robot_x, robot_y, robot_z, px, py, pz);
  }
}

void ScanHandler3::MarkRay(double from_x, double from_y, double from_z,
                           double to_x, double to_y, double to_z) {
  const int steps = static_cast<int>(std::hypot(to_x - from_x, to_y - from_y,
                                                to_z - from_z) /
                                     std::max(options_.far_scan_voxel_m(), 0.1));
  for (int i = 0; i < steps; ++i) {
    const double t = static_cast<double>(i) / std::max(steps, 1);
    int gx = 0;
    int gy = 0;
    int gz = 0;
    Voxel(from_x + t * (to_x - from_x), from_y + t * (to_y - from_y),
          from_z + t * (to_z - from_z), &gx, &gy, &gz);
    cells_[Key(gx, gy, gz)] |= kRay;
  }
}

void ScanHandler3::ExtractDynamicCloud(std::vector<double>* x,
                                       std::vector<double>* y,
                                       std::vector<double>* z) const {
  if (x == nullptr || y == nullptr || z == nullptr) {
    return;
  }
  x->clear();
  y->clear();
  z->clear();
  const double res =
      options_.far_scan_voxel_m() > 0 ? options_.far_scan_voxel_m() : 0.2;
  for (const auto& entry : cells_) {
    if ((entry.second & kScan) && (entry.second & kRay)) {
      const int gz = static_cast<int>(entry.first & 0x1fffff);
      const int gy = static_cast<int>((entry.first >> 21) & 0xfffff);
      const int gx = static_cast<int>((entry.first >> 42));
      x->push_back((gx + 0.5) * res);
      y->push_back((gy + 0.5) * res);
      z->push_back((gz + 0.5) * res);
    }
  }
}


TerrainPlanner3::TerrainPlanner3(const proto::ExplorationOptions& options) {
  SetOptions(options);
}

void TerrainPlanner3::SetOptions(const proto::ExplorationOptions& options) {
  options_ = options;
  const double res = options_.far_terrain_grid_res_m() > 0
                         ? options_.far_terrain_grid_res_m()
                         : 0.25;
  const double range = options_.far_local_planner_range_m() > 0
                           ? options_.far_local_planner_range_m()
                           : 5.0;
  grid_size_ = std::max(10, static_cast<int>(std::ceil(2.0 * range / res)));
  grid_.assign(static_cast<size_t>(grid_size_ * grid_size_), 0);
}

void TerrainPlanner3::SetCenter(double x, double y) {
  center_x_ = x;
  center_y_ = y;
}

void TerrainPlanner3::SetObstacles(const MapHandler3& map) {
  const double res = options_.far_terrain_grid_res_m() > 0
                         ? options_.far_terrain_grid_res_m()
                         : 0.25;
  const double half = 0.5 * grid_size_ * res;
  for (int iy = 0; iy < grid_size_; ++iy) {
    for (int ix = 0; ix < grid_size_; ++ix) {
      const double x = center_x_ - half + (ix + 0.5) * res;
      const double y = center_y_ - half + (iy + 0.5) * res;
      const size_t idx = static_cast<size_t>(iy * grid_size_ + ix);
      grid_[idx] = map.HasObsBetween(x, y, 0.0, x, y, 1.0) ? 1 : 0;
    }
  }
}

bool TerrainPlanner3::IsOccupied(double x, double y) const {
  const double res = options_.far_terrain_grid_res_m() > 0
                         ? options_.far_terrain_grid_res_m()
                         : 0.25;
  const double half = 0.5 * grid_size_ * res;
  const int ix = static_cast<int>((x - (center_x_ - half)) / res);
  const int iy = static_cast<int>((y - (center_y_ - half)) / res);
  if (ix < 0 || iy < 0 || ix >= grid_size_ || iy >= grid_size_) {
    return true;
  }
  return grid_[static_cast<size_t>(iy * grid_size_ + ix)] != 0;
}

bool TerrainPlanner3::Plan(
    double from_x, double from_y, double to_x, double to_y,
    std::vector<automsgs::msgs::geometry_msgs::Point>* path) const {
  if (path == nullptr) {
    return false;
  }
  path->clear();
  const double res = options_.far_terrain_grid_res_m() > 0
                         ? options_.far_terrain_grid_res_m()
                         : 0.25;
  const double half = 0.5 * grid_size_ * res;
  auto to_cell = [&](double x, double y) {
    return std::make_pair(
        static_cast<int>((x - (center_x_ - half)) / res),
        static_cast<int>((y - (center_y_ - half)) / res));
  };
  const auto start = to_cell(from_x, from_y);
  const auto goal = to_cell(to_x, to_y);
  if (start.first < 0 || start.second < 0 || start.first >= grid_size_ ||
      start.second >= grid_size_ || goal.first < 0 || goal.second < 0 ||
      goal.first >= grid_size_ || goal.second >= grid_size_) {
    return false;
  }
  const int n = grid_size_ * grid_size_;
  std::vector<double> g(static_cast<size_t>(n),
                        std::numeric_limits<double>::infinity());
  std::vector<int> parent(static_cast<size_t>(n), -1);
  auto ind = [&](int x, int y) { return y * grid_size_ + x; };
  using Entry = std::pair<double, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  const int s = ind(start.first, start.second);
  const int t = ind(goal.first, goal.second);
  g[static_cast<size_t>(s)] = 0.0;
  open.push({0.0, s});
  const int dx[4] = {1, -1, 0, 0};
  const int dy[4] = {0, 0, 1, -1};
  while (!open.empty()) {
    const Entry cur = open.top();
    open.pop();
    if (cur.second == t) {
      break;
    }
    if (cur.first > g[static_cast<size_t>(cur.second)]) {
      continue;
    }
    const int cx = cur.second % grid_size_;
    const int cy = cur.second / grid_size_;
    for (int k = 0; k < 4; ++k) {
      const int nx = cx + dx[k];
      const int ny = cy + dy[k];
      if (nx < 0 || ny < 0 || nx >= grid_size_ || ny >= grid_size_) {
        continue;
      }
      const int ni = ind(nx, ny);
      if (grid_[static_cast<size_t>(ni)] != 0) {
        continue;
      }
      const double tentative = g[static_cast<size_t>(cur.second)] + res;
      if (tentative >= g[static_cast<size_t>(ni)]) {
        continue;
      }
      g[static_cast<size_t>(ni)] = tentative;
      parent[static_cast<size_t>(ni)] = cur.second;
      open.push({tentative, ni});
    }
  }
  if (!std::isfinite(g[static_cast<size_t>(t)])) {
    return false;
  }
  for (int cur = t; cur != -1; cur = parent[static_cast<size_t>(cur)]) {
    automsgs::msgs::geometry_msgs::Point pt;
    const int cx = cur % grid_size_;
    const int cy = cur / grid_size_;
    pt.set_x(center_x_ - half + (cx + 0.5) * res);
    pt.set_y(center_y_ - half + (cy + 0.5) * res);
    path->push_back(pt);
  }
  std::reverse(path->begin(), path->end());
  return !path->empty();
}


ViewpointExtension3::ViewpointExtension3(
    const proto::ExplorationOptions& options) {
  SetOptions(options);
}

void ViewpointExtension3::SetOptions(const proto::ExplorationOptions& options) {
  options_ = options;
}

automsgs::msgs::geometry_msgs::PoseStamped ViewpointExtension3::Extend(
    const automsgs::msgs::geometry_msgs::PoseStamped& waypoint,
    const CtNode& corner, const MapHandler3& map) const {
  if (!options_.far_viewpoint_extend() ||
      corner.convexity != ContourConvexity::kConvex) {
    return waypoint;
  }
  automsgs::msgs::geometry_msgs::PoseStamped out = waypoint;
  const double range = options_.far_local_planner_range_m() > 0
                           ? options_.far_local_planner_range_m()
                           : 5.0;
  const double step = options_.far3d_voxel_dim_m() > 0
                          ? options_.far3d_voxel_dim_m()
                          : 0.2;
  double wx = waypoint.pose().position().x();
  double wy = waypoint.pose().position().y();
  double wz = waypoint.pose().position().z();
  for (double d = step; d <= range; d += step) {
    const double nx = corner.position.x() - corner.surface_dir_x * d;
    const double ny = corner.position.y() - corner.surface_dir_y * d;
    const double nz = corner.position.z();
    if (map.HasObsBetween(corner.position.x(), corner.position.y(),
                          corner.position.z(), nx, ny, nz)) {
      break;
    }
    wx = nx;
    wy = ny;
    wz = nz;
  }
  out.mutable_pose()->mutable_position()->set_x(wx);
  out.mutable_pose()->mutable_position()->set_y(wy);
  out.mutable_pose()->mutable_position()->set_z(wz);
  return out;
}

}  // namespace autonomy::perception::exploration::far3d
