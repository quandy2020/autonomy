/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/perception/exploration/common/terrain_height_map.hpp"

#include <cmath>
#include <cstring>

namespace autonomy::perception::exploration {

TerrainHeightMap::TerrainHeightMap(const proto::ExplorationOptions& options) {
  SetOptions(options);
}

void TerrainHeightMap::SetOptions(const proto::ExplorationOptions& options) {
  options_ = options;
}

void TerrainHeightMap::Clear() { ground_z_.clear(); }

int64_t TerrainHeightMap::CellKey(int gx, int gy) {
  return (static_cast<int64_t>(gx) << 32) ^
         static_cast<int64_t>(gy & 0xffffffff);
}

void TerrainHeightMap::WorldToCell(double wx, double wy, int* gx,
                                   int* gy) const {
  const double res = options_.far_terrain_grid_res_m() > 0
                         ? options_.far_terrain_grid_res_m()
                         : options_.costmap_resolution_m();
  *gx = static_cast<int>(std::floor(wx / res));
  *gy = static_cast<int>(std::floor(wy / res));
}

void TerrainHeightMap::UpdateFromPointCloud(
    const automsgs::msgs::sensor_msgs::PointCloud2& cloud, double robot_x,
    double robot_y, double robot_z) {
  if (cloud.data().empty()) {
    return;
  }
  int ox = -1;
  int oy = -1;
  int oz = -1;
  int oi = -1;
  for (int i = 0; i < cloud.fields_size(); ++i) {
    const auto& field = cloud.fields(i);
    if (field.name() == "x") {
      ox = static_cast<int>(field.offset());
    } else if (field.name() == "y") {
      oy = static_cast<int>(field.offset());
    } else if (field.name() == "z") {
      oz = static_cast<int>(field.offset());
    } else if (field.name() == "intensity") {
      oi = static_cast<int>(field.offset());
    }
  }
  if (ox < 0 || oy < 0) {
    return;
  }
  const int count = static_cast<int>(cloud.width() * cloud.height());
  const int stride = std::max(options_.depth_stride(), 1);
  const double range = options_.sensor_range_m();
  for (int i = 0; i < count; i += stride) {
    const int idx = i * static_cast<int>(cloud.point_step());
    if (idx + static_cast<int>(sizeof(float)) >
        static_cast<int>(cloud.data().size())) {
      continue;
    }
    float px = 0.f;
    float py = 0.f;
    float pz = 0.f;
    std::memcpy(&px, cloud.data().data() + idx + ox, sizeof(float));
    std::memcpy(&py, cloud.data().data() + idx + oy, sizeof(float));
    if (oz >= 0) {
      std::memcpy(&pz, cloud.data().data() + idx + oz, sizeof(float));
    }
    if (oi >= 0) {
      float intensity = 0.f;
      std::memcpy(&intensity, cloud.data().data() + idx + oi, sizeof(float));
      pz = intensity;
    }
    if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) {
      continue;
    }
    if (std::hypot(px - robot_x, py - robot_y) > range) {
      continue;
    }
    int gx = 0;
    int gy = 0;
    WorldToCell(px, py, &gx, &gy);
    const int64_t key = CellKey(gx, gy);
    const auto it = ground_z_.find(key);
    if (it == ground_z_.end() || pz > it->second) {
      ground_z_[key] = pz;
    }
  }
  (void)robot_z;
}

double TerrainHeightMap::QueryGroundZ(double x, double y,
                                      double fallback_z) const {
  int gx = 0;
  int gy = 0;
  WorldToCell(x, y, &gx, &gy);
  const auto it = ground_z_.find(CellKey(gx, gy));
  return it != ground_z_.end() ? static_cast<double>(it->second) : fallback_z;
}

}  // namespace autonomy::perception::exploration
