/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/motion/scene/perception.hpp"

#include <cmath>
#include <cstring>
#include <unordered_set>

namespace autonomy {
namespace manipulation {
namespace perception {
namespace {

struct FieldOff {
  int offset = -1;
  int datatype = 0;
};

FieldOff FindField(const automsgs::msgs::sensor_msgs::PointCloud2& cloud,
                   const std::string& name) {
  FieldOff out;
  for (const auto& f : cloud.fields()) {
    if (f.name() == name) {
      out.offset = static_cast<int>(f.offset());
      out.datatype = static_cast<int>(f.datatype());
      break;
    }
  }
  return out;
}

bool ReadFloat(const std::string& data, std::size_t idx, float* out) {
  if (!out || idx + 4 > data.size()) {
    return false;
  }
  std::memcpy(out, data.data() + idx, sizeof(float));
  return true;
}

}  // namespace

bool OccupiedPointsFromPointCloud2(
    const automsgs::msgs::sensor_msgs::PointCloud2& cloud,
    std::vector<scene::OccupiedPoint>* points,
    const CloudToOccupancyOptions& options) {
  if (!points) {
    return false;
  }
  points->clear();
  const auto fx = FindField(cloud, "x");
  const auto fy = FindField(cloud, "y");
  const auto fz = FindField(cloud, "z");
  // PointField FLOAT32 == 7 in sensor_msgs
  if (fx.offset < 0 || fy.offset < 0 || fz.offset < 0) {
    return false;
  }
  const std::size_t point_step =
      cloud.point_step() > 0 ? cloud.point_step() : 12;
  const std::size_t n =
      cloud.width() * std::max<uint32_t>(1, cloud.height());
  const std::string& data = cloud.data();
  const double res =
      options.resolution > 1e-6 ? options.resolution : 0.05;
  std::unordered_set<int64_t> voxels;
  voxels.reserve(std::min(n, options.max_points > 0 ? options.max_points : n));

  auto key = [res](double x, double y, double z) {
    const int64_t ix = static_cast<int64_t>(std::floor(x / res));
    const int64_t iy = static_cast<int64_t>(std::floor(y / res));
    const int64_t iz = static_cast<int64_t>(std::floor(z / res));
    return (ix * 73856093) ^ (iy * 19349663) ^ (iz * 83492791);
  };

  for (std::size_t i = 0; i < n; ++i) {
    const std::size_t base = i * point_step;
    float x = 0.f;
    float y = 0.f;
    float z = 0.f;
    if (!ReadFloat(data, base + static_cast<std::size_t>(fx.offset), &x) ||
        !ReadFloat(data, base + static_cast<std::size_t>(fy.offset), &y) ||
        !ReadFloat(data, base + static_cast<std::size_t>(fz.offset), &z)) {
      break;
    }
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    if (options.max_range > 0.0) {
      const double r2 = static_cast<double>(x) * x + static_cast<double>(y) * y +
                        static_cast<double>(z) * z;
      if (r2 > options.max_range * options.max_range) {
        continue;
      }
    }
    const int64_t k = key(x, y, z);
    if (!voxels.insert(k).second) {
      continue;
    }
    points->push_back({x, y, z});
    if (options.max_points > 0 && points->size() >= options.max_points) {
      break;
    }
  }
  return !points->empty();
}

std::size_t FilterSelfOccupiedPoints(
    const std::vector<scene::OccupiedPoint>& link_origins, double padding,
    std::vector<scene::OccupiedPoint>* points) {
  if (!points || padding <= 0.0 || link_origins.empty() || points->empty()) {
    return 0;
  }
  const double pad2 = padding * padding;
  std::size_t removed = 0;
  std::vector<scene::OccupiedPoint> kept;
  kept.reserve(points->size());
  for (const auto& p : *points) {
    bool near = false;
    for (const auto& o : link_origins) {
      const double dx = p.x - o.x;
      const double dy = p.y - o.y;
      const double dz = p.z - o.z;
      if (dx * dx + dy * dy + dz * dz <= pad2) {
        near = true;
        break;
      }
    }
    if (near) {
      ++removed;
    } else {
      kept.push_back(p);
    }
  }
  *points = std::move(kept);
  return removed;
}

}  // namespace perception
}  // namespace manipulation
}  // namespace autonomy
