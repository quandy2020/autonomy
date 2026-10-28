/*
 * Copyright 2026 The Openbot Authors
 */

#include "frontend/lidar/odometry/lidar_vendor.hpp"

#include <cmath>

namespace argus {
namespace vxslam {

void ProcessLidarVendor(const sensor::PointCloudXYZI& in,
                        sensor::PointCloudXYZI* out,
                        const LidarVendorOptions& opt) {
  if (out == nullptr) {
    return;
  }
  out->clear();
  out->reserve(
      in.size() / static_cast<size_t>(std::max(1, opt.point_filter_num)) + 1);
  const float blind2 = static_cast<float>(opt.blind * opt.blind);
  const int stride = std::max(1, opt.point_filter_num);
  bool need_time = opt.synthesize_time;
  if (need_time) {
    for (size_t i = 0; i < in.size(); i += static_cast<size_t>(stride)) {
      if (std::fabs(in.points[i].curvature) > 1e-6f) {
        need_time = false;
        break;
      }
    }
  }
  size_t kept = 0;
  const size_t approx = in.size() / static_cast<size_t>(stride) + 1;
  for (size_t i = 0; i < in.size(); i += static_cast<size_t>(stride)) {
    const sensor::PointType& p = in.points[i];
    const float r2 = p.x * p.x + p.y * p.y + p.z * p.z;
    if (r2 < blind2) {
      continue;
    }
    sensor::PointType q = p;
    if (need_time && approx > 1) {
      q.curvature =
          static_cast<float>(opt.scan_period * static_cast<double>(kept) /
                             static_cast<double>(approx - 1));
    }
    out->push_back(q);
    ++kept;
  }
}

}  // namespace vxslam
}  // namespace argus
