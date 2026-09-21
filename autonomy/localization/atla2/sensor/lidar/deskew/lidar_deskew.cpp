/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autonomy/localization/atla2/sensor/lidar/deskew/lidar_deskew.hpp"

#include "Eigen/Geometry"

#include "autonomy/localization/atla2/common/time.hpp"

namespace autonomy::localization::atla2 {

LidarScan LidarDeskew::Compensate(const LidarScan& in, const std::vector<ImuSample>& imu,
                                  const Vec3& ba, const Vec3& bg) const {
  LidarScan out = in;
  if (imu.size() < 2 || in.points.empty()) {
    return out;
  }

  const TimeStamp t_end = in.t;
  TimeStamp t_begin = in.t_begin;
  if (t_begin == kInvalidTime) {
    t_begin = t_end - SecToStamp(0.1);
  }

  // Pose at end: identity; propagate backward/forward from begin.
  SE3 T_end = Se3Identity();
  Vec3 v = Vec3::Zero();
  // Build pose at end from begin using full IMU in [t_begin, t_end]
  std::vector<ImuSample> segment;
  for (const auto& s : imu) {
    if (s.t >= t_begin && s.t <= t_end) {
      segment.push_back(s);
    }
  }
  SE3 T_begin = Se3Identity();
  Vec3 v_begin = Vec3::Zero();
  ImuPropagate(segment, ba, bg, Vec3(0, 0, -9.81), &T_begin, &v_begin);
  // T_end is identity in deskew frame; T_pt_end = T_end * inv(T_pt)
  // Approximate: use linear slerp ratio by point time.
  const double span = StampToSec(t_end - t_begin);
  if (span < 1e-4) {
    return out;
  }

  const Mat33 R_end = Se3RotationMatrix(T_begin);  // motion over scan
  const Vec3 t_end_vec = Se3Translation(T_begin);

  for (auto& p : out.points) {
    double ratio = 1.0;
    if (p.timestamp > 0.0) {
      // absolute seconds → relative to begin
      const double t_pt = p.timestamp;
      const double t0 = StampToSec(t_begin);
      ratio = (t_pt - t0) / span;
      if (ratio < 0.0) {
        ratio = 0.0;
      }
      if (ratio > 1.0) {
        ratio = 1.0;
      }
    }
    // Interpolate motion from point time → end: (1-ratio) of total motion inverse
    const Vec3 dp = (1.0 - ratio) * t_end_vec;
    const Eigen::AngleAxisd A(Se3Rotation(T_begin));
    const double ang = A.angle() * (1.0 - ratio);
    Mat33 R = Mat33::Identity();
    if (std::abs(ang) > 1e-8) {
      R = Eigen::AngleAxisd(ang, A.axis()).toRotationMatrix();
    }
    const Vec3 pw = R * Vec3(p.x, p.y, p.z) + dp;
    p.x = static_cast<float>(pw.x());
    p.y = static_cast<float>(pw.y());
    p.z = static_cast<float>(pw.z());
  }
  (void)R_end;
  (void)v;
  (void)T_end;
  return out;
}

}  // namespace autonomy::localization::atla2
