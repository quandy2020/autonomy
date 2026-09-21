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

#include "autonomy/localization/atla2/proto/convert/sensor_packet_codec.hpp"

#include <cmath>

namespace autonomy::localization::atla2 {
namespace {

bool Vec3Eq(const Vec3& a, const Vec3& b, double eps = 1e-9) {
  return (a - b).norm() <= eps;
}

bool ImageEq(const ImageFrame& a, const ImageFrame& b) {
  return a.t == b.t && a.width == b.width && a.height == b.height &&
         a.channels == b.channels && a.frame_id == b.frame_id && a.data == b.data;
}

bool LidarEq(const LidarScan& a, const LidarScan& b) {
  if (a.t != b.t || a.t_begin != b.t_begin || a.frame_id != b.frame_id ||
      a.model != b.model || a.points.size() != b.points.size()) {
    return false;
  }
  for (size_t i = 0; i < a.points.size(); ++i) {
    const auto& pa = a.points[i];
    const auto& pb = b.points[i];
    if (std::fabs(pa.x - pb.x) > 1e-6f || std::fabs(pa.y - pb.y) > 1e-6f ||
        std::fabs(pa.z - pb.z) > 1e-6f ||
        std::fabs(pa.intensity - pb.intensity) > 1e-6f ||
        std::fabs(pa.timestamp - pb.timestamp) > 1e-9) {
      return false;
    }
  }
  return true;
}

}  // namespace

bool SensorDataToDto(const SensorData& in, SyncedSensorPacketDto* out) {
  if (!out) {
    return false;
  }
  out->t = in.t;
  out->imu = in.imu;
  out->image = in.image;
  out->image_right = in.image_right;
  out->lidar = in.lidar;
  out->gps = in.gps;
  out->baro = in.baro;
  out->optical_flow = in.optical_flow;
  out->has_image = in.has_image;
  out->has_image_right = in.image_right.width > 0;
  out->has_lidar = in.has_lidar;
  out->has_imu = in.has_imu;
  out->has_gps = in.has_gps;
  out->has_baro = in.has_baro;
  out->has_optical_flow = in.has_optical_flow;
  if (!out->frame_id.empty() && in.has_image) {
    out->frame_id = in.image.frame_id.empty() ? "cam" : in.image.frame_id;
  } else if (in.has_lidar) {
    out->frame_id = in.lidar.frame_id.empty() ? "lidar" : in.lidar.frame_id;
  }
  return true;
}

bool DtoToSensorData(const SyncedSensorPacketDto& in, SensorData* out) {
  if (!out) {
    return false;
  }
  out->t = in.t;
  out->imu = in.imu;
  out->image = in.image;
  out->image_right = in.image_right;
  out->lidar = in.lidar;
  out->gps = in.gps;
  out->baro = in.baro;
  out->optical_flow = in.optical_flow;
  out->has_image = in.has_image;
  out->has_lidar = in.has_lidar;
  out->has_imu = in.has_imu;
  out->has_gps = in.has_gps;
  out->has_baro = in.has_baro;
  out->has_optical_flow = in.has_optical_flow;
  return true;
}

bool SensorDataRoundTripEqual(const SensorData& a, const SensorData& b) {
  if (a.t != b.t || a.has_image != b.has_image || a.has_lidar != b.has_lidar ||
      a.has_imu != b.has_imu || a.has_gps != b.has_gps || a.has_baro != b.has_baro ||
      a.has_optical_flow != b.has_optical_flow) {
    return false;
  }
  if (a.imu.size() != b.imu.size()) {
    return false;
  }
  for (size_t i = 0; i < a.imu.size(); ++i) {
    if (a.imu[i].t != b.imu[i].t || !Vec3Eq(a.imu[i].accel, b.imu[i].accel) ||
        !Vec3Eq(a.imu[i].gyro, b.imu[i].gyro)) {
      return false;
    }
  }
  if (a.has_image && !ImageEq(a.image, b.image)) {
    return false;
  }
  if (a.image_right.width > 0 && !ImageEq(a.image_right, b.image_right)) {
    return false;
  }
  if (a.has_lidar && !LidarEq(a.lidar, b.lidar)) {
    return false;
  }
  if (a.has_gps) {
    if (a.gps.valid != b.gps.valid || a.gps.fix_type != b.gps.fix_type ||
        std::fabs(a.gps.hdop - b.gps.hdop) > 1e-9 || !Vec3Eq(a.gps.lla, b.gps.lla) ||
        !Vec3Eq(a.gps.position_enu, b.gps.position_enu) ||
        !Vec3Eq(a.gps.velocity_enu, b.gps.velocity_enu)) {
      return false;
    }
  }
  if (a.has_baro) {
    if (a.baro.valid != b.baro.valid ||
        std::fabs(a.baro.pressure_pa - b.baro.pressure_pa) > 1e-6 ||
        std::fabs(a.baro.temperature_c - b.baro.temperature_c) > 1e-6 ||
        std::fabs(a.baro.altitude_m - b.baro.altitude_m) > 1e-6) {
      return false;
    }
  }
  if (a.has_optical_flow) {
    if (a.optical_flow.valid != b.optical_flow.valid ||
        std::fabs(a.optical_flow.quality - b.optical_flow.quality) > 1e-9 ||
        (a.optical_flow.flow_px - b.optical_flow.flow_px).norm() > 1e-9 ||
        (a.optical_flow.velocity_xy - b.optical_flow.velocity_xy).norm() > 1e-9) {
      return false;
    }
  }
  return true;
}

}  // namespace autonomy::localization::atla2
