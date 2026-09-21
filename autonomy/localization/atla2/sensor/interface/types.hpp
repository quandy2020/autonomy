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

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "autonomy/localization/atla2/common/types.hpp"

namespace autonomy::localization::atla2 {

enum class SensorKind {
  kCamera = 0,
  kImu,
  kLidar,
  kGps,
  kBarometer,
  kOpticalFlow,
};

struct ImuSample {
  TimeStamp t = kInvalidTime;
  Vec3 accel = Vec3::Zero();  // m/s^2
  Vec3 gyro = Vec3::Zero();   // rad/s
};

struct ImageFrame {
  TimeStamp t = kInvalidTime;
  int width = 0;
  int height = 0;
  int channels = 1;
  //! Row-major grayscale or RGB8 buffer (empty if zero-copy external).
  std::vector<uint8_t> data;
  std::string frame_id = "cam";
};

struct LidarScan {
  TimeStamp t = kInvalidTime;  // end-of-scan
  TimeStamp t_begin = kInvalidTime;
  PointCloud points;
  std::string frame_id = "lidar";
  std::string model;  // livox / ouster / velodyne / generic
};

struct GpsSample {
  TimeStamp t = kInvalidTime;
  Vec3 lla = Vec3::Zero();  // lat deg, lon deg, alt m
  Vec3 position_enu = Vec3::Zero();
  Vec3 velocity_enu = Vec3::Zero();
  int fix_type = 0;  // 0=none, 2=2D, 3=3D
  double hdop = 99.0;
  bool valid = false;
};

struct BaroSample {
  TimeStamp t = kInvalidTime;
  double pressure_pa = 0.0;
  double temperature_c = 0.0;
  double altitude_m = 0.0;  // relative / QNH
  bool valid = false;
};

struct OpticalFlowSample {
  TimeStamp t = kInvalidTime;
  Vec2 flow_px = Vec2::Zero();       // pixels / frame
  Vec2 velocity_xy = Vec2::Zero();   // m/s in body xy (if scaled)
  double quality = 0.0;              // [0,1]
  bool valid = false;
};

//! Bundled multi-sensor packet for one frontend step.
struct SensorData {
  TimeStamp t = kInvalidTime;
  std::vector<ImuSample> imu;
  ImageFrame image;
  ImageFrame image_right;
  LidarScan lidar;
  GpsSample gps;
  BaroSample baro;
  OpticalFlowSample optical_flow;
  bool has_image = false;
  bool has_lidar = false;
  bool has_imu = false;
  bool has_gps = false;
  bool has_baro = false;
  bool has_optical_flow = false;
};

}  // namespace autonomy::localization::atla2
