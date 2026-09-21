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

//! Offline smoke runner for Atla2 SLAM (synthetic IMU + optional lidar).
//!
//! Usage:
//!   autonomy.localization.atla2_offline --config <platform.yaml> [--steps N]

#include <cmath>
#include <iostream>
#include <string>

#include "autonomy/localization/atla2/common/time.hpp"
#include "autonomy/localization/atla2/pipeline/slam_system.hpp"

namespace {

using autonomy::localization::atla2::Atla2Config;
using autonomy::localization::atla2::FrontendMode;
using autonomy::localization::atla2::ImageFrame;
using autonomy::localization::atla2::ImuSample;
using autonomy::localization::atla2::LidarScan;
using autonomy::localization::atla2::LoadConfig;
using autonomy::localization::atla2::OdometryResult;
using autonomy::localization::atla2::PointXYZI;
using autonomy::localization::atla2::SecToStamp;
using autonomy::localization::atla2::SensorData;
using autonomy::localization::atla2::Se3Translation;
using autonomy::localization::atla2::SlamSystem;
using autonomy::localization::atla2::ToString;
using autonomy::localization::atla2::Vec3;

void PrintUsage(const char* argv0) {
  std::cerr << "Usage: " << argv0
            << " --config <platform.yaml> [--steps N]\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::string config_path;
  int steps = 100;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--config" && i + 1 < argc) {
      config_path = argv[++i];
    } else if (a == "--steps" && i + 1 < argc) {
      steps = std::stoi(argv[++i]);
    } else if (a == "-h" || a == "--help") {
      PrintUsage(argv[0]);
      return 0;
    }
  }
  if (config_path.empty()) {
    PrintUsage(argv[0]);
    return 1;
  }

  Atla2Config cfg;
  if (!LoadConfig(config_path, &cfg)) {
    std::cerr << "Failed to load config: " << config_path << "\n";
    return 2;
  }

  SlamSystem slam;
  if (!slam.Init(cfg)) {
    std::cerr << "SlamSystem::Init failed\n";
    return 3;
  }

  std::cout << "Atla2 offline: mode=" << ToString(cfg.mode)
            << " steps=" << steps << "\n";

  const double dt = 0.01;
  for (int k = 0; k < steps; ++k) {
    const double t = k * dt;
    SensorData data;
    data.t = SecToStamp(t);

    // Synthetic IMU: gravity + small yaw rate (not used by VO).
    if (cfg.mode != FrontendMode::kVo) {
      for (int j = 0; j < 5; ++j) {
        ImuSample s;
        s.t = SecToStamp(t - 0.04 + j * 0.01);
        s.accel = Vec3(0.0, 0.0, 9.81);
        s.gyro = Vec3(0.0, 0.0, 0.05);
        data.imu.push_back(s);
      }
      data.has_imu = true;
    }

    if (cfg.mode == FrontendMode::kVo || cfg.mode == FrontendMode::kVio ||
        cfg.mode == FrontendMode::kLivo) {
      data.has_image = true;
      data.image.t = data.t;
      data.image.width = 640;
      data.image.height = 480;
      data.image.channels = 1;
      data.image.data.assign(640 * 480, 128);
    }
    if (cfg.mode == FrontendMode::kLio || cfg.mode == FrontendMode::kLivo) {
      data.has_lidar = true;
      data.lidar.t = data.t;
      for (int i = 0; i < 100; ++i) {
        const float ang = static_cast<float>(i) * 0.06f;
        PointXYZI p;
        p.x = 5.f * std::cos(ang);
        p.y = 5.f * std::sin(ang);
        p.z = 0.1f * (i % 10);
        p.intensity = 1.f;
        data.lidar.points.push_back(p);
      }
    }

    if (!slam.Step(data)) {
      std::cerr << "Step failed at k=" << k << "\n";
      return 4;
    }
  }

  OdometryResult odom;
  if (!slam.GetOdometry(&odom)) {
    std::cerr << "No odometry\n";
    return 5;
  }

  const auto p = Se3Translation(odom.pose);
  std::cout << "Final pose t=[" << p.x() << ", " << p.y() << ", " << p.z()
            << "] map_points=" << odom.local_map.size()
            << " keyframes=" << slam.map()->Keyframes().size() << "\n";
  return 0;
}
