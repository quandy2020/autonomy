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

//! Micro-benchmark: wall time per SlamSystem::Step on synthetic data.

#include <chrono>
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
using autonomy::localization::atla2::PointXYZI;
using autonomy::localization::atla2::SecToStamp;
using autonomy::localization::atla2::SensorData;
using autonomy::localization::atla2::SlamSystem;
using autonomy::localization::atla2::Vec3;

}  // namespace

int main(int argc, char** argv) {
  std::string config_path;
  int steps = 200;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--config" && i + 1 < argc) {
      config_path = argv[++i];
    } else if (a == "--steps" && i + 1 < argc) {
      steps = std::stoi(argv[++i]);
    }
  }
  if (config_path.empty()) {
    std::cerr << "Usage: atla2_benchmark --config <platform.yaml> [--steps N]\n";
    return 1;
  }

  Atla2Config cfg;
  if (!LoadConfig(config_path, &cfg)) {
    return 2;
  }
  SlamSystem slam;
  if (!slam.Init(cfg)) {
    return 3;
  }

  const auto t0 = std::chrono::steady_clock::now();
  const double dt = 0.01;
  int ok = 0;
  for (int k = 0; k < steps; ++k) {
    const double t = k * dt;
    SensorData data;
    data.t = SecToStamp(t);
    if (cfg.mode != FrontendMode::kVo) {
      ImuSample s;
      s.t = data.t;
      s.accel = Vec3(0.0, 0.0, 9.81);
      s.gyro = Vec3::Zero();
      data.imu.push_back(s);
      data.has_imu = true;
    }
    if (cfg.mode == FrontendMode::kVo || cfg.mode == FrontendMode::kVio ||
        cfg.mode == FrontendMode::kLivo) {
      data.has_image = true;
      data.image.t = data.t;
      data.image.width = 64;
      data.image.height = 48;
      data.image.channels = 1;
      data.image.data.assign(64 * 48, 100);
    }
    if (cfg.mode == FrontendMode::kLio || cfg.mode == FrontendMode::kLivo) {
      data.has_lidar = true;
      data.lidar.t = data.t;
      for (int i = 0; i < 32; ++i) {
        PointXYZI p;
        p.x = static_cast<float>(i);
        p.y = 0.f;
        p.z = 0.f;
        data.lidar.points.push_back(p);
      }
    }
    if (slam.Step(data)) {
      ++ok;
    }
  }
  const auto t1 = std::chrono::steady_clock::now();
  const double ms =
      std::chrono::duration<double, std::milli>(t1 - t0).count();
  std::cout << "steps=" << steps << " ok=" << ok << " total_ms=" << ms
            << " per_step_ms=" << (ms / std::max(1, steps)) << "\n";
  return ok > 0 ? 0 : 4;
}
