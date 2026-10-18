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

/**
 * @file offline_livo.cpp
 * @brief Offline smoke: YAML load, LivoOdometry init, optional synthetic LO.
 *
 * Usage:
 *   offline_livo [--config-only|--synth] [path/to/livo_esikf.yaml]
 *
 * `--synth` feeds planar body clouds (LO, no IMU/bag) and optionally saves a map.
 */

#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>

#include "argus/common/config.hpp"
#include "argus/common/logger.hpp"
#include "argus/odometry/load_livo_config.hpp"

#ifndef ARGUS_OFFLINE_CONFIG_ONLY
#include "argus/odometry/livo_odometry.hpp"
#include "argus/odometry/odometry_base.hpp"
#include "argus/sensor/point_types.hpp"
#endif

namespace {

const char* SlamModeName(argus::frontend::SlamMode m) {
  using argus::frontend::SlamMode;
  switch (m) {
    case SlamMode::ONLY_LO:
      return "LO";
    case SlamMode::kOnlyLio:
      return "LIO";
    case SlamMode::LIVO:
      return "LIVO";
    case SlamMode::ONLY_VIO:
      return "VIO";
    default:
      return "?";
  }
}

#ifndef ARGUS_OFFLINE_CONFIG_ONLY
argus::sensor::PointCloudXYZI::Ptr MakePlaneCloud(double x_shift) {
  auto cloud = std::make_shared<argus::sensor::PointCloudXYZI>();
  auto push = [&](float x, float y, float z, float t_ms) {
    argus::sensor::PointType p;
    p.x = x;
    p.y = y;
    p.z = z;
    p.intensity = 1.f;
    p.curvature = t_ms;  // offset time within scan (ms), for LO sync
    cloud->push_back(p);
  };
  for (int i = 0; i < 40; ++i) {
    for (int j = -10; j <= 10; ++j) {
      push(static_cast<float>(x_shift + 0.25 * i),
           static_cast<float>(0.25 * j), 1.0f, static_cast<float>(i));
    }
  }
  // Second wall for non-degenerate planes.
  for (int i = 0; i < 40; ++i) {
    for (int k = 0; k < 8; ++k) {
      push(static_cast<float>(x_shift + 0.25 * i), 3.0f,
           static_cast<float>(0.3 * k), static_cast<float>(40 + i));
    }
  }
  return cloud;
}

int RunSynthetic(argus::odometry::LivoOdometryConfig cfg) {
  // LO-only synthetic path: no IMU / image / bag.
  cfg.slam_mode = argus::frontend::ONLY_LO;
  cfg.enable_lidar = true;
  cfg.enable_imu = false;
  cfg.enable_image = false;
  cfg.enable_mapping = true;
  if (cfg.mapping.session_id.empty()) {
    cfg.mapping.session_id = "synth";
  }
  if (cfg.mapping_output_dir.empty()) {
    cfg.mapping_output_dir = "argus_synth_map_out";
  }

  argus::odometry::LivoOdometry odom(cfg);
  std::cout << "[offline_livo] synth LO start (mapping → "
            << cfg.mapping_output_dir << ")\n";

  int processed = 0;
  int map_updates = 0;
  for (int f = 0; f < 12; ++f) {
    const double t = 0.1 * (f + 1);
    odom.AddLidar(MakePlaneCloud(0.15 * f), t);
    argus::odometry::OdometryOutput out;
    if (!odom.Process(&out)) {
      continue;
    }
    ++processed;
    if (out.map_updated) {
      ++map_updates;
    }
  }

  if (processed == 0) {
    std::cerr << "[offline_livo] synth: no package processed\n";
    return 1;
  }

  std::filesystem::remove_all(cfg.mapping_output_dir);
  if (!odom.SaveBuiltMap()) {
    std::cerr << "[offline_livo] synth: SaveBuiltMap failed\n";
    return 1;
  }
  if (!std::filesystem::exists(cfg.mapping_output_dir + "/map.xyz")) {
    std::cerr << "[offline_livo] synth: map.xyz missing\n";
    return 1;
  }

  std::cout << "[offline_livo] synth pass processed=" << processed
            << " map_updates=" << map_updates
            << " kf=" << (odom.map_builder() ? odom.map_builder()->keyframe_count()
                                             : 0)
            << "\n";
  return 0;
}
#endif

}  // namespace

int main(int argc, char** argv) {
  argus::common::InitLogging(argv[0]);
  std::string yaml;
  bool config_only = false;
  bool synth = false;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--config-only" || a == "--smoke" || a == "-s") {
      config_only = true;
    } else if (a == "--synth") {
      synth = true;
    } else if (a == "--help" || a == "-h") {
      std::cout
          << "Usage: offline_livo [--config-only|--synth] [livo_esikf.yaml]\n"
          << "  --config-only   load/validate YAML only\n"
          << "  --synth         synthetic LO frames + MapBuilder save (no bag)\n";
      return 0;
    } else if (yaml.empty()) {
      yaml = a;
    }
  }
  if (yaml.empty()) {
    yaml = "config/livo_esikf.yaml";
  }

  const auto loaded = argus::odometry::LoadLivoOdometryConfig(yaml);
  if (!loaded.ok) {
    std::cerr << "[offline_livo] config error: " << loaded.error << "\n";
    return 1;
  }

  const auto& cfg = loaded.config;
  std::cout << "[offline_livo] loaded " << loaded.main_config_path << "\n"
            << "  version=" << argus::common::VersionString() << "\n"
            << "  mode=" << SlamModeName(cfg.slam_mode)
            << " lidar=" << cfg.enable_lidar << " imu=" << cfg.enable_imu
            << " img=" << cfg.enable_image << "\n"
            << "  voxel_res=" << cfg.voxel.max_voxel_size_
            << " max_iter=" << cfg.voxel.max_iterations_ << "\n"
            << "  degeneracy=" << cfg.enable_degeneracy
            << " gating=" << cfg.enable_confidence_gating
            << " intensity=" << cfg.enable_intensity_assist << "\n"
            << "  loop=" << cfg.loop_closure.enable
            << " mapping=" << cfg.enable_mapping
            << " reloc=" << cfg.relocalization.enable << "\n";

  if (config_only) {
    std::cout << "[offline_livo] config smoke pass\n";
    return 0;
  }

#ifndef ARGUS_OFFLINE_CONFIG_ONLY
  if (synth) {
    return RunSynthetic(cfg);
  }
  argus::odometry::LivoOdometry odom(cfg);
  std::cout << "[offline_livo] LivoOdometry initialized OK\n";
  std::cout << "[offline_livo] init smoke pass "
               "(use --synth for frame-level smoke without bag)\n";
#else
  (void)synth;
  std::cout << "[offline_livo] built with ARGUS_OFFLINE_CONFIG_ONLY; "
               "skip LivoOdometry init\n";
#endif
  return 0;
}
