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

//! Atla2 online entry (ROS-free). Push sensors + Step() in a loop.
//!
//! Usage:
//!   autonomy.localization.atla2_node --config <platform.yaml> [--steps N]

#include <iostream>
#include <string>

#include "autonomy/localization/atla2/common/time.hpp"
#include "autonomy/localization/atla2/pipeline/slam_system.hpp"

namespace {

using autonomy::localization::atla2::Atla2Config;
using autonomy::localization::atla2::LoadConfig;
using autonomy::localization::atla2::OdometryResult;
using autonomy::localization::atla2::Se3Translation;
using autonomy::localization::atla2::SlamSystem;

void PrintUsage(const char* argv0) {
  std::cerr << "Usage: " << argv0
            << " --config <platform.yaml> [--steps N]\n"
            << "  Online node stub: runs N empty Step() polls (use Push* from\n"
            << "  an adapter / Autolink bridge when wiring sensors).\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::string config_path;
  int steps = 0;
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

  std::cout << "atla2_node ready (platform=" << cfg.platform_name
            << "). Waiting for Push* / Step.\n";

  for (int k = 0; k < steps; ++k) {
    if (!slam.Step()) {
      // No synced packet yet — normal when sensors are idle.
    }
  }

  OdometryResult odom;
  if (slam.GetOdometry(&odom)) {
    const auto p = Se3Translation(odom.pose);
    std::cout << "Last odom t=[" << p.x() << ", " << p.y() << ", " << p.z()
              << "]\n";
  }
  return 0;
}
