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
 * @file livo_esikf_node.cpp
 * @brief ROS-free online entry: load config, init LivoOdometry (feed via API).
 *
 * Usage:
 *   livo_esikf_node [path/to/livo_esikf.yaml]
 *
 * Sensor I/O is expected from a host binary (autolink / custom). This node
 * validates config-driven modality wiring for smoke / bring-up.
 */

#include <filesystem>
#include <iostream>
#include <string>

#include "argus/common/config.hpp"
#include "argus/common/logger.hpp"
#include "argus/odometry/load_livo_config.hpp"

#ifndef ARGUS_NODE_CONFIG_ONLY
#include "argus/odometry/livo_odometry.hpp"
#endif

int main(int argc, char** argv) {
  argus::common::InitLogging(argv[0]);

  std::string yaml = "config/livo_esikf.yaml";
  bool config_only = false;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--help" || a == "-h") {
      std::cout
          << "Usage: livo_esikf_node [--config-only] [livo_esikf.yaml]\n"
          << "Host contract: single thread calls Add* then Process();\n"
          << "  check OdometryOutput.status / soft_skip for recoverable skips.\n";
      return 0;
    }
    if (a == "--config-only") {
      config_only = true;
      continue;
    }
    if (a[0] != '-') {
      yaml = a;
    }
  }

  const auto loaded = argus::odometry::LoadLivoOdometryConfig(yaml);
  if (!loaded.ok) {
    ARGUS_ERROR << "[livo_esikf_node] config error: " << loaded.error;
    return 1;
  }

  argus::common::SetDefaultConfigurationDirectory(
      std::filesystem::path(loaded.main_config_path).parent_path().string());

  ARGUS_INFO << "[livo_esikf_node] Argus " << argus::common::VersionString()
             << " config OK (" << loaded.main_config_path << ")";

  if (config_only) {
    ARGUS_INFO << "[livo_esikf_node] config smoke pass";
    return 0;
  }

#ifndef ARGUS_NODE_CONFIG_ONLY
  argus::odometry::LivoOdometry odom(loaded.config);
  ARGUS_INFO << "Host: AddLidar/AddImu/AddImage (any thread) → "
                "Process() on estimate thread only; "
                "inspect out.status / soft_skip";
  ARGUS_INFO << "[livo_esikf_node] init OK";
#else
  ARGUS_INFO << "built with ARGUS_NODE_CONFIG_ONLY; skip LivoOdometry init";
#endif
  return 0;
}
