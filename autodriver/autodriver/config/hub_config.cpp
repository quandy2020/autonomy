/*
 * Copyright 2026 Autodriver contributors
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
 * @file
 * @brief Implements hub configuration presets.
 */

#include "autodriver/config/hub_config.hpp"

namespace autodriver {

HubConfig DefaultSimulationHubConfig()
{
  HubConfig config;
  config.register_builtin_mocks = true;
  config.hub_options.publish_period = std::chrono::milliseconds(20);
  config.hub_options.alignment_window = std::chrono::milliseconds(100);
  config.drivers = {
    {"mock_imu", "imu/primary"},
    {"mock_wheel_odom", "wheel_odom/primary"},
    {"mock_lidar", "lidar/front"},
    {"mock_gps", "gps/primary"},
    {"mock_range", "range/front"},
    {"mock_camera", "camera/front"},
  };
  return config;
}

}  // namespace autodriver
