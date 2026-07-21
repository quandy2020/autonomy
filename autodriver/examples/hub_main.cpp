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
 * @brief Standalone HAL hub process using Lua configuration.
 */

#include <chrono>
#include <csignal>
#include <atomic>
#include <iostream>
#include <string>
#include <thread>

#include "autodriver/app/sensor_manager.hpp"
#include "autodriver/bridge/autonomy/hub_config_loader.hpp"
#include "autodriver/types/imu_sample.hpp"
#include "autodriver/types/sensor_type.hpp"

namespace {

std::atomic<bool> g_running{true};

void HandleSignal(int)
{
  g_running = false;
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string configuration_directory;
  std::string configuration_file = "driver/autodriver.lua";
  if (argc > 1) {
    configuration_directory = argv[1];
  }
  if (argc > 2) {
    configuration_file = argv[2];
  }

  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);

  try {
    autodriver::HubConfig config =
      autodriver::bridge::LoadHubConfigFromDirectory(
        configuration_directory, configuration_file);
    autodriver::SensorManager manager(std::move(config));

    std::atomic<int> frames{0};
    manager.SetAlignedCallback([&](const autodriver::AlignedSnapshot & snapshot) {
      ++frames;
      const auto * imu = snapshot.Get<autodriver::ImuSample>(
        autodriver::SensorType::kImu);
      if (imu && frames.load() % 50 == 0) {
        std::cout << "[autodriver_hub] frame=" << frames.load()
                  << " sensors=" << snapshot.samples.size()
                  << " imu_wz=" << imu->angular_velocity[2]
                  << std::endl;
      }
    });

    if (!manager.Initialize()) {
      std::cerr << "SensorManager init failed\n";
      for (const auto & err : manager.init_errors()) {
        std::cerr << "  driver: " << err << "\n";
      }
      return 1;
    }

    if (!manager.Start()) {
      std::cerr << "SensorManager start failed\n";
      return 1;
    }

    std::cout << "autodriver_hub running (Ctrl+C to stop)\n";
    while (g_running.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    manager.Stop();
    std::cout << "autodriver_hub stopped, frames=" << frames.load() << "\n";
  } catch (const std::exception & ex) {
    std::cerr << "autodriver_hub failed: " << ex.what() << "\n";
    return 1;
  }

  return 0;
}
