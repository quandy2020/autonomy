/*
 * Copyright 2026 Autodriver contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

#include "autodriver/config_loader.hpp"
#include "autodriver/sensor_traits.hpp"

namespace {

std::filesystem::path WriteTempConfig(const std::string& basename,
                                      const std::string& yaml) {
    const std::filesystem::path root =
        std::filesystem::temp_directory_path() /
        "autodriver_config_loader_test";
    const std::filesystem::path config_dir = root / "config";
    std::filesystem::create_directories(config_dir);
    const std::filesystem::path path = config_dir / basename;
    std::ofstream out(path);
    out << yaml;
    out.close();
    setenv("AUTODRIVER_PATH", root.string().c_str(), 1);
    return path;
}

}  // namespace

TEST(ConfigLoader, SkipsDisabledDevices) {
    unsetenv("AUTODRIVER_PATH");
    const autodriver::Config config = autodriver::LoadConfig();
    EXPECT_TRUE(config.sensors.empty());
}

TEST(ConfigLoader, LoadsChannelArray) {
    WriteTempConfig(
        "channels_test.yaml",
        R"(sensors:
  imu:
    - name: torso
      enable: true
      channel:
        - /imu/torso
        - /imu/torso/raw
      port: /dev/ttyUSB0
      baudrate: 115200
)");
    const autodriver::Config config =
        autodriver::LoadConfig("channels_test.yaml");
    ASSERT_EQ(config.sensors.size(), 1u);
    EXPECT_EQ(config.sensors[0].id, "imu/torso");
    ASSERT_EQ(config.sensors[0].channels.size(), 2u);
    EXPECT_EQ(config.sensors[0].channels[0], "/imu/torso");
    EXPECT_EQ(config.sensors[0].channels[1], "/imu/torso/raw");
}

TEST(ConfigLoader, LoadsSingleChannelString) {
    WriteTempConfig(
        "single_channel_test.yaml",
        R"(sensors:
  gps:
    - name: main
      enable: true
      channel: /gps/fix
      port: /dev/ttyUSB2
)");
    const autodriver::Config config =
        autodriver::LoadConfig("single_channel_test.yaml");
    ASSERT_EQ(config.sensors.size(), 1u);
    ASSERT_EQ(config.sensors[0].channels.size(), 1u);
    EXPECT_EQ(config.sensors[0].channels[0], "/gps/fix");
}

TEST(ConfigLoader, ResolvesDefaultLidarChannels) {
    autodriver::Config::Sensor lidar2d;
    lidar2d.id = "lidar/front";
    EXPECT_EQ(autodriver::ResolveChannel("", lidar2d.id,
                                         autodriver::SensorType::kLidar2d),
              "/lidar/front/scan");
    autodriver::Config::Sensor lidar3d;
    lidar3d.id = "lidar/velo";
    EXPECT_EQ(autodriver::ResolveChannel("", lidar3d.id,
                                         autodriver::SensorType::kLidar3d),
              "/lidar/velo/points");
}
