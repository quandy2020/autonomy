/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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

/**
 * @file test_config_loader.cpp
 * @brief Unit tests for config loader.
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
    WriteTempConfig(
        "all_disabled.yaml",
        R"(sensors:
  imu:
    - name: torso
      enable: false
      channel: /imu/torso
      port: /dev/ttyUSB0
)");
    const autodriver::Config config =
        autodriver::LoadConfig("all_disabled.yaml");
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

TEST(ConfigLoader, MergesCameraParamsFile) {
    const std::filesystem::path root =
        std::filesystem::temp_directory_path() /
        "autodriver_config_loader_params_file";
    const std::filesystem::path vendor_dir =
        root / "config" / "camera" / "orbbec";
    std::filesystem::create_directories(vendor_dir);
    {
        std::ofstream out(vendor_dir / "gemini_330.yaml");
        out << R"(model: Gemini
enable_laser: true
device_preset: Default
)";
    }
    {
        std::ofstream out(root / "config" / "hardware_test.yaml");
        out << R"(sensors:
  camera:
    - name: orbbec_color
      enable: true
      channel: /camera/color/image_raw
      backend: orbbec
      stream: color
      params_file: camera/orbbec/gemini_330.yaml
      params:
        frame_id: camera_color_optical_frame
)";
    }
    setenv("AUTODRIVER_PATH", root.string().c_str(), 1);

    const autodriver::Config config =
        autodriver::LoadConfig("hardware_test.yaml");
    ASSERT_EQ(config.sensors.size(), 1u);
    EXPECT_EQ(config.sensors[0].params.at("model"), "Gemini");
    EXPECT_EQ(config.sensors[0].params.at("enable_laser"), "true");
    EXPECT_EQ(config.sensors[0].params.at("frame_id"),
              "camera_color_optical_frame");
}

TEST(ConfigLoader, ExpandsFoldedCameraDevice) {
    const std::filesystem::path root =
        std::filesystem::temp_directory_path() /
        "autodriver_config_loader_folded_camera";
    const std::filesystem::path vendor_dir =
        root / "config" / "camera" / "realsense";
    std::filesystem::create_directories(vendor_dir);
    {
        std::ofstream out(vendor_dir / "d455.yaml");
        out << "model: D455\nemitter_enabled: true\n";
    }
    {
        std::ofstream out(root / "config" / "folded_camera.yaml");
        out << R"(sensors:
  camera:
    - name: realsense_d455
      enable: true
      backend: realsense
      params_file: camera/realsense/d455.yaml
      width: 848
      height: 480
      fps: 30
      streams:
        - stream: color
          channel: /camera/color/image_raw
          frame_id: camera_color_optical_frame
        - stream: depth
          enable: false
          channel: /camera/depth/image_rect_raw
          frame_id: camera_depth_optical_frame
      point_clouds:
        - name: points
          channel: /camera/depth/color/points
          frame_id: camera_depth_optical_frame
      imu:
        channel: /camera/imu
        frame_id: camera_imu_optical_frame
)";
    }
    setenv("AUTODRIVER_PATH", root.string().c_str(), 1);

    const autodriver::Config config =
        autodriver::LoadConfig("folded_camera.yaml");
    ASSERT_EQ(config.sensors.size(), 3u);

    const autodriver::Config::Sensor* color = nullptr;
    const autodriver::Config::Sensor* points = nullptr;
    const autodriver::Config::Sensor* imu = nullptr;
    for (const auto& sensor : config.sensors) {
        if (sensor.id == "camera/realsense_d455_color") {
            color = &sensor;
        } else if (sensor.id == "camera/realsense_d455_points") {
            points = &sensor;
        } else if (sensor.id == "imu/realsense_d455_imu") {
            imu = &sensor;
        }
    }
    ASSERT_NE(color, nullptr);
    ASSERT_NE(points, nullptr);
    ASSERT_NE(imu, nullptr);

    EXPECT_EQ(color->module, "CameraModule");
    EXPECT_EQ(color->backend, "realsense");
    ASSERT_EQ(color->channels.size(), 1u);
    EXPECT_EQ(color->channels[0], "/camera/color/image_raw");
    EXPECT_EQ(color->params.at("stream"), "color");
    EXPECT_EQ(color->params.at("width"), "848");
    EXPECT_EQ(color->params.at("model"), "D455");
    EXPECT_EQ(color->params.at("frame_id"), "camera_color_optical_frame");

    EXPECT_EQ(points->module, "PointCloudModule");
    ASSERT_EQ(points->channels.size(), 1u);
    EXPECT_EQ(points->channels[0], "/camera/depth/color/points");
    EXPECT_EQ(points->params.at("model"), "D455");

    EXPECT_EQ(imu->module, "ImuModule");
    ASSERT_EQ(imu->channels.size(), 1u);
    EXPECT_EQ(imu->channels[0], "/camera/imu");
    EXPECT_EQ(imu->params.at("frame_id"), "camera_imu_optical_frame");
    EXPECT_EQ(imu->backend, "realsense");
}

TEST(ConfigLoader, FlatCameraEntryStillWorks) {
    WriteTempConfig(
        "flat_camera.yaml",
        R"(sensors:
  camera:
    - name: realsense_d455_color
      enable: true
      channel: /camera/color/image_raw
      backend: realsense
      stream: color
      width: 640
      height: 480
      fps: 15
)");
    const autodriver::Config config =
        autodriver::LoadConfig("flat_camera.yaml");
    ASSERT_EQ(config.sensors.size(), 1u);
    EXPECT_EQ(config.sensors[0].id, "camera/realsense_d455_color");
    EXPECT_EQ(config.sensors[0].params.at("stream"), "color");
    EXPECT_EQ(config.sensors[0].params.at("width"), "640");
}
