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

#include <gtest/gtest.h>

#include <string>

#include "autodriver/config.hpp"
#include "autodriver/sensor_traits.hpp"

TEST(Config, DetectsDuplicateSensorId) {
    autodriver::Config config;
    autodriver::Config::Sensor a;
    a.id = "imu/a";
    autodriver::Config::Sensor b;
    b.id = "imu/a";
    config.sensors = {a, b};
    EXPECT_TRUE(config.HasDuplicateId());
}

TEST(Config, UniqueIdsAreOk) {
    autodriver::Config config;
    autodriver::Config::Sensor a;
    a.id = "imu/a";
    autodriver::Config::Sensor b;
    b.id = "gps/b";
    config.sensors = {a, b};
    EXPECT_FALSE(config.HasDuplicateId());
}

TEST(Config, DefaultChannels) {
    using autodriver::SensorType;
    EXPECT_EQ(autodriver::ResolveChannel("", "imu/front", SensorType::kImu),
              "/imu/front");
    EXPECT_EQ(autodriver::ResolveChannel("", "gps/rear", SensorType::kGps),
              "/gps/rear");
    EXPECT_EQ(autodriver::ResolveChannel("", "lidar/front", SensorType::kLidar2d),
              "/lidar/front/scan");
    EXPECT_EQ(autodriver::ResolveChannel("", "lidar/velo", SensorType::kLidar3d),
              "/lidar/velo/points");
    EXPECT_EQ(autodriver::ResolveChannel("", "range/front",
                                        SensorType::kRangeFinder),
              "/range/front");
    EXPECT_EQ(autodriver::ResolveChannel("", "camera/front", SensorType::kCamera),
              "/camera/front/image_raw");
    EXPECT_EQ(autodriver::ResolveChannel("", "camera/front", SensorType::kCamera,
                                        "depth"),
              "/camera/front/depth/image_raw");
    EXPECT_EQ(autodriver::ResolveChannel("/x", "imu/a", SensorType::kImu), "/x");
}

TEST(Config, MatchDeviceHexVendor) {
    autodriver::DeviceMatch rule;
    rule.vendor = "0x8086";
    autodriver::DeviceMatch observed;
    observed.vendor = "8086";
    EXPECT_TRUE(autodriver::MatchDevice(observed, rule));
}

TEST(Config, MatchDevice) {
    autodriver::DeviceMatch rule;
    autodriver::DeviceMatch observed;
    observed.subsystem = "tty";
    observed.device = "/dev/ttyUSB0";
    EXPECT_FALSE(autodriver::MatchDevice(observed, rule));
    rule.subsystem = "tty";
    rule.device = "/dev/ttyUSB0";
    EXPECT_TRUE(autodriver::MatchDevice(observed, rule));
    observed.device = "/dev/ttyUSB1";
    EXPECT_FALSE(autodriver::MatchDevice(observed, rule));
}
