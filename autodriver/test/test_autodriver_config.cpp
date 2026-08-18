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

#include "autodriver/config/autodriver_config.hpp"

TEST(AutodriverConfig, DetectsDuplicateSensorId) {
    autodriver::AutodriverConfig config;
    autodriver::SensorConfig a;
    a.sensor_id = "imu/a";
    autodriver::SensorConfig b;
    b.sensor_id = "imu/a";
    config.sensors = {a, b};
    EXPECT_TRUE(autodriver::HasDuplicateSensorId(config));
}

TEST(AutodriverConfig, UniqueIdsAreOk) {
    autodriver::AutodriverConfig config;
    autodriver::SensorConfig a;
    a.sensor_id = "imu/a";
    autodriver::SensorConfig b;
    b.sensor_id = "gps/b";
    config.sensors = {a, b};
    EXPECT_FALSE(autodriver::HasDuplicateSensorId(config));
}
