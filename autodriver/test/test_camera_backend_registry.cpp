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

#include "autodriver/camera/backend_registry.hpp"
#include "autodriver/camera/orbbec/camera_driver.hpp"
#include "autodriver/camera/orbbec/device_hub.hpp"
#include "autodriver/camera/orbbec/pointcloud_driver.hpp"

TEST(CameraBackendRegistry, OrbbecRegistered) {
    auto& cameras = autodriver::camera::CameraBackendRegistry::Instance();
    auto& clouds = autodriver::camera::PointCloudBackendRegistry::Instance();
    EXPECT_TRUE(cameras.Has("orbbec"));
    EXPECT_TRUE(clouds.Has("orbbec"));
    autodriver::hardware::DriverParams params;
    if (autodriver::io::OrbbecAvailable()) {
        EXPECT_NE(cameras.Create("orbbec", "camera/orbbec", params), nullptr);
        EXPECT_NE(clouds.Create("orbbec", "camera/orbbec_points", params),
                  nullptr);
    } else {
        EXPECT_EQ(cameras.Create("orbbec", "camera/orbbec", params), nullptr);
        EXPECT_EQ(clouds.Create("orbbec", "camera/orbbec_points", params),
                  nullptr);
    }
}

TEST(OrbbecStreamKind, Parse) {
    using autodriver::hardware::orbbec::ParseStreamKind;
    using autodriver::hardware::orbbec::StreamKind;
    EXPECT_EQ(ParseStreamKind("color", StreamKind::kDepth), StreamKind::kColor);
    EXPECT_EQ(ParseStreamKind("depth", StreamKind::kColor), StreamKind::kDepth);
    EXPECT_EQ(ParseStreamKind("ir", StreamKind::kColor), StreamKind::kInfrared);
}
