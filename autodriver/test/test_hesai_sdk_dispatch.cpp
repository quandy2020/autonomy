/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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

#include <memory>

#include "autodriver/lidar/hesai/sdk_driver.hpp"
#include "autodriver/lidar/hesai/udp_driver.hpp"
#include "gtest/gtest.h"

namespace autodriver {
namespace hardware {
namespace {

TEST(HesaiDispatch, WantSdkExplicitFlags) {
    DriverParams p;
    EXPECT_FALSE(WantHesaiSdk(p));

    p["use_sdk"] = "true";
    EXPECT_TRUE(WantHesaiSdk(p));

    p.clear();
    p["parser"] = "sdk";
    EXPECT_TRUE(WantHesaiSdk(p));

    p.clear();
    p["parser"] = "ros2";
    EXPECT_TRUE(WantHesaiSdk(p));
}

TEST(HesaiDispatch, WantSdkByModel) {
    DriverParams xt;
    xt["model"] = "XT32";
    EXPECT_FALSE(WantHesaiSdk(xt));

    DriverParams ot;
    ot["model"] = "OT128";
    EXPECT_TRUE(WantHesaiSdk(ot));

    DriverParams jt;
    jt["model"] = "JT128";
    EXPECT_TRUE(WantHesaiSdk(jt));
}

TEST(HesaiDispatch, CreateUdpDefault) {
    DriverParams p;
    p["model"] = "XT32";
    p["data_port"] = "2368";
    std::unique_ptr<SensorDriver> drv(CreateHesaiLidarDriver("lidar/hesai", p));
    ASSERT_NE(drv, nullptr);
    EXPECT_EQ(drv->GetSensorType(), SensorType::kLidar3d);
}

TEST(HesaiDispatch, CreateSdkWithoutMacro) {
#ifndef AUTODRIVER_HAVE_HESAI_SDK
    DriverParams p;
    p["use_sdk"] = "true";
    EXPECT_EQ(CreateHesaiLidarDriver("lidar/hesai", p), nullptr);
#endif
}

}  // namespace
}  // namespace hardware
}  // namespace autodriver
