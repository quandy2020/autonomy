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

#include "chassis/backend_registry.hpp"
#include "chassis/stub/driver.hpp"

#include <gtest/gtest.h>

#include "autodriver/driver_params.hpp"
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>

namespace {

TEST(ChassisBackendRegistry, StubCreateAndDrive) {
  ASSERT_TRUE(autodriver::chassis::ChassisBackendRegistry::Instance().HasBackend(
      "stub"));
  ASSERT_TRUE(autodriver::chassis::ChassisBackendRegistry::Instance().HasBackend(
      "sim"));

  autodriver::hardware::DriverParams params;
  auto driver =
      autodriver::chassis::ChassisBackendRegistry::Instance().CreateDriver(
          "stub", "chassis/test", params);
  ASSERT_NE(driver, nullptr);
  EXPECT_TRUE(driver->Start());
  EXPECT_TRUE(driver->IsRunning());

  autodriver::chassis::ChassisCommand cmd;
  cmd.mutable_twist()->mutable_linear()->set_x(0.5);
  cmd.mutable_twist()->mutable_angular()->set_z(0.1);
  EXPECT_TRUE(driver->ApplyVelocityCommand(cmd));

  autodriver::chassis::ChassisState state;
  EXPECT_TRUE(driver->ReadChassisState(&state));
  ASSERT_TRUE(state.has_twist());
  ASSERT_TRUE(state.twist().has_twist());
  EXPECT_NEAR(state.twist().twist().linear().x(), 0.5, 1e-6);
  EXPECT_NEAR(state.twist().twist().angular().z(), 0.1, 1e-6);
  EXPECT_TRUE(state.motion_enabled());

  EXPECT_TRUE(driver->TriggerEmergencyStop());
  EXPECT_TRUE(driver->ReadChassisState(&state));
  EXPECT_FALSE(state.motion_enabled());

  driver->Stop();
  EXPECT_FALSE(driver->IsRunning());
}

TEST(ChassisBackendRegistry, UnknownBackend) {
  autodriver::hardware::DriverParams params;
  auto driver =
      autodriver::chassis::ChassisBackendRegistry::Instance().CreateDriver(
          "no_such_vendor", "chassis/x", params);
  EXPECT_EQ(driver, nullptr);
}

}  // namespace
