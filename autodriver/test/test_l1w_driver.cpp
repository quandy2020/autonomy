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
 * @file test_l1w_driver.cpp
 * @brief Simulate-mode unit tests for full L1-W HighLevel surface.
 */

#include "chassis/l1w/driver.hpp"

#include <memory>

#include <gtest/gtest.h>

#include "chassis/backend_registry.hpp"
#include "chassis/operational_mode.hpp"
#include "chassis/tool_command.hpp"
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>

namespace {

autodriver::chassis::ChassisDriver::SharedPtr MakeSimDriver() {
  autodriver::hardware::DriverParams params;
  params["simulate"] = "true";
  params["auto_stand"] = "true";
  params["allow_lateral"] = "true";
  params["max_vx"] = "1.0";
  params["max_vy"] = "0.5";
  params["max_wz"] = "1.0";
  return autodriver::chassis::ChassisDriver::SharedPtr(
      autodriver::chassis::CreateL1wChassisDriver("chassis/l1w", params));
}

TEST(L1wDriver, WheelMoveWithLateral) {
  auto driver = MakeSimDriver();
  ASSERT_TRUE(driver->Start());
  EXPECT_TRUE(driver->ApplyLocomotionIntent(
      autodriver::chassis::LocomotionIntent::kWheel));

  autodriver::chassis::ChassisCommand cmd;
  cmd.mutable_twist()->mutable_linear()->set_x(0.2);
  cmd.mutable_twist()->mutable_linear()->set_y(0.1);
  cmd.mutable_twist()->mutable_angular()->set_z(0.05);
  EXPECT_TRUE(driver->ApplyVelocityCommand(cmd));

  autodriver::chassis::ChassisState state;
  EXPECT_TRUE(driver->ReadChassisState(&state));
  EXPECT_NEAR(state.twist().twist().linear().y(), 0.1, 1e-6);
  driver->Stop();
}

TEST(L1wDriver, WalkUsesCrawlGaitAndTools) {
  auto driver = MakeSimDriver();
  ASSERT_TRUE(driver->Start());
  EXPECT_TRUE(driver->ApplyLocomotionIntent(
      autodriver::chassis::LocomotionIntent::kWalk));

  autodriver::chassis::ChassisCommand cmd;
  cmd.mutable_twist()->mutable_linear()->set_x(0.15);
  EXPECT_TRUE(driver->ApplyVelocityCommand(cmd));

  autodriver::chassis::ToolCommand cancel;
  cancel.name = "cancel_crawl";
  EXPECT_TRUE(driver->ApplyToolCommand(cancel));

  autodriver::chassis::ToolCommand attitude;
  attitude.name = "attitude";
  attitude.value = "0.1,-0.1,0,0.05";
  EXPECT_TRUE(driver->ApplyToolCommand(attitude));

  autodriver::chassis::ToolCommand lie;
  lie.name = "lie";
  EXPECT_TRUE(driver->ApplyToolCommand(lie));

  autodriver::chassis::ToolCommand stand;
  stand.name = "stand";
  EXPECT_TRUE(driver->ApplyToolCommand(stand));

  autodriver::chassis::ToolCommand passive;
  passive.name = "passive";
  EXPECT_TRUE(driver->ApplyToolCommand(passive));

  EXPECT_TRUE(driver->TriggerEmergencyStop());
  driver->Stop();
}

TEST(L1wDriver, ClimbShakeAndRearSquat) {
  auto driver = MakeSimDriver();
  ASSERT_TRUE(driver->Start());

  // Climb is L1-W-only (tool / default_gait), not a shared LocomotionIntent.
  autodriver::chassis::ToolCommand climb;
  climb.name = "climb";
  EXPECT_TRUE(driver->ApplyToolCommand(climb));

  autodriver::chassis::ChassisCommand cmd;
  cmd.mutable_twist()->mutable_linear()->set_x(0.1);
  EXPECT_TRUE(driver->ApplyVelocityCommand(cmd));

  autodriver::chassis::ToolCommand cancel_climb;
  cancel_climb.name = "cancel_climb";
  EXPECT_TRUE(driver->ApplyToolCommand(cancel_climb));

  autodriver::chassis::ToolCommand shake;
  shake.name = "shake_hand";
  EXPECT_TRUE(driver->ApplyToolCommand(shake));

  autodriver::chassis::ToolCommand squat;
  squat.name = "rear_squat";
  EXPECT_TRUE(driver->ApplyToolCommand(squat));

  autodriver::chassis::ToolCommand climb_once;
  climb_once.name = "climb";
  climb_once.value = "0.05,0,0";
  EXPECT_TRUE(driver->ApplyToolCommand(climb_once));

  driver->Stop();
}

TEST(L1wDriver, StandIgnoresNonZeroTwist) {
  auto driver = MakeSimDriver();
  ASSERT_TRUE(driver->Start());
  EXPECT_TRUE(driver->ApplyLocomotionIntent(
      autodriver::chassis::LocomotionIntent::kStand));
  autodriver::chassis::ChassisCommand cmd;
  cmd.mutable_twist()->mutable_linear()->set_x(0.3);
  EXPECT_TRUE(driver->ApplyVelocityCommand(cmd));
  autodriver::chassis::ChassisState state;
  ASSERT_TRUE(driver->ReadChassisState(&state));
  EXPECT_NEAR(state.twist().twist().linear().x(), 0.0, 1e-6);
  driver->Stop();
}

TEST(L1wBackendRegistry, Aliases) {
  autodriver::hardware::DriverParams params;
  params["simulate"] = "true";
  auto warmup = std::unique_ptr<autodriver::chassis::ChassisDriver>(
      autodriver::chassis::CreateL1wChassisDriver("chassis/warmup", params));
  ASSERT_NE(warmup, nullptr);

  for (const char* name : {"l1w", "genisom", "zsibot", "zsl-1w", "l1-w"}) {
    auto driver =
        autodriver::chassis::ChassisBackendRegistry::Instance().CreateDriver(
            name, "chassis/base", params);
    ASSERT_NE(driver, nullptr) << name;
    EXPECT_TRUE(driver->Start()) << name;
    driver->Stop();
  }
}

}  // namespace
