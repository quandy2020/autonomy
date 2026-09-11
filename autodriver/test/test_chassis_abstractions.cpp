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
 * @file test_chassis_abstractions.cpp
 * @brief Unit tests for locomotion / mode / safety / tool / capability.
 */

#include "chassis/capability.hpp"
#include "chassis/locomotion_model.hpp"
#include "chassis/operational_mode.hpp"
#include "chassis/safety_gate.hpp"
#include "chassis/tool_command.hpp"

#include <gtest/gtest.h>

#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>

namespace {

TEST(ChassisLocomotion, ParseAliases) {
  EXPECT_EQ(autodriver::chassis::ParseLocomotionType("diff"),
            autodriver::chassis::LocomotionType::kDifferential);
  EXPECT_EQ(autodriver::chassis::ParseLocomotionType("mecanum"),
            autodriver::chassis::LocomotionType::kOmnidirectional);
  EXPECT_EQ(autodriver::chassis::ParseLocomotionType("quadruped"),
            autodriver::chassis::LocomotionType::kLegged);
  EXPECT_EQ(autodriver::chassis::ParseLocomotionType("wheel_leg"),
            autodriver::chassis::LocomotionType::kWheelLegged);
}

TEST(ChassisCapability, JsonContainsLocomotion) {
  autodriver::chassis::CapabilityProfile profile;
  profile.chassis_id = "chassis/base";
  profile.backend = "stub";
  profile.locomotion.type = autodriver::chassis::LocomotionType::kDifferential;
  profile.tools = {"brush"};
  const std::string json =
      autodriver::chassis::CapabilityProfileToJson(profile);
  EXPECT_NE(json.find("\"locomotion\":\"differential\""), std::string::npos);
  EXPECT_NE(json.find("\"brush\""), std::string::npos);
}

TEST(ChassisOperationalMode, ArmAndEStop) {
  autodriver::chassis::OperationalModeController modes;
  EXPECT_TRUE(modes.HandleModeCommand("arm"));
  EXPECT_EQ(modes.mode(), autodriver::chassis::OperationalMode::kArmed);
  EXPECT_TRUE(modes.AllowsMotion(true));
  EXPECT_TRUE(modes.HandleModeCommand("estop"));
  EXPECT_EQ(modes.mode(), autodriver::chassis::OperationalMode::kEStop);
  EXPECT_FALSE(modes.AllowsMotion(false));
  EXPECT_TRUE(modes.HandleModeCommand("clear_estop"));
  EXPECT_EQ(modes.mode(), autodriver::chassis::OperationalMode::kIdle);
}

TEST(ChassisSafetyGate, ZerosLateralWhenNotSupported) {
  autodriver::chassis::SafetyLimits limits;
  limits.model.type = autodriver::chassis::LocomotionType::kDifferential;
  limits.model.supports_lateral = false;
  limits.model.max_linear_speed = 1.0;
  limits.model.max_angular_speed = 1.0;
  autodriver::chassis::SafetyGate gate(limits);

  autodriver::chassis::ChassisCommand in;
  in.mutable_twist()->mutable_linear()->set_x(2.0);
  in.mutable_twist()->mutable_linear()->set_y(0.5);
  in.mutable_twist()->mutable_angular()->set_z(0.1);
  const autodriver::chassis::ChassisCommand out = gate.ClampVelocity(in);
  EXPECT_NEAR(out.twist().linear().x(), 1.0, 1e-6);
  EXPECT_NEAR(out.twist().linear().y(), 0.0, 1e-6);
}

TEST(ChassisToolCommand, ParseEnableDisable) {
  autodriver::chassis::ToolCommand cmd;
  ASSERT_TRUE(autodriver::chassis::ParseToolCommand("brush=1", &cmd));
  EXPECT_EQ(cmd.name, "brush");
  EXPECT_TRUE(cmd.enable);
  ASSERT_TRUE(autodriver::chassis::ParseToolCommand("blade=off", &cmd));
  EXPECT_EQ(cmd.name, "blade");
  EXPECT_FALSE(cmd.enable);
}

}  // namespace
