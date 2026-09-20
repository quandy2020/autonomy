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
 * @file test_jetauto_kinematics.cpp
 * @brief Unit tests for JetAuto IK and RRC motor packet framing.
 */

#include "chassis/backend_registry.hpp"
#include "chassis/jetauto/driver.hpp"
#include "chassis/jetauto/kinematics.hpp"
#include "chassis/jetauto/rrc_protocol.hpp"

#include <cmath>
#include <cstring>
#include <memory>

#include <gtest/gtest.h>

#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>

namespace {

TEST(JetAutoKinematics, ForwardOnlyMecanumSymmetric) {
  autodriver::chassis::jetauto::ChassisGeometry geo;
  const auto rps = autodriver::chassis::jetauto::TwistToMotorRps(
      0.5, 0.0, 0.0, geo,
      autodriver::chassis::jetauto::DriveMode::kMecanum);
  const double expected =
      autodriver::chassis::jetauto::SpeedToRps(0.5, geo.wheel_diameter);
  EXPECT_NEAR(rps[0], expected, 1e-6);
  EXPECT_NEAR(rps[1], expected, 1e-6);
  EXPECT_NEAR(rps[2], -expected, 1e-6);
  EXPECT_NEAR(rps[3], -expected, 1e-6);
}

TEST(JetAutoKinematics, DifferentialZerosVy) {
  autodriver::chassis::jetauto::ChassisGeometry geo;
  const auto with_vy = autodriver::chassis::jetauto::TwistToMotorRps(
      0.3, 0.2, 0.0, geo,
      autodriver::chassis::jetauto::DriveMode::kDifferential);
  const auto no_vy = autodriver::chassis::jetauto::TwistToMotorRps(
      0.3, 0.0, 0.0, geo,
      autodriver::chassis::jetauto::DriveMode::kDifferential);
  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(with_vy[i], no_vy[i], 1e-6);
  }
}

TEST(JetAutoKinematics, ParseDriveMode) {
  using autodriver::chassis::jetauto::DriveMode;
  using autodriver::chassis::jetauto::ParseDriveMode;
  EXPECT_EQ(ParseDriveMode("mecanum", false), DriveMode::kMecanum);
  EXPECT_EQ(ParseDriveMode("omni", false), DriveMode::kMecanum);
  EXPECT_EQ(ParseDriveMode("differential", false), DriveMode::kDifferential);
  EXPECT_EQ(ParseDriveMode("diff", true), DriveMode::kDifferential);
  EXPECT_EQ(ParseDriveMode("", true), DriveMode::kDifferential);
  EXPECT_EQ(ParseDriveMode("", false), DriveMode::kMecanum);
}

TEST(JetAutoRrcProtocol, MultiMotorSingleWheel) {
  // Multi-set with N=1: payload len = 5*1+2 = 7.
  const std::uint8_t ids[1] = {1};
  const float rps[1] = {-1.0f};
  const auto packet =
      autodriver::chassis::jetauto::BuildSetMotorsPacket(ids, rps, 1);
  ASSERT_EQ(packet.size(), 12u);  // 2+1+1+7+1
  EXPECT_EQ(packet[0], 0xAA);
  EXPECT_EQ(packet[1], 0x55);
  EXPECT_EQ(packet[2], 3);
  EXPECT_EQ(packet[3], 7);
  EXPECT_EQ(packet[4], 0x01);
  EXPECT_EQ(packet[5], 1);
  EXPECT_EQ(packet[6], 1);
  float decoded = 0.0f;
  std::memcpy(&decoded, &packet[7], sizeof(decoded));
  EXPECT_FLOAT_EQ(decoded, -1.0f);
  EXPECT_EQ(packet.back(), autodriver::chassis::jetauto::ChecksumCrc8(
                               3, 7, &packet[4]));
}

TEST(JetAutoRrcProtocol, MultiMotorFourWheels) {
  const float rps[4] = {-1.0f, 2.0f, 0.0f, 0.5f};
  const auto packet =
      autodriver::chassis::jetauto::BuildSetFourMotorsPacket(rps);
  // len = 5*4+2 = 22; total = 2+1+1+22+1 = 27
  ASSERT_EQ(packet.size(), 27u);
  EXPECT_EQ(packet[0], 0xAA);
  EXPECT_EQ(packet[1], 0x55);
  EXPECT_EQ(packet[2], 3);
  EXPECT_EQ(packet[3], 22);
  EXPECT_EQ(packet[4], 0x01);
  EXPECT_EQ(packet[5], 4);
  EXPECT_EQ(packet[6], 1);
  float decoded = 0.0f;
  std::memcpy(&decoded, &packet[7], sizeof(decoded));
  EXPECT_FLOAT_EQ(decoded, -1.0f);

  const std::uint8_t* payload = &packet[4];
  const std::uint8_t crc = autodriver::chassis::jetauto::ChecksumCrc8(
      3, 22, payload);
  EXPECT_EQ(packet.back(), crc);
}

TEST(JetAutoDriver, SimulateStartAndCmdVel) {
  autodriver::hardware::DriverParams params;
  params["simulate"] = "true";
  params["drive_mode"] = "mecanum";
  auto driver = std::unique_ptr<autodriver::chassis::ChassisDriver>(
      autodriver::chassis::CreateJetAutoChassisDriver("chassis/jetauto",
                                                      params));
  ASSERT_NE(driver, nullptr);
  ASSERT_TRUE(driver->Start());
  EXPECT_TRUE(driver->IsRunning());

  autodriver::chassis::ChassisCommand cmd;
  cmd.mutable_twist()->mutable_linear()->set_x(0.2);
  cmd.mutable_twist()->mutable_linear()->set_y(0.1);
  cmd.mutable_twist()->mutable_angular()->set_z(0.05);
  EXPECT_TRUE(driver->ApplyVelocityCommand(cmd));

  autodriver::chassis::ChassisState state;
  EXPECT_TRUE(driver->ReadChassisState(&state));
  EXPECT_TRUE(state.motion_enabled());

  EXPECT_TRUE(driver->TriggerEmergencyStop());
  EXPECT_FALSE(driver->ApplyVelocityCommand(cmd));
  driver->Stop();
  EXPECT_FALSE(driver->IsRunning());
}

TEST(JetAutoBackendRegistry, CreateViaRegistry) {
  autodriver::hardware::DriverParams warmup;
  warmup["simulate"] = "true";
  auto linked = std::unique_ptr<autodriver::chassis::ChassisDriver>(
      autodriver::chassis::CreateJetAutoChassisDriver("chassis/warmup",
                                                      warmup));
  ASSERT_NE(linked, nullptr);

  autodriver::hardware::DriverParams params;
  params["simulate"] = "true";
  params["drive_mode"] = "differential";
  auto driver =
      autodriver::chassis::ChassisBackendRegistry::Instance().CreateDriver(
          "jetauto", "chassis/base", params);
  ASSERT_NE(driver, nullptr);
  EXPECT_TRUE(driver->Start());
  driver->Stop();

  auto alias =
      autodriver::chassis::ChassisBackendRegistry::Instance().CreateDriver(
          "hiwonder", "chassis/alias", params);
  ASSERT_NE(alias, nullptr);
}

}  // namespace
