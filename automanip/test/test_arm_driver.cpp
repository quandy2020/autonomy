/*
 * Copyright 2026 Automanip contributors duyongquan (quandy2020@126.com)
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
 * @file test_arm_driver.cpp
 * @brief Backend registry, stub integration, and osc2 driver tracking.
 */

#include "arm/backend_registry.hpp"
#include "arm/codec.hpp"
#include "arm/osc2/driver.hpp"
#include "arm/stub/driver.hpp"

#include <cmath>
#include <memory>

#include <gtest/gtest.h>

namespace {

automanip::arm::ArmPlantOptions Options() {
  automanip::arm::ArmPlantOptions options;
  options.chain = automanip::arm::MakeDefaultArm();
  options.osc2.horizon = 6;
  options.osc2.iterations = 2;
  options.osc2.position_weight = 80.0;
  options.osc2.input_weight = 0.01;
  options.osc2.posture_weight = 0.05;
  return options;
}

}  // namespace

TEST(ArmBackend, ResolvesOsc2Alias) {
  const auto options = Options();
  auto osc2 = automanip::arm::ArmBackendRegistry::Instance().Create(
      "osc2", "arm/osc2", options);
  auto ocs2 = automanip::arm::ArmBackendRegistry::Instance().Create(
      "ocs2", "arm/ocs2", options);
  auto stub = automanip::arm::ArmBackendRegistry::Instance().Create(
      "sim", "arm/stub", options);
  ASSERT_NE(osc2, nullptr);
  ASSERT_NE(ocs2, nullptr);
  ASSERT_NE(stub, nullptr);
  EXPECT_TRUE(osc2->Start());
  EXPECT_TRUE(ocs2->Start());
  EXPECT_TRUE(stub->Start());
}

TEST(StubArm, IntegratesVelocityCommand) {
  auto driver = std::unique_ptr<automanip::arm::ArmDriver>(
      automanip::arm::CreateStubArmDriver("arm/stub", Options()));
  ASSERT_TRUE(driver->Start());
  automanip::arm::JointCommand command;
  command.set_interface_name("velocity");
  for (int i = 0; i < 6; ++i) {
    command.add_values(i == 0 ? 0.5 : 0.0);
  }
  ASSERT_TRUE(driver->ApplyJointCommand(command));
  ASSERT_TRUE(driver->Step(0.2));
  automanip::arm::JointState state;
  ASSERT_TRUE(driver->ReadJointState(&state));
  EXPECT_NEAR(state.position(0), 0.1, 1e-6);
}

TEST(Osc2Arm, PoseCommandReducesError) {
  auto driver = std::unique_ptr<automanip::arm::ArmDriver>(
      automanip::arm::osc2::CreateOsc2ArmDriver("arm/osc2", Options()));
  ASSERT_TRUE(driver->Start());
  automanip::arm::PoseCommand current;
  ASSERT_TRUE(driver->ReadEndEffectorPose(&current));
  automanip::arm::PoseCommand target = current;
  target.mutable_pose()->mutable_position()->set_x(
      current.pose().position().x() + 0.04);
  ASSERT_TRUE(driver->ApplyPoseTarget(target));
  ASSERT_TRUE(driver->ApplyMode("track"));
  for (int step = 0; step < 100; ++step) {
    ASSERT_TRUE(driver->Step(0.02));
  }
  automanip::arm::PoseCommand reached;
  ASSERT_TRUE(driver->ReadEndEffectorPose(&reached));
  const double dx = reached.pose().position().x() - target.pose().position().x();
  const double dy = reached.pose().position().y() - target.pose().position().y();
  const double dz = reached.pose().position().z() - target.pose().position().z();
  EXPECT_LT(std::sqrt(dx * dx + dy * dy + dz * dz), 0.01);
  EXPECT_EQ(driver->GetMode(), automanip::arm::ArmMode::kTrack);
}
