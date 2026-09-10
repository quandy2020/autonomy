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

namespace {

TEST(ChassisBackendRegistry, StubCreateAndDrive) {
  ASSERT_TRUE(autodriver::chassis::ChassisBackendRegistry::Instance().Has(
      "stub"));
  ASSERT_TRUE(autodriver::chassis::ChassisBackendRegistry::Instance().Has(
      "sim"));

  autodriver::hardware::DriverParams params;
  auto driver = autodriver::chassis::ChassisBackendRegistry::Instance().Create(
      "stub", "chassis/test", params);
  ASSERT_NE(driver, nullptr);
  EXPECT_TRUE(driver->Start());
  EXPECT_TRUE(driver->IsRunning());

  autodriver::chassis::ChassisCommand cmd;
  cmd.linear_x = 0.5;
  cmd.angular_z = 0.1;
  EXPECT_TRUE(driver->ApplyCommand(cmd));

  autodriver::chassis::ChassisState state;
  EXPECT_TRUE(driver->GetState(&state));
  EXPECT_NEAR(state.linear_x, 0.5, 1e-6);

  driver->Stop();
  EXPECT_FALSE(driver->IsRunning());
}

TEST(ChassisBackendRegistry, UnknownBackend) {
  autodriver::hardware::DriverParams params;
  auto driver = autodriver::chassis::ChassisBackendRegistry::Instance().Create(
      "no_such_vendor", "chassis/x", params);
  EXPECT_EQ(driver, nullptr);
}

}  // namespace
