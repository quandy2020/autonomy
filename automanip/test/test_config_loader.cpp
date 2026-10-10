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
 * @file test_config_loader.cpp
 * @brief YAML arm block and the default chain.
 */

#include "automanip/config_loader.hpp"

#include <filesystem>
#include <fstream>

#include <gtest/gtest.h>

namespace {

std::filesystem::path WriteTree(const std::string& yaml) {
  const std::filesystem::path root =
      std::filesystem::temp_directory_path() / "automanip_config_loader_test";
  std::filesystem::create_directories(root / "config");
  const std::filesystem::path file = root / "config" / "automanip.yaml";
  std::ofstream out(file);
  out << yaml;
  return root;
}

}  // namespace

TEST(AutomanipConfig, ParsesArm) {
  const auto root = WriteTree(R"(
node_name: arm_test
arm:
  enable: true
  id: arm/left
  backend: vendor
  initial_mode: home
  watchdog_ms: 250
  joints:
    - {name: j1, axis: [0, 0, 1], origin: [0, 0, 0.1], lower: -1, upper: 1, velocity: 0.5, home: 0.2}
)");
  const automanip::Config config =
      automanip::LoadConfig(root.string(), "automanip.yaml");
  EXPECT_EQ(config.node_name, "arm_test");
  EXPECT_TRUE(config.arm.enable);
  EXPECT_EQ(config.arm.id, "arm/left");
  EXPECT_EQ(config.arm.backend, "vendor");
  EXPECT_EQ(config.arm.initial_mode, "home");
  EXPECT_EQ(config.arm.watchdog_ms, 250);
  ASSERT_EQ(config.arm.plant.chain.dof(), 1);
  EXPECT_EQ(config.arm.plant.chain.joints[0].name, "j1");
  EXPECT_DOUBLE_EQ(config.arm.plant.chain.joints[0].home, 0.2);
  EXPECT_DOUBLE_EQ(config.arm.plant.chain.joints[0].velocity_limit, 0.5);
}

TEST(AutomanipConfig, MissingJointsUseDefaultArm) {
  const auto root = WriteTree("arm:\n  backend: stub\n");
  const automanip::Config config =
      automanip::LoadConfig(root.string(), "automanip.yaml");
  EXPECT_EQ(config.arm.backend, "stub");
  EXPECT_EQ(config.arm.plant.chain.dof(), 6);
}
