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
 * @brief YAML node_name loader.
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

TEST(AutomanipConfig, ParsesNodeName) {
  const auto root = WriteTree("node_name: arm_test\n");
  const automanip::Config config =
      automanip::LoadConfig(root.string(), "automanip.yaml");
  EXPECT_EQ(config.node_name, "arm_test");
}

TEST(AutomanipConfig, DefaultsNodeName) {
  const auto root = WriteTree("{}\n");
  const automanip::Config config =
      automanip::LoadConfig(root.string(), "automanip.yaml");
  EXPECT_EQ(config.node_name, "automanip");
}
