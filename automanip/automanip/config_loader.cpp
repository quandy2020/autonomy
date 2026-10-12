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
 * @file config_loader.cpp
 * @brief YAML → Config (implementation).
 */

#include "automanip/config_loader.hpp"

#include <filesystem>
#include <stdexcept>
#include <string>

#include "autolink/common/log.hpp"
#include "automanip/environment.hpp"
#include "automanip/yaml.hpp"

namespace automanip {
namespace {

std::string ReadString(const YAML::Node& node, const char* key,
                       const std::string& fallback) {
  const YAML::Node value = node[key];
  if (!value || value.IsNull() || !value.IsScalar()) {
    return fallback;
  }
  try {
    return value.as<std::string>();
  } catch (const YAML::Exception&) {
  }
  return fallback;
}

std::filesystem::path ResolveConfigPath(const std::string& directory,
                                        const std::string& basename) {
  namespace fs = std::filesystem;
  const fs::path file(basename);
  if (file.is_absolute()) {
    return file;
  }
  std::string root = directory;
  if (root.empty()) {
    root = common::WorkRoot();
  }
  const fs::path direct = fs::path(root) / basename;
  if (fs::exists(direct)) {
    return direct;
  }
  return fs::path(root) / "config" / basename;
}

Config LoadFile(const std::filesystem::path& path) {
  if (!std::filesystem::exists(path)) {
    throw std::runtime_error("automanip config not found: " + path.string());
  }
  const YAML::Node root = YAML::LoadFile(path.string());
  Config config;
  if (root && root.IsMap()) {
    config.node_name = ReadString(root, "node_name", config.node_name);
  }
  AINFO << "loaded automanip config " << path.string()
        << " node_name=" << config.node_name;
  return config;
}

}  // namespace

Config LoadConfig() { return LoadConfig({}, "automanip.yaml"); }

Config LoadConfig(const std::string& config_basename) {
  return LoadConfig({}, config_basename);
}

Config LoadConfig(const std::string& configuration_directory,
                  const std::string& config_basename) {
  const std::string basename =
      config_basename.empty() ? "automanip.yaml" : config_basename;
  return LoadFile(ResolveConfigPath(configuration_directory, basename));
}

}  // namespace automanip
