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

#include "arm/chain.hpp"
#include "autolink/common/log.hpp"
#include "automanip/environment.hpp"
#include "automanip/yaml.hpp"

namespace automanip {
namespace {

double ReadDouble(const YAML::Node& node, const char* key, double fallback) {
  const YAML::Node value = node[key];
  if (!value || value.IsNull() || !value.IsScalar()) {
    return fallback;
  }
  try {
    return value.as<double>();
  } catch (const YAML::Exception&) {
  }
  try {
    return static_cast<double>(value.as<int>());
  } catch (const YAML::Exception&) {
  }
  return fallback;
}

int ReadInt(const YAML::Node& node, const char* key, int fallback) {
  const YAML::Node value = node[key];
  if (!value || value.IsNull() || !value.IsScalar()) {
    return fallback;
  }
  try {
    return value.as<int>();
  } catch (const YAML::Exception&) {
  }
  try {
    return static_cast<int>(value.as<double>());
  } catch (const YAML::Exception&) {
  }
  return fallback;
}

bool ReadBool(const YAML::Node& node, const char* key, bool fallback) {
  const YAML::Node value = node[key];
  if (!value || value.IsNull() || !value.IsScalar()) {
    return fallback;
  }
  try {
    return value.as<bool>();
  } catch (const YAML::Exception&) {
  }
  try {
    const std::string text = value.as<std::string>();
    if (text == "1" || text == "true" || text == "True" || text == "yes") {
      return true;
    }
    if (text == "0" || text == "false" || text == "False" || text == "no") {
      return false;
    }
  } catch (const YAML::Exception&) {
  }
  return fallback;
}

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

double ReadScalarDouble(const YAML::Node& node, double fallback) {
  if (!node || !node.IsScalar()) {
    return fallback;
  }
  try {
    return node.as<double>();
  } catch (const YAML::Exception&) {
  }
  try {
    return static_cast<double>(node.as<int>());
  } catch (const YAML::Exception&) {
  }
  return fallback;
}

Eigen::Vector3d ReadVec3(const YAML::Node& node,
                         const Eigen::Vector3d& fallback) {
  if (!node || !node.IsSequence() || node.size() < 3) {
    return fallback;
  }
  return Eigen::Vector3d(ReadScalarDouble(node[0], fallback.x()),
                         ReadScalarDouble(node[1], fallback.y()),
                         ReadScalarDouble(node[2], fallback.z()));
}

void ReadOsc2(const YAML::Node& node, arm::osc2::Osc2Settings* settings) {
  if (!node || !settings) {
    return;
  }
  settings->horizon = ReadInt(node, "horizon", settings->horizon);
  settings->mpc_dt = ReadDouble(node, "mpc_dt", settings->mpc_dt);
  settings->iterations = ReadInt(node, "iterations", settings->iterations);
  settings->position_weight =
      ReadDouble(node, "position_weight", settings->position_weight);
  settings->orientation_weight =
      ReadDouble(node, "orientation_weight", settings->orientation_weight);
  settings->input_weight =
      ReadDouble(node, "input_weight", settings->input_weight);
  settings->joint_weight =
      ReadDouble(node, "joint_weight", settings->joint_weight);
  settings->posture_weight =
      ReadDouble(node, "posture_weight", settings->posture_weight);
  settings->terminal_scale =
      ReadDouble(node, "terminal_scale", settings->terminal_scale);
  settings->limit_weight =
      ReadDouble(node, "limit_weight", settings->limit_weight);
  settings->limit_margin =
      ReadDouble(node, "limit_margin", settings->limit_margin);
  settings->damping = ReadDouble(node, "damping", settings->damping);
}

void ReadChain(const YAML::Node& arm, arm::SerialChain* chain) {
  if (chain == nullptr) {
    return;
  }
  const YAML::Node joints = arm["joints"];
  if (!joints || !joints.IsSequence() || joints.size() == 0) {
    *chain = arm::MakeDefaultArm();
  } else {
    chain->joints.clear();
    for (const YAML::Node& item : joints) {
      arm::JointSpec joint;
      joint.name = ReadString(item, "name", "joint");
      joint.axis = ReadVec3(item["axis"], joint.axis);
      joint.origin = ReadVec3(item["origin"], joint.origin);
      joint.lower = ReadDouble(item, "lower", joint.lower);
      joint.upper = ReadDouble(item, "upper", joint.upper);
      joint.velocity_limit =
          ReadDouble(item, "velocity", joint.velocity_limit);
      joint.home = ReadDouble(item, "home", joint.home);
      chain->joints.push_back(std::move(joint));
    }
  }
  if (arm["tool_xyz"]) {
    chain->tool = ReadVec3(arm["tool_xyz"], chain->tool);
  }
}

void ReadArm(const YAML::Node& root, Config* config) {
  const YAML::Node arm = root["arm"];
  if (!arm || !arm.IsMap()) {
    config->arm.plant.chain = arm::MakeDefaultArm();
    return;
  }
  Config::Arm& out = config->arm;
  out.enable = ReadBool(arm, "enable", out.enable);
  out.id = ReadString(arm, "id", out.id);
  out.backend = ReadString(arm, "backend", out.backend);
  out.initial_mode = ReadString(arm, "initial_mode", out.initial_mode);
  out.control_period_ms =
      ReadInt(arm, "control_period_ms", out.control_period_ms);
  out.watchdog_ms = ReadInt(arm, "watchdog_ms", out.watchdog_ms);
  out.capability_period_ticks =
      ReadInt(arm, "capability_period_ticks", out.capability_period_ticks);
  out.pose_channel = ReadString(arm, "pose_channel", out.pose_channel);
  out.joint_command_channel =
      ReadString(arm, "joint_command_channel", out.joint_command_channel);
  out.mode_channel = ReadString(arm, "mode_channel", out.mode_channel);
  out.joint_state_channel =
      ReadString(arm, "joint_state_channel", out.joint_state_channel);
  out.ee_pose_channel = ReadString(arm, "ee_pose_channel", out.ee_pose_channel);
  out.mode_state_channel =
      ReadString(arm, "mode_state_channel", out.mode_state_channel);
  out.capability_channel =
      ReadString(arm, "capability_channel", out.capability_channel);
  out.event_channel = ReadString(arm, "event_channel", out.event_channel);
  out.plant.frame_id = ReadString(arm, "frame_id", out.plant.frame_id);
  out.plant.tool_frame_id =
      ReadString(arm, "tool_frame_id", out.plant.tool_frame_id);
  ReadOsc2(arm["osc2"], &out.plant.osc2);
  ReadChain(arm, &out.plant.chain);
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
    ReadArm(root, &config);
  } else {
    config.arm.plant.chain = arm::MakeDefaultArm();
  }
  AINFO << "loaded automanip config " << path.string()
        << " backend=" << config.arm.backend
        << " dof=" << config.arm.plant.chain.dof();
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
