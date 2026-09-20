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
 * @file jetauto_component.cpp
 * @brief JetAutoComponent (implementation).
 */

#include "chassis/jetauto/jetauto_component.hpp"

#include "chassis/jetauto/driver.hpp"
#include "autodriver/config_loader.hpp"
#include "autolink/common/log.hpp"
#include "autolink/component/component.hpp"

namespace autodriver {
namespace chassis {
namespace jetauto {
namespace {

Config LoadJetAutoConfig(const std::string& config_path) {
  if (config_path.empty()) {
    return LoadConfig({}, "chassis/jetauto.yaml");
  }
  return LoadConfig({}, config_path);
}

void ApplyJetAutoDefaults(Config* config) {
  if (config == nullptr) {
    return;
  }
  config->chassis.enable = true;
  if (config->chassis.backend.empty() || config->chassis.backend == "stub" ||
      config->chassis.backend == "sim" || config->chassis.backend == "fake") {
    config->chassis.backend = "jetauto";
  }
  if (config->chassis.params.count("drive_mode") == 0) {
    if (config->chassis.locomotion == "omni" ||
        config->chassis.locomotion == "omnidirectional") {
      config->chassis.params["drive_mode"] = "mecanum";
    } else if (config->chassis.locomotion == "differential" ||
               config->chassis.locomotion == "diff") {
      config->chassis.params["drive_mode"] = "differential";
    }
  }
}

}  // namespace

bool JetAutoComponent::Init() {
  // Ensure REGISTER_CHASSIS_BACKEND(jetauto) from this DSO is linked.
  (void)&CreateJetAutoChassisDriver;

  Config config;
  try {
    config = LoadJetAutoConfig(ConfigFilePath());
  } catch (const std::exception& ex) {
    AERROR << "JetAutoComponent: load config failed: " << ex.what();
    return false;
  }
  ApplyJetAutoDefaults(&config);

  if (!manager_.Start(node_.get(), config)) {
    AERROR << "JetAutoComponent: ChassisManager::Start failed";
    return false;
  }

  const auto& p = config.chassis.params;
  const auto device_it = p.find("device");
  const auto baud_it = p.find("baud");
  const auto mode_it = p.find("drive_mode");
  AINFO << "JetAutoComponent init ok"
        << " backend=" << config.chassis.backend
        << " locomotion=" << config.chassis.locomotion
        << " drive_mode="
        << (mode_it != p.end() ? mode_it->second : "(derived)")
        << " device="
        << (device_it != p.end() ? device_it->second : "(default)")
        << " baud=" << (baud_it != p.end() ? baud_it->second : "(default)")
        << " cmd=" << config.chassis.cmd_vel_channel;
  return true;
}

bool JetAutoComponent::Proc() {
  // ChassisManager owns readers + state publish thread.
  return true;
}

void JetAutoComponent::Clear() { manager_.Stop(); }

}  // namespace jetauto
}  // namespace chassis
}  // namespace autodriver

AUTOLINK_REGISTER_COMPONENT(autodriver::chassis::jetauto::JetAutoComponent)
