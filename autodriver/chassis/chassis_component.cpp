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
 * @file chassis_component.cpp
 * @brief ChassisComponent (implementation).
 */

#include "chassis/chassis_component.hpp"

#include "autodriver/config_loader.hpp"
#include "autolink/common/log.hpp"
#include "autolink/component/component.hpp"

namespace autodriver {
namespace chassis {
namespace {

Config LoadComponentConfig(const std::string& config_path) {
  if (config_path.empty()) {
    return LoadConfig();
  }
  // Absolute file path, or basename under AUTODRIVER_PATH/config/
  // (e.g. "chassis/jetauto.yaml").
  return LoadConfig({}, config_path);
}

}  // namespace

bool ChassisComponent::Init() {
  Config config;
  try {
    config = LoadComponentConfig(ConfigFilePath());
  } catch (const std::exception& ex) {
    AERROR << "ChassisComponent: load config failed: " << ex.what();
    return false;
  }
  if (!manager_.Start(node_.get(), config)) {
    AERROR << "ChassisComponent: ChassisManager::Start failed";
    return false;
  }
  AINFO << "ChassisComponent init ok backend=" << config.chassis.backend
        << " enable=" << (config.chassis.enable ? "true" : "false");
  return true;
}

bool ChassisComponent::Proc() {
  // Manager owns cmd_vel readers and the state publish thread; timer is
  // only required so mainboard keeps the component alive.
  return true;
}

void ChassisComponent::Clear() { manager_.Stop(); }

}  // namespace chassis
}  // namespace autodriver

AUTOLINK_REGISTER_COMPONENT(autodriver::chassis::ChassisComponent)
