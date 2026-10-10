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
 * @file arm_component.cpp
 * @brief ArmComponent (implementation).
 */

#include "automanip/arm/arm_component.hpp"

#include "autolink/common/log.hpp"
#include "autolink/component/component.hpp"
#include "automanip/config_loader.hpp"

namespace automanip {
namespace arm {
namespace {

Config LoadComponentConfig(const std::string& config_path) {
  if (config_path.empty()) {
    return LoadConfig();
  }
  return LoadConfig({}, config_path);
}

}  // namespace

bool ArmComponent::Init() {
  Config config;
  try {
    config = LoadComponentConfig(ConfigFilePath());
  } catch (const std::exception& ex) {
    AERROR << "ArmComponent: load config failed: " << ex.what();
    return false;
  }
  if (!manager_.Start(node_.get(), config)) {
    AERROR << "ArmComponent: ArmManager::Start failed";
    return false;
  }
  AINFO << "ArmComponent init ok backend=" << config.arm.backend;
  return true;
}

bool ArmComponent::Proc() { return true; }

void ArmComponent::Clear() { manager_.Stop(); }

}  // namespace arm
}  // namespace automanip

AUTOLINK_REGISTER_COMPONENT(automanip::arm::ArmComponent)
