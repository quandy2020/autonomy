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
 * @file l1w_component.cpp
 * @brief L1wComponent (implementation).
 */

#include "chassis/l1w/l1w_component.hpp"

#include "chassis/l1w/driver.hpp"
#include "autodriver/config_loader.hpp"
#include "autolink/common/log.hpp"
#include "autolink/component/component.hpp"

namespace autodriver {
namespace chassis {
namespace l1w {
namespace {

Config LoadL1wConfig(const std::string& config_path) {
  if (config_path.empty()) {
    return LoadConfig({}, "chassis/l1w.yaml");
  }
  return LoadConfig({}, config_path);
}

void ApplyL1wDefaults(Config* config) {
  if (config == nullptr) {
    return;
  }
  config->chassis.enable = true;
  if (config->chassis.backend.empty() || config->chassis.backend == "stub" ||
      config->chassis.backend == "sim" || config->chassis.backend == "fake") {
    config->chassis.backend = "l1w";
  }
  if (config->chassis.locomotion.empty() ||
      config->chassis.locomotion == "differential") {
    config->chassis.locomotion = "wheel_legged";
  }
  if (config->chassis.tools.empty()) {
    config->chassis.tools = {"lie", "passive", "cancel_crawl", "attitude",
                             "stand"};
  }
  if (config->chassis.tool_cmd_channel.empty()) {
    config->chassis.tool_cmd_channel = "/chassis/tool";
  }
  if (config->chassis.supports_lateral.empty()) {
    config->chassis.supports_lateral = "true";
  }
}

}  // namespace

bool L1wComponent::Init() {
  (void)&CreateL1wChassisDriver;

  Config config;
  try {
    config = LoadL1wConfig(ConfigFilePath());
  } catch (const std::exception& ex) {
    AERROR << "L1wComponent: load config failed: " << ex.what();
    return false;
  }
  ApplyL1wDefaults(&config);

  if (!manager_.Start(node_.get(), config)) {
    AERROR << "L1wComponent: ChassisManager::Start failed";
    return false;
  }

  const auto& p = config.chassis.params;
  const auto host_it = p.find("host");
  AINFO << "L1wComponent init ok"
        << " backend=" << config.chassis.backend
        << " locomotion=" << config.chassis.locomotion
        << " host="
        << (host_it != p.end() ? host_it->second : "(default)")
#if defined(AUTODRIVER_HAVE_GENISOM_L1W)
        << " sdk=linked"
#else
        << " sdk=simulate-only"
#endif
        << " cmd=" << config.chassis.cmd_vel_channel;
  return true;
}

bool L1wComponent::Proc() { return true; }

void L1wComponent::Clear() { manager_.Stop(); }

}  // namespace l1w
}  // namespace chassis
}  // namespace autodriver

AUTOLINK_REGISTER_COMPONENT(autodriver::chassis::l1w::L1wComponent)
