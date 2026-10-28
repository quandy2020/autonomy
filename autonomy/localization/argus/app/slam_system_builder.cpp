/*
 * Copyright 2026 The Openbot Authors
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

#include "slam_system_builder.hpp"

#include <utility>

namespace argus {
namespace app {
namespace {

class RegistryComponent : public common::Component {
 public:
  RegistryComponent(std::string name, std::string type)
      : name_(std::move(name)), type_(std::move(type)) {}

  const std::string& name() const override { return name_; }

  bool Configure(const Config& config) override {
    if (config.name.empty() || config.name != name_) {
      return false;
    }
    configured_ = true;
    return true;
  }

  bool Start() override {
    if (!configured_) {
      return false;
    }
    started_ = true;
    return true;
  }

  void Stop() override { started_ = false; }

  common::Health health() const override {
    if (!started_) {
      return common::Health{common::HealthLevel::kUnknown, type_};
    }
    return common::Health{common::HealthLevel::kOk, type_};
  }

 private:
  std::string name_;
  std::string type_;
  bool configured_ = false;
  bool started_ = false;
};

common::ComponentFactory::Creator MakeCreator(std::string type) {
  return [type = std::move(type)](const common::Component::Config& config)
             -> std::unique_ptr<common::Component> {
    if (config.name.empty()) {
      return nullptr;
    }
    return std::make_unique<RegistryComponent>(config.name, type);
  };
}

}  // namespace

std::vector<std::string> SlamSystemBuilder::BuiltinTypes() {
  return {"lidar_frontend", "visual_frontend"};
}

void SlamSystemBuilder::RegisterBuiltins(
    common::ComponentFactory* factory) const {
  if (factory == nullptr) {
    return;
  }
  for (const std::string& type : BuiltinTypes()) {
    factory->RegisterCreator(type, MakeCreator(type));
  }
}

std::unique_ptr<SlamSystem> SlamSystemBuilder::Build(
    const SlamSystemConfig& config) const {
  if (!ValidateSlamSystemConfig(config)) {
    return nullptr;
  }
  auto system = std::make_unique<SlamSystem>(config);
  common::ComponentFactory factory;
  RegisterBuiltins(&factory);
  for (const ComponentSpec& spec : config.components) {
    common::Component::Config component_config;
    component_config.name = spec.name;
    component_config.params["type"] = spec.type;
    component_config.params["pipeline"] = config.pipeline;
    std::unique_ptr<common::Component> created =
        factory.Create(spec.type, component_config);
    if (created == nullptr || !created->Configure(component_config)) {
      return nullptr;
    }
    system->registry().Register(
        std::shared_ptr<common::Component>(std::move(created)));
  }
  return system;
}

}  // namespace app
}  // namespace argus
