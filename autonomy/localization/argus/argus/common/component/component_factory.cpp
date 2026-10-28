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

#include "common/component/component_factory.hpp"

#include <utility>
#include <vector>

namespace argus {
namespace common {
namespace {

class PluginBoundComponent final : public Component {
 public:
  PluginBoundComponent(std::unique_ptr<Component> component,
                       std::shared_ptr<void> library)
      : library_(std::move(library)), component_(std::move(component)) {}

  const std::string& name() const override { return component_->name(); }
  bool Configure(const Config& config) override {
    return component_->Configure(config);
  }
  bool Start() override { return component_->Start(); }
  void Stop() override { component_->Stop(); }
  Health health() const override { return component_->health(); }

 private:
  // Member destruction is reversed: component_ is destroyed before the
  // libraries that contain its vtable and destructor.
  std::shared_ptr<void> library_;
  std::unique_ptr<Component> component_;
};

struct PluginCreator {
  // creator is destroyed first, so any target destructor still runs while
  // library code remains loaded.
  std::shared_ptr<void> library;
  ComponentFactory::Creator creator;

  std::unique_ptr<Component> operator()(const Component::Config& config) const {
    std::unique_ptr<Component> component = creator(config);
    if (!component) {
      return nullptr;
    }
    return std::make_unique<PluginBoundComponent>(std::move(component),
                                                  library);
  }
};

}  // namespace

void ComponentFactory::RegisterCreator(const std::string& type,
                                       Creator creator) {
  creators_[type] = std::move(creator);
}

std::unique_ptr<Component> ComponentFactory::Create(
    const std::string& type, const Component::Config& config) const {
  const auto it = creators_.find(type);
  if (it == creators_.end() || !it->second) {
    return nullptr;
  }
  return it->second(config);
}

void ComponentFactory::MergePluginCreators(ComponentFactory&& other,
                                           std::shared_ptr<void> library) {
  if (!library) {
    return;
  }
  for (auto& entry : other.creators_) {
    creators_[entry.first] = PluginCreator{library, std::move(entry.second)};
  }
}

}  // namespace common
}  // namespace argus
