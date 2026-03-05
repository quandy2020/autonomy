/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "autonomy/tools/aviz/plugins/plugin_factory.hpp"

#include <iostream>
#include <memory>

#include "autonomy/tools/aviz/common/display.hpp"

PluginFactory& PluginFactory::Instance() {
  static PluginFactory instance;
  return instance;
}

void PluginFactory::registerPlugin(const PluginDescription& description, DisplayCreator creator) {
  // Check if already registered
  if (class_id_to_index_.find(description.class_id) != class_id_to_index_.end()) {
    std::cerr << "PluginFactory: Plugin " << description.class_id << " already registered" << std::endl;
    return;
  }

  plugins_.push_back(description);
  size_t index = plugins_.size() - 1;
  class_id_to_index_[description.class_id] = index;
  creators_[description.class_id] = creator;
}

std::unique_ptr<aviz::common::Display> PluginFactory::createDisplay(const std::string& class_id,
                                                                    const std::string& name) {
  auto it = creators_.find(class_id);
  if (it == creators_.end()) {
    std::cerr << "PluginFactory: Unknown plugin class_id: " << class_id << std::endl;
    return nullptr;
  }

  return it->second(name);
}

const PluginDescription* PluginFactory::getPluginDescription(const std::string& class_id) const {
  auto it = class_id_to_index_.find(class_id);
  if (it == class_id_to_index_.end()) {
    return nullptr;
  }

  return &plugins_[it->second];
}
