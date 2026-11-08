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

#include "argus/common/component_factory.hpp"

namespace argus {
namespace common {

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

}  // namespace common
}  // namespace argus
