/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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

#include "autonomy/perception/fathom/component_config.hpp"

#include <cstdint>
#include <limits>
#include <utility>

namespace autonomy {
namespace perception {
namespace fathom {
namespace {

void SetError(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = "Fathom component: " + message;
    }
}

bool FitsInt(uint32_t value) {
    return value <= static_cast<uint32_t>(std::numeric_limits<int>::max());
}

}  // namespace

bool TranslateFathomComponentConfig(
    const proto::FathomComponentConfig& component_config,
    FathomConfig* fathom_config, FathomComponentTopics* topics,
    std::string* error) {
    if (error != nullptr) {
        error->clear();
    }
    if (fathom_config == nullptr || topics == nullptr) {
        SetError(error, "output configuration pointers must not be null.");
        return false;
    }
    if (component_config.refined_depth_topic().empty()) {
        SetError(error, "refined_depth_topic must not be empty.");
        return false;
    }
    if (component_config.point_cloud_topic().empty()) {
        SetError(error, "point_cloud_topic must not be empty.");
        return false;
    }
    if (!FitsInt(component_config.input_width()) ||
        !FitsInt(component_config.input_height())) {
        SetError(error, "input_width and input_height exceed supported range.");
        return false;
    }

    FathomConfig translated;
    translated.model_path = component_config.model_path();
    translated.backend = component_config.backend();
    translated.input_width = static_cast<int>(component_config.input_width());
    translated.input_height = static_cast<int>(component_config.input_height());
    translated.depth_scale = component_config.depth_scale();
    translated.mask_threshold = component_config.mask_threshold();
    if (!ValidateFathomConfig(translated, error)) {
        return false;
    }

    FathomComponentTopics translated_topics;
    translated_topics.refined_depth = component_config.refined_depth_topic();
    translated_topics.point_cloud = component_config.point_cloud_topic();
    *fathom_config = std::move(translated);
    *topics = std::move(translated_topics);
    return true;
}

}  // namespace fathom
}  // namespace perception
}  // namespace autonomy
