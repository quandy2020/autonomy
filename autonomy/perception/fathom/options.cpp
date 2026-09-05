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

/**
 * @file options.cpp
 * @brief Validation and translation of Fathom component protobuf options.
 */

#include "autonomy/perception/fathom/options.hpp"

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

bool TranslateFathomOptions(const proto::FathomOptions& options,
                            FathomConfig* fathom_config, FathomTopics* topics,
                            std::string* error) {
    if (error != nullptr) {
        error->clear();
    }
    if (fathom_config == nullptr || topics == nullptr) {
        SetError(error, "output configuration pointers must not be null.");
        return false;
    }
    if (options.refined_depth_topic().empty()) {
        SetError(error, "refined_depth_topic must not be empty.");
        return false;
    }
    if (options.point_cloud_topic().empty()) {
        SetError(error, "point_cloud_topic must not be empty.");
        return false;
    }
    if (options.refined_depth_topic() == options.point_cloud_topic()) {
        SetError(error,
                 "refined_depth_topic and point_cloud_topic must differ.");
        return false;
    }
    if (!FitsInt(options.input_width()) || !FitsInt(options.input_height())) {
        SetError(error, "input_width and input_height exceed supported range.");
        return false;
    }

    FathomConfig translated;
    translated.model_path = options.model_path();
    translated.backend = options.backend();
    translated.input_width = static_cast<int>(options.input_width());
    translated.input_height = static_cast<int>(options.input_height());
    translated.depth_scale = options.depth_scale();
    translated.mask_threshold = options.mask_threshold();
    if (!ValidateFathomConfig(translated, error)) {
        return false;
    }

    FathomTopics translated_topics;
    translated_topics.refined_depth = options.refined_depth_topic();
    translated_topics.point_cloud = options.point_cloud_topic();
    *fathom_config = std::move(translated);
    *topics = std::move(translated_topics);
    return true;
}

}  // namespace fathom
}  // namespace perception
}  // namespace autonomy
