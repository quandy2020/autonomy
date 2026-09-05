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
 * @brief Validation for Fathom model and component protobuf options.
 */

#include "autonomy/perception/fathom/options.hpp"

#include <cmath>
#include <cstdint>
#include <limits>

namespace autonomy {
namespace perception {
namespace fathom {
namespace {

void SetError(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = "Fathom: " + message;
    }
}

bool FitsInt(uint32_t value) {
    return value <= static_cast<uint32_t>(std::numeric_limits<int>::max());
}

}  // namespace

bool ValidateModelOptions(const proto::FathomOptions& options,
                          std::string* error) {
    if (error != nullptr) {
        error->clear();
    }
    if (options.model_path().empty()) {
        SetError(error, "model_path must not be empty.");
        return false;
    }
    if (options.backend() != "onnx" && options.backend() != "tensorrt") {
        SetError(error, "backend must be 'onnx' or 'tensorrt'.");
        return false;
    }
    if (options.input_width() == 0 || options.input_height() == 0) {
        SetError(error, "input_width and input_height must be positive.");
        return false;
    }
    if (!FitsInt(options.input_width()) || !FitsInt(options.input_height())) {
        SetError(error, "input_width and input_height exceed supported range.");
        return false;
    }
    if (!std::isfinite(options.depth_scale()) ||
        options.depth_scale() <= 0.0F) {
        SetError(error, "depth_scale must be finite and positive.");
        return false;
    }
    if (!std::isfinite(options.mask_threshold()) ||
        options.mask_threshold() < 0.0F || options.mask_threshold() > 1.0F) {
        SetError(error, "mask_threshold must be finite and within [0, 1].");
        return false;
    }
    return true;
}

bool ValidateFathomOptions(const proto::FathomOptions& options,
                           std::string* error) {
    if (!ValidateModelOptions(options, error)) {
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
    return true;
}

}  // namespace fathom
}  // namespace perception
}  // namespace autonomy
