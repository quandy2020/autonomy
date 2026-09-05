/*
 * Copyright 2025 The OpenRobotic Beginner Authors (duyongquan)
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

#include "autonomy/perception/fathom/config.hpp"

#include <cmath>

namespace autonomy {
namespace perception {
namespace fathom {
namespace {

void SetError(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = "Fathom: " + message;
    }
}

}  // namespace

bool ValidateFathomConfig(const FathomConfig& config, std::string* error) {
    if (error != nullptr) {
        error->clear();
    }
    if (config.model_path.empty()) {
        SetError(error, "model_path must not be empty.");
        return false;
    }
    if (config.backend != "onnx" && config.backend != "tensorrt") {
        SetError(error, "backend must be 'onnx' or 'tensorrt'.");
        return false;
    }
    if (config.input_width <= 0 || config.input_height <= 0) {
        SetError(error, "input_width and input_height must be positive.");
        return false;
    }
    if (!std::isfinite(config.depth_scale) || config.depth_scale <= 0.0F) {
        SetError(error, "depth_scale must be finite and positive.");
        return false;
    }
    if (!std::isfinite(config.mask_threshold) || config.mask_threshold < 0.0F ||
        config.mask_threshold > 1.0F) {
        SetError(error, "mask_threshold must be finite and within [0, 1].");
        return false;
    }
    return true;
}

}  // namespace fathom
}  // namespace perception
}  // namespace autonomy
