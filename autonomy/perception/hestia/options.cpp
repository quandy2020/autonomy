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
 * @brief Validation for Hestia open-vocabulary perception protobuf options.
 */

#include "autonomy/perception/hestia/options.hpp"

#include <cmath>
#include <string>

namespace autonomy {
namespace perception {
namespace hestia {
namespace {

void SetError(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = "Hestia: " + message;
    }
}

bool IsFinite(float value) { return std::isfinite(value); }

bool IsPositive(float value) { return IsFinite(value) && value > 0.0F; }

bool IsUnitInterval(float value) {
    return IsFinite(value) && value >= 0.0F && value <= 1.0F;
}

bool HasSupportedBackend(const std::string& backend) {
    return backend == "onnx" || backend == "tensorrt";
}

}  // namespace

bool ValidateHestiaOptions(const proto::HestiaOptions& options,
                           std::string* error) {
    if (error != nullptr) {
        error->clear();
    }

    const std::string& mode = options.mode();
    if (mode != "open" && mode != "dual") {
        SetError(error, "mode must be 'open' or 'dual'.");
        return false;
    }

    if (!HasSupportedBackend(options.backend())) {
        SetError(error, "backend must be 'onnx' or 'tensorrt'.");
        return false;
    }

    if (options.open_model_path().empty()) {
        SetError(error, "open_model_path must not be empty.");
        return false;
    }
    if (options.open_width() == 0 || options.open_height() == 0 ||
        options.max_detections() == 0) {
        SetError(error,
                 "open_width, open_height, and max_detections must be "
                 "positive.");
        return false;
    }
    if (options.open_prompts_size() == 0) {
        SetError(error, "open_prompts must not be empty.");
        return false;
    }

    if (mode == "dual") {
        if (options.home_model_path().empty()) {
            SetError(error, "home_model_path must not be empty in dual mode.");
            return false;
        }
        if (options.home_width() == 0 || options.home_height() == 0) {
            SetError(error,
                     "home_width and home_height must be positive in dual "
                     "mode.");
            return false;
        }
        if (options.home_labels_size() == 0) {
            SetError(error, "home_labels must not be empty in dual mode.");
            return false;
        }
    }

    if (!IsUnitInterval(options.confidence_threshold())) {
        SetError(error, "confidence_threshold must be finite and within [0, 1].");
        return false;
    }
    if (!IsPositive(options.depth_scale())) {
        SetError(error, "depth_scale must be finite and positive.");
        return false;
    }
    if (!IsPositive(options.min_depth_m()) ||
        !IsPositive(options.max_depth_m()) ||
        options.min_depth_m() >= options.max_depth_m()) {
        SetError(error, "min_depth_m must be less than max_depth_m.");
        return false;
    }
    if (options.min_depth_samples() == 0) {
        SetError(error, "min_depth_samples must be positive.");
        return false;
    }
    if (!IsFinite(options.inner_box_scale()) ||
        options.inner_box_scale() <= 0.0F ||
        options.inner_box_scale() > 1.0F) {
        SetError(error, "inner_box_scale must be in (0, 1].");
        return false;
    }
    if (!IsPositive(options.depth_outlier_m())) {
        SetError(error, "depth_outlier_m must be finite and positive.");
        return false;
    }
    if (options.camera_frame().empty()) {
        SetError(error, "camera_frame must not be empty.");
        return false;
    }
    if (!IsUnitInterval(options.association_iou_threshold()) ||
        !IsUnitInterval(options.merge_iou_threshold())) {
        SetError(error,
                 "association_iou_threshold and merge_iou_threshold must be "
                 "in [0, 1].");
        return false;
    }
    if (!IsPositive(options.lost_timeout_sec())) {
        SetError(error, "lost_timeout_sec must be finite and positive.");
        return false;
    }
    if (options.detections_2d_topic().empty() ||
        options.detections_3d_topic().empty()) {
        SetError(error, "detections_2d_topic and detections_3d_topic must not "
                        "be empty.");
        return false;
    }
    if (options.detections_2d_topic() == options.detections_3d_topic()) {
        SetError(error,
                 "detections_2d_topic and detections_3d_topic must differ.");
        return false;
    }
    if (!IsPositive(options.max_input_skew_sec()) ||
        !IsPositive(options.max_data_age_sec())) {
        SetError(error,
                 "max_input_skew_sec and max_data_age_sec must be finite and "
                 "positive.");
        return false;
    }

    return true;
}

}  // namespace hestia
}  // namespace perception
}  // namespace autonomy
