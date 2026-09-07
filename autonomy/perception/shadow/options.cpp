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
 * @brief Validation for Shadow selected-person following protobuf options.
 */

#include "autonomy/perception/shadow/options.hpp"

#include <cmath>
#include <initializer_list>
#include <string>

namespace autonomy {
namespace perception {
namespace shadow {
namespace {

void SetError(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = "Shadow: " + message;
    }
}

bool IsFinite(float value) {
    return std::isfinite(value);
}

bool IsPositive(float value) {
    return IsFinite(value) && value > 0.0F;
}

bool IsUnitInterval(float value) {
    return IsFinite(value) && value >= 0.0F && value <= 1.0F;
}

bool HasSupportedBackend(const std::string& backend) {
    return backend == "onnx" || backend == "tensorrt";
}

bool AnyPositive(std::initializer_list<float> weights) {
    for (const float weight : weights) {
        if (weight > 0.0F) {
            return true;
        }
    }
    return false;
}

bool AllNonNegativeFinite(std::initializer_list<float> values) {
    for (const float value : values) {
        if (!IsFinite(value) || value < 0.0F) {
            return false;
        }
    }
    return true;
}

}  // namespace

bool ValidateShadowOptions(const proto::ShadowOptions& options,
                           std::string* error) {
    if (error != nullptr) {
        error->clear();
    }
    if (options.detector_model_path().empty() ||
        options.policy_model_path().empty()) {
        SetError(
            error,
            "detector_model_path and policy_model_path must not be empty.");
        return false;
    }
    if (!HasSupportedBackend(options.detector_backend()) ||
        !HasSupportedBackend(options.policy_backend())) {
        SetError(error,
                 "detector_backend and policy_backend must be 'onnx' or "
                 "'tensorrt'.");
        return false;
    }
    if (options.detector_width() == 0 || options.detector_height() == 0 ||
        options.max_detections() == 0 || options.policy_width() == 0 ||
        options.policy_height() == 0 || options.candidate_count() == 0 ||
        options.trajectory_steps() == 0) {
        SetError(error,
                 "model dimensions and candidate counts must be positive.");
        return false;
    }
    if (options.person_class_id() < 0) {
        SetError(error, "person_class_id must be non-negative.");
        return false;
    }
    if (!IsUnitInterval(options.confidence_threshold()) ||
        !IsUnitInterval(options.track_high_threshold()) ||
        !IsUnitInterval(options.track_low_threshold()) ||
        !IsUnitInterval(options.association_iou_threshold())) {
        SetError(
            error,
            "confidence and association thresholds must be within [0, 1].");
        return false;
    }
    if (options.track_low_threshold() > options.track_high_threshold()) {
        SetError(error,
                 "track_low_threshold must not exceed track_high_threshold.");
        return false;
    }
    if (!IsPositive(options.prediction_timeout_sec()) ||
        !IsPositive(options.lost_timeout_sec()) ||
        !IsPositive(options.cell_ttl_sec()) ||
        !IsPositive(options.max_input_skew_sec()) ||
        !IsPositive(options.max_data_age_sec())) {
        SetError(error, "timeouts and age limits must be finite and positive.");
        return false;
    }
    if (!IsPositive(options.inner_box_scale()) ||
        options.inner_box_scale() > 1.0F) {
        SetError(error, "inner_box_scale must be finite and within (0, 1].");
        return false;
    }
    if (!IsPositive(options.min_depth_m()) ||
        !IsPositive(options.max_depth_m()) ||
        options.max_depth_m() <= options.min_depth_m() ||
        options.min_depth_samples() == 0 ||
        !IsPositive(options.depth_outlier_m()) ||
        !IsPositive(options.depth_scale())) {
        SetError(error,
                 "depth range and sampling limits must be valid and positive.");
        return false;
    }
    if (options.map_frame().empty() || options.base_frame().empty() ||
        options.camera_frame().empty()) {
        SetError(error,
                 "map_frame, base_frame, and camera_frame must not be empty.");
        return false;
    }
    if (!IsPositive(options.map_length_x()) ||
        !IsPositive(options.map_length_y()) ||
        !IsPositive(options.map_resolution()) ||
        !IsPositive(options.map_roll_threshold()) ||
        !IsPositive(options.max_step_height()) ||
        !IsPositive(options.max_slope_rad()) ||
        !IsPositive(options.obstacle_min_height())) {
        SetError(
            error,
            "map geometry and obstacle limits must be finite and positive.");
        return false;
    }
    if (!IsPositive(options.robot_radius()) ||
        !IsPositive(options.inflation_radius()) ||
        options.inflation_radius() < options.robot_radius()) {
        SetError(error,
                 "inflation_radius must be finite and no smaller than "
                 "robot_radius.");
        return false;
    }
    if (!IsPositive(options.max_linear_speed()) ||
        !IsPositive(options.max_angular_speed()) ||
        !IsPositive(options.trajectory_step_sec()) ||
        !IsPositive(options.follow_distance())) {
        SetError(error,
                 "kinematic bounds, trajectory_step_sec, and follow_distance "
                 "must be finite and positive.");
        return false;
    }
    const std::initializer_list<float> planner_weights = {
        options.learned_weight(),        options.clearance_weight(),
        options.traversability_weight(), options.curvature_weight(),
        options.progress_weight(),       options.distance_weight(),
        options.visibility_weight(),
    };
    if (!AllNonNegativeFinite(planner_weights)) {
        SetError(error, "planner weights must be finite and non-negative.");
        return false;
    }
    if (!AnyPositive(planner_weights)) {
        SetError(error, "at least one planner weight must be positive.");
        return false;
    }
    if (options.select_topic().empty() || options.detections_topic().empty() ||
        options.target_topic().empty() || options.path_topic().empty() ||
        options.grid_topic().empty()) {
        SetError(error, "select and output topics must not be empty.");
        return false;
    }
    return true;
}

}  // namespace shadow
}  // namespace perception
}  // namespace autonomy
