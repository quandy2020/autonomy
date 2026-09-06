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
 * @file options_test.cpp
 * @brief Contract tests for the single Shadow protobuf configuration model.
 */

#include "autonomy/perception/shadow/options.hpp"

#include <gtest/gtest.h>

#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace autonomy::perception::shadow {
namespace {

proto::ShadowOptions ValidOptions() {
    proto::ShadowOptions options;
    options.set_detector_model_path("/models/shadow_detector.onnx");
    options.set_policy_model_path("/models/shadow_policy.onnx");
    options.set_detector_backend("onnx");
    options.set_policy_backend("onnx");
    options.set_detector_width(640);
    options.set_detector_height(640);
    options.set_max_detections(300);
    options.set_person_class_id(0);
    options.set_confidence_threshold(0.35F);
    options.set_track_high_threshold(0.50F);
    options.set_track_low_threshold(0.10F);
    options.set_association_iou_threshold(0.30F);
    options.set_min_confirmed_hits(3);
    options.set_prediction_timeout_sec(0.35F);
    options.set_lost_timeout_sec(1.5F);
    options.set_inner_box_scale(0.50F);
    options.set_min_depth_m(0.20F);
    options.set_max_depth_m(8.0F);
    options.set_min_depth_samples(12);
    options.set_depth_outlier_m(0.25F);
    options.set_map_frame("map");
    options.set_base_frame("base_link");
    options.set_camera_frame("camera_link");
    options.set_map_length_x(10.0F);
    options.set_map_length_y(10.0F);
    options.set_map_resolution(0.05F);
    options.set_map_roll_threshold(0.35F);
    options.set_cell_ttl_sec(1.0F);
    options.set_max_step_height(0.20F);
    options.set_max_slope_rad(0.35F);
    options.set_obstacle_min_height(0.10F);
    options.set_robot_radius(0.25F);
    options.set_inflation_radius(0.35F);
    options.set_policy_width(160);
    options.set_policy_height(96);
    options.set_candidate_count(64);
    options.set_trajectory_steps(12);
    options.set_max_linear_speed(0.8F);
    options.set_max_angular_speed(1.2F);
    options.set_follow_distance(1.5F);
    options.set_trajectory_step_sec(0.1F);
    options.set_learned_weight(1.0F);
    options.set_clearance_weight(1.0F);
    options.set_traversability_weight(1.0F);
    options.set_curvature_weight(1.0F);
    options.set_progress_weight(1.0F);
    options.set_distance_weight(1.0F);
    options.set_visibility_weight(1.0F);
    options.set_select_topic("/perception/shadow/select");
    options.set_detections_topic("/perception/shadow/detections");
    options.set_target_topic("/perception/shadow/target");
    options.set_path_topic("/perception/shadow/path");
    options.set_grid_topic("/perception/shadow/grid");
    options.set_max_input_skew_sec(0.10F);
    options.set_max_data_age_sec(0.50F);
    options.set_depth_scale(0.001F);
    return options;
}

TEST(ShadowOptionsTest, AcceptsCompleteConfiguration) {
    const auto options = ValidOptions();
    std::string error = "stale error";

    EXPECT_TRUE(ValidateShadowOptions(options, &error)) << error;
    EXPECT_TRUE(error.empty());
}

TEST(ShadowOptionsTest, RejectsInvalidIndividualFields) {
    struct InvalidCase {
        std::string name;
        std::function<void(proto::ShadowOptions*)> mutate;
    };
    const std::vector<InvalidCase> cases = {
        {"empty detector model path",
         [](proto::ShadowOptions* options) {
             options->clear_detector_model_path();
         }},
        {"empty policy model path",
         [](proto::ShadowOptions* options) {
             options->clear_policy_model_path();
         }},
        {"unknown detector backend",
         [](proto::ShadowOptions* options) {
             options->set_detector_backend("unknown");
         }},
        {"unknown policy backend",
         [](proto::ShadowOptions* options) {
             options->set_policy_backend("unknown");
         }},
        {"zero detector width",
         [](proto::ShadowOptions* options) { options->set_detector_width(0); }},
        {"zero policy height",
         [](proto::ShadowOptions* options) { options->set_policy_height(0); }},
        {"invalid confidence threshold",
         [](proto::ShadowOptions* options) {
             options->set_confidence_threshold(1.01F);
         }},
        {"invalid high tracking threshold",
         [](proto::ShadowOptions* options) {
             options->set_track_high_threshold(-0.01F);
         }},
        {"invalid low tracking threshold",
         [](proto::ShadowOptions* options) {
             options->set_track_low_threshold(1.01F);
         }},
        {"invalid association threshold",
         [](proto::ShadowOptions* options) {
             options->set_association_iou_threshold(-0.01F);
         }},
        {"zero prediction timeout",
         [](proto::ShadowOptions* options) {
             options->set_prediction_timeout_sec(0.0F);
         }},
        {"zero lost timeout",
         [](proto::ShadowOptions* options) {
             options->set_lost_timeout_sec(0.0F);
         }},
        {"inverted depth range",
         [](proto::ShadowOptions* options) {
             options->set_max_depth_m(options->min_depth_m());
         }},
        {"zero map length",
         [](proto::ShadowOptions* options) {
             options->set_map_length_x(0.0F);
         }},
        {"zero map resolution",
         [](proto::ShadowOptions* options) {
             options->set_map_resolution(0.0F);
         }},
        {"zero trajectory step duration",
         [](proto::ShadowOptions* options) {
             options->set_trajectory_step_sec(0.0F);
         }},
        {"negative trajectory step duration",
         [](proto::ShadowOptions* options) {
             options->set_trajectory_step_sec(-0.1F);
         }},
        {"non-finite trajectory step duration",
         [](proto::ShadowOptions* options) {
             options->set_trajectory_step_sec(
                 std::numeric_limits<float>::infinity());
         }},
        {"negative learned planner weight",
         [](proto::ShadowOptions* options) {
             options->set_learned_weight(-0.01F);
         }},
        {"all zero planner weights",
         [](proto::ShadowOptions* options) {
             options->set_learned_weight(0.0F);
             options->set_clearance_weight(0.0F);
             options->set_traversability_weight(0.0F);
             options->set_curvature_weight(0.0F);
             options->set_progress_weight(0.0F);
             options->set_distance_weight(0.0F);
             options->set_visibility_weight(0.0F);
         }},
    };

    for (const auto& invalid_case : cases) {
        SCOPED_TRACE(invalid_case.name);
        auto options = ValidOptions();
        invalid_case.mutate(&options);
        std::string error;

        EXPECT_FALSE(ValidateShadowOptions(options, &error));
        EXPECT_FALSE(error.empty());
    }
}

TEST(ShadowOptionsTest, RejectsInflationSmallerThanRobot) {
    auto options = ValidOptions();
    options.set_robot_radius(0.30F);
    options.set_inflation_radius(0.20F);
    std::string error;
    EXPECT_FALSE(ValidateShadowOptions(options, &error));
    EXPECT_FALSE(error.empty());
}

}  // namespace
}  // namespace autonomy::perception::shadow
