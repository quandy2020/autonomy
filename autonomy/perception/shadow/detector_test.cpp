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
 * @file detector_test.cpp
 * @brief Contract tests for Shadow YOLO person preprocessing and parsing.
 */

#include "autonomy/perception/shadow/detector.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autonomy {
namespace perception {
namespace shadow {
namespace {

class FakeRunner final : public DetectorRunner
{
public:
    explicit FakeRunner(std::vector<float> output, bool succeeds = true)
        : output_(std::move(output)), succeeds_(succeeds) {}

    bool Run(const common::network::TensorMap& inputs,
             common::network::TensorMap* outputs, std::string* error) override {
        captured_inputs_ = inputs;
        if (outputs != nullptr) {
            outputs->clear();
        }
        if (!succeeds_) {
            if (error != nullptr) {
                *error = "fake inference failure";
            }
            return false;
        }
        if (outputs != nullptr) {
            outputs->emplace("output0",
                             common::network::Tensor::FromFloat32(output_));
        }
        return true;
    }

    const common::network::TensorMap& captured_inputs() const {
        return captured_inputs_;
    }

private:
    std::vector<float> output_;
    bool succeeds_;
    common::network::TensorMap captured_inputs_;
};

proto::ShadowOptions Options(uint32_t width = 640, uint32_t height = 640,
                             uint32_t max_detections = 1) {
    proto::ShadowOptions options;
    options.set_detector_model_path("/models/shadow_detector.onnx");
    options.set_policy_model_path("/models/shadow_policy.onnx");
    options.set_detector_backend("onnx");
    options.set_policy_backend("onnx");
    options.set_detector_width(width);
    options.set_detector_height(height);
    options.set_max_detections(max_detections);
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
    options.set_select_topic("/shadow/select");
    options.set_detections_topic("/shadow/detections");
    options.set_target_topic("/shadow/target");
    options.set_path_topic("/shadow/path");
    options.set_grid_topic("/shadow/grid");
    options.set_max_input_skew_sec(0.05F);
    options.set_max_data_age_sec(0.20F);
    options.set_depth_scale(0.001F);
    return options;
}

automsgs::msgs::sensor_msgs::Image MakeImage(const std::string& encoding,
                                             uint32_t width, uint32_t height,
                                             std::vector<uint8_t> pixels) {
    automsgs::msgs::sensor_msgs::Image image;
    image.mutable_header()->set_frame_id("camera_optical");
    image.set_encoding(encoding);
    image.set_width(width);
    image.set_height(height);
    image.set_step(width * 3);
    image.set_is_bigendian(false);
    image.mutable_data()->assign(reinterpret_cast<const char*>(pixels.data()),
                                 pixels.size());
    return image;
}

std::unique_ptr<FakeRunner> MakeRunner(std::vector<float> output,
                                       bool succeeds = true) {
    return std::make_unique<FakeRunner>(std::move(output), succeeds);
}

TEST(YoloDetectorTest, RestoresLetterboxedPersonBox) {
    auto runner = MakeRunner({100.0F, 160.0F, 300.0F, 480.0F, 0.9F, 0.0F});
    auto detector = YoloDetector::Create(Options(), std::move(runner));
    automsgs::msgs::vision_msgs::Detection2DArray detections;

    ASSERT_NE(detector, nullptr);
    ASSERT_TRUE(detector->Detect(
        MakeImage("rgb8", 640, 320, std::vector<uint8_t>(640 * 320 * 3)),
        &detections));
    ASSERT_EQ(detections.detections_size(), 1);
    const auto& detection = detections.detections(0);
    ASSERT_EQ(detection.results_size(), 1);
    EXPECT_EQ(detection.results(0).hypothesis().class_id(), "person");
    EXPECT_FLOAT_EQ(detection.results(0).hypothesis().score(), 0.9F);
    EXPECT_DOUBLE_EQ(detection.bbox().center().position().x(), 200.0);
    EXPECT_DOUBLE_EQ(detection.bbox().center().position().y(), 160.0);
    EXPECT_DOUBLE_EQ(detection.bbox().size_x(), 200.0);
    EXPECT_DOUBLE_EQ(detection.bbox().size_y(), 320.0);
    EXPECT_TRUE(detection.id().empty());
    EXPECT_EQ(detections.header().frame_id(), "camera_optical");
}

TEST(YoloDetectorTest, ConvertsRgbAndBgrToNormalizedNchwWithLetterboxPadding) {
    const std::vector<float> output = {0.0F, 0.0F, 1.0F, 1.0F, 0.0F, 0.0F};
    auto rgb_runner = MakeRunner(output);
    FakeRunner* rgb_runner_ptr = rgb_runner.get();
    auto rgb_detector =
        YoloDetector::Create(Options(2, 4), std::move(rgb_runner));
    automsgs::msgs::vision_msgs::Detection2DArray detections;

    ASSERT_NE(rgb_detector, nullptr);
    ASSERT_TRUE(rgb_detector->Detect(
        MakeImage("rgb8", 2, 1, {255, 0, 0, 0, 255, 0}), &detections));
    const float* rgb =
        rgb_runner_ptr->captured_inputs().at("images").data_as<float>();
    ASSERT_NE(rgb, nullptr);
    constexpr float kPadding = 114.0F / 255.0F;
    EXPECT_FLOAT_EQ(rgb[0], kPadding);
    EXPECT_FLOAT_EQ(rgb[2], 1.0F);
    EXPECT_FLOAT_EQ(rgb[3], 0.0F);
    EXPECT_FLOAT_EQ(rgb[6], kPadding);
    EXPECT_FLOAT_EQ(rgb[7], kPadding);
    EXPECT_FLOAT_EQ(rgb[8], kPadding);
    EXPECT_FLOAT_EQ(rgb[10], 0.0F);
    EXPECT_FLOAT_EQ(rgb[11], 1.0F);
    EXPECT_FLOAT_EQ(rgb[16], kPadding);
    EXPECT_FLOAT_EQ(rgb[18], 0.0F);
    EXPECT_FLOAT_EQ(rgb[19], 0.0F);

    auto bgr_runner = MakeRunner(output);
    FakeRunner* bgr_runner_ptr = bgr_runner.get();
    auto bgr_detector =
        YoloDetector::Create(Options(2, 4), std::move(bgr_runner));
    ASSERT_NE(bgr_detector, nullptr);
    ASSERT_TRUE(bgr_detector->Detect(
        MakeImage("bgr8", 2, 1, {0, 0, 255, 0, 255, 0}), &detections));
    const float* bgr =
        bgr_runner_ptr->captured_inputs().at("images").data_as<float>();
    ASSERT_NE(bgr, nullptr);
    EXPECT_FLOAT_EQ(bgr[2], 1.0F);
    EXPECT_FLOAT_EQ(bgr[3], 0.0F);
    EXPECT_FLOAT_EQ(bgr[6], kPadding);
    EXPECT_FLOAT_EQ(bgr[7], kPadding);
    EXPECT_FLOAT_EQ(bgr[10], 0.0F);
    EXPECT_FLOAT_EQ(bgr[11], 1.0F);
    EXPECT_FLOAT_EQ(bgr[18], 0.0F);
    EXPECT_FLOAT_EQ(bgr[19], 0.0F);
}

TEST(YoloDetectorTest, FiltersClassAndKeepsThresholdEquality) {
    auto runner = MakeRunner({0.0F, 0.0F, 100.0F, 100.0F, 0.35F, 0.0F, 0.0F,
                              0.0F, 100.0F, 100.0F, 0.9F, 1.0F});
    auto detector =
        YoloDetector::Create(Options(640, 640, 2), std::move(runner));
    automsgs::msgs::vision_msgs::Detection2DArray detections;

    ASSERT_NE(detector, nullptr);
    ASSERT_TRUE(detector->Detect(
        MakeImage("rgb8", 640, 640, std::vector<uint8_t>(640 * 640 * 3)),
        &detections));
    ASSERT_EQ(detections.detections_size(), 1);
    EXPECT_FLOAT_EQ(detections.detections(0).results(0).hypothesis().score(),
                    0.35F);
}

TEST(YoloDetectorTest, ClipsRestoredBoxesToImageBounds) {
    auto runner = MakeRunner({-100.0F, 60.0F, 700.0F, 500.0F, 0.9F, 0.0F});
    auto detector = YoloDetector::Create(Options(), std::move(runner));
    automsgs::msgs::vision_msgs::Detection2DArray detections;

    ASSERT_NE(detector, nullptr);
    ASSERT_TRUE(detector->Detect(
        MakeImage("rgb8", 640, 320, std::vector<uint8_t>(640 * 320 * 3)),
        &detections));
    ASSERT_EQ(detections.detections_size(), 1);
    EXPECT_DOUBLE_EQ(detections.detections(0).bbox().center().position().x(),
                     320.0);
    EXPECT_DOUBLE_EQ(detections.detections(0).bbox().center().position().y(),
                     160.0);
    EXPECT_DOUBLE_EQ(detections.detections(0).bbox().size_x(), 640.0);
    EXPECT_DOUBLE_EQ(detections.detections(0).bbox().size_y(), 320.0);
}

TEST(YoloDetectorTest, RejectsInvalidBoxesWithoutFailingFrame) {
    auto runner = MakeRunner({100.0F, 100.0F, 100.0F, 200.0F, 0.9F, 0.0F});
    auto detector = YoloDetector::Create(Options(), std::move(runner));
    automsgs::msgs::vision_msgs::Detection2DArray detections;

    ASSERT_NE(detector, nullptr);
    ASSERT_TRUE(detector->Detect(
        MakeImage("rgb8", 640, 640, std::vector<uint8_t>(640 * 640 * 3)),
        &detections));
    EXPECT_EQ(detections.detections_size(), 0);
}

TEST(YoloDetectorTest, RejectsWrongOutputTensorSizeAndClearsOutput) {
    auto runner = MakeRunner({0.0F, 0.0F, 100.0F, 100.0F, 0.9F, 0.0F});
    auto detector =
        YoloDetector::Create(Options(640, 640, 2), std::move(runner));
    automsgs::msgs::vision_msgs::Detection2DArray detections;
    detections.add_detections();
    std::string error;

    ASSERT_NE(detector, nullptr);
    EXPECT_FALSE(detector->Detect(
        MakeImage("rgb8", 640, 640, std::vector<uint8_t>(640 * 640 * 3)),
        &detections, &error));
    EXPECT_TRUE(detections.detections().empty());
    EXPECT_FALSE(error.empty());
}

TEST(YoloDetectorTest, ClearsOutputWhenRunnerFails) {
    auto runner = MakeRunner({}, false);
    auto detector = YoloDetector::Create(Options(), std::move(runner));
    automsgs::msgs::vision_msgs::Detection2DArray detections;
    detections.add_detections();
    std::string error;

    ASSERT_NE(detector, nullptr);
    EXPECT_FALSE(detector->Detect(
        MakeImage("rgb8", 640, 640, std::vector<uint8_t>(640 * 640 * 3)),
        &detections, &error));
    EXPECT_TRUE(detections.detections().empty());
    EXPECT_EQ(error, "Shadow detector: fake inference failure");
}

}  // namespace
}  // namespace shadow
}  // namespace perception
}  // namespace autonomy
