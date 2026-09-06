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
 * @brief Contract tests for Hestia open/home detector preprocessing and parsing.
 */

#include "autonomy/perception/hestia/detector.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autonomy {
namespace perception {
namespace hestia {
namespace {

class FakeRunner final : public Runner
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

proto::HestiaOptions OpenOptions(uint32_t width = 640, uint32_t height = 640,
                                 uint32_t max_detections = 1) {
    proto::HestiaOptions o;
    o.set_mode(proto::MODE_OPEN);
    o.set_open_model_path("/models/hestia_open.onnx");
    o.set_backend(proto::BACKEND_ONNX);
    o.set_open_width(width);
    o.set_open_height(height);
    o.set_max_detections(max_detections);
    o.set_confidence_threshold(0.25F);
    o.add_open_prompts("chair");
    o.add_open_prompts("cup");
    o.set_depth_scale(0.001F);
    o.set_min_depth_m(0.2F);
    o.set_max_depth_m(8.0F);
    o.set_min_depth_samples(12);
    o.set_inner_box_scale(0.5F);
    o.set_depth_outlier_m(0.25F);
    o.set_camera_frame("camera_optical");
    o.set_base_frame("base_link");
    o.set_association_iou_threshold(0.3F);
    o.set_lost_timeout_sec(1.5F);
    o.set_merge_iou_threshold(0.5F);
    o.set_detections_2d_topic("/perception/hestia/detections_2d");
    o.set_detections_3d_topic("/perception/hestia/detections_3d");
    o.set_max_input_skew_sec(0.05F);
    o.set_nms_iou_threshold(0.0F);
    o.set_tf_timeout_sec(0.05F);
    return o;
}

proto::HestiaOptions DualOptions() {
    auto o = OpenOptions();
    o.set_mode(proto::MODE_DUAL);
    o.set_home_model_path("/models/hestia_home.onnx");
    o.set_home_width(640);
    o.set_home_height(640);
    o.add_home_labels("person");
    o.add_home_labels("chair");
    return o;
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

TEST(OpenDetectorTest, MapsClassIndexToOpenPromptLabel) {
    auto runner =
        std::make_unique<FakeRunner>(std::vector<float>{100.0F, 80.0F, 200.0F,
                                                        180.0F, 0.9F, 0.0F});
    auto detector = OpenDetector::Create(OpenOptions(), std::move(runner));
    automsgs::msgs::vision_msgs::Detection2DArray detections;

    ASSERT_NE(detector, nullptr);
    ASSERT_TRUE(detector->Detect(
        MakeImage("rgb8", 640, 640, std::vector<uint8_t>(640 * 640 * 3)),
        &detections));
    ASSERT_EQ(detections.detections_size(), 1);
    EXPECT_EQ(detections.detections(0).results(0).hypothesis().class_id(),
              "chair");
    EXPECT_FLOAT_EQ(detections.detections(0).results(0).hypothesis().score(),
                    0.9F);
    EXPECT_NEAR(detections.detections(0).bbox().center().position().x(), 150.0,
                1.0);
    EXPECT_NEAR(detections.detections(0).bbox().center().position().y(), 130.0,
                1.0);
    EXPECT_TRUE(detections.detections(0).id().empty());
}

TEST(OpenDetectorTest, RestoresLetterboxedBoxOnNonSquareImage) {
    // Same fixture geometry as Shadow: 640x320 source into 640x640 letterbox.
    auto runner = std::make_unique<FakeRunner>(
        std::vector<float>{100.0F, 160.0F, 300.0F, 480.0F, 0.9F, 0.0F});
    auto detector = OpenDetector::Create(OpenOptions(), std::move(runner));
    automsgs::msgs::vision_msgs::Detection2DArray detections;

    ASSERT_NE(detector, nullptr);
    ASSERT_TRUE(detector->Detect(
        MakeImage("rgb8", 640, 320, std::vector<uint8_t>(640 * 320 * 3)),
        &detections));
    ASSERT_EQ(detections.detections_size(), 1);
    EXPECT_DOUBLE_EQ(detections.detections(0).bbox().center().position().x(),
                     200.0);
    EXPECT_DOUBLE_EQ(detections.detections(0).bbox().center().position().y(),
                     160.0);
    EXPECT_DOUBLE_EQ(detections.detections(0).bbox().size_x(), 200.0);
    EXPECT_DOUBLE_EQ(detections.detections(0).bbox().size_y(), 320.0);
}

TEST(OpenDetectorTest, ClearsOutputWhenRunnerFails) {
    auto runner = std::make_unique<FakeRunner>(std::vector<float>{}, false);
    auto detector = OpenDetector::Create(OpenOptions(), std::move(runner));
    automsgs::msgs::vision_msgs::Detection2DArray detections;
    detections.add_detections();
    std::string error;

    ASSERT_NE(detector, nullptr);
    EXPECT_FALSE(detector->Detect(
        MakeImage("rgb8", 640, 640, std::vector<uint8_t>(640 * 640 * 3)),
        &detections, &error));
    EXPECT_TRUE(detections.detections().empty());
    EXPECT_EQ(error, "Hestia detector: fake inference failure");
}

TEST(ClosedDetectorTest, MapsClassIndexToHomeLabel) {
    auto runner = std::make_unique<FakeRunner>(
        std::vector<float>{0.0F, 0.0F, 20.0F, 20.0F, 0.8F, 1.0F});
    auto detector = ClosedDetector::Create(DualOptions(), std::move(runner));
    automsgs::msgs::vision_msgs::Detection2DArray detections;

    ASSERT_NE(detector, nullptr);
    ASSERT_TRUE(detector->Detect(
        MakeImage("rgb8", 640, 640, std::vector<uint8_t>(640 * 640 * 3)),
        &detections));
    ASSERT_EQ(detections.detections_size(), 1);
    EXPECT_EQ(detections.detections(0).results(0).hypothesis().class_id(),
              "chair");
}

TEST(OpenDetectorTest, AppliesOptionalNms) {
    auto options = OpenOptions(640, 640, 2);
    options.set_nms_iou_threshold(0.5F);
    auto runner = std::make_unique<FakeRunner>(std::vector<float>{
        100.0F, 100.0F, 200.0F, 200.0F, 0.95F, 0.0F, 105.0F, 105.0F, 205.0F,
        205.0F, 0.90F, 0.0F});
    auto detector = OpenDetector::Create(options, std::move(runner));
    automsgs::msgs::vision_msgs::Detection2DArray detections;

    ASSERT_NE(detector, nullptr);
    ASSERT_TRUE(detector->Detect(
        MakeImage("rgb8", 640, 640, std::vector<uint8_t>(640 * 640 * 3)),
        &detections));
    EXPECT_EQ(detections.detections_size(), 1);
}

}  // namespace
}  // namespace hestia
}  // namespace perception
}  // namespace autonomy
