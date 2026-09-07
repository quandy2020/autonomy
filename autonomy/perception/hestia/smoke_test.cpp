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
 * @file smoke_test.cpp
 * @brief Optional ONNX smoke: set HESTIA_SMOKE_MODEL to a real open model path.
 */

#include "autonomy/perception/hestia/detector.hpp"
#include "autonomy/perception/hestia/options.hpp"

#include <gtest/gtest.h>

#include <cstdlib>
#include <string>
#include <vector>

namespace autonomy {
namespace perception {
namespace hestia {
namespace {

TEST(HestiaSmokeTest, LoadsOpenModelWhenEnvSet) {
    const char* path = std::getenv("HESTIA_SMOKE_MODEL");
    if (path == nullptr || path[0] == '\0') {
        GTEST_SKIP() << "Set HESTIA_SMOKE_MODEL to run ONNX smoke.";
    }

    proto::HestiaOptions options;
    options.set_mode(proto::MODE_OPEN);
    options.set_open_model_path(path);
    options.set_backend(proto::BACKEND_ONNX);
    options.set_open_width(640);
    options.set_open_height(640);
    options.set_max_detections(100);
    options.set_confidence_threshold(0.25F);
    options.add_open_prompts("chair");
    options.set_depth_scale(0.001F);
    options.set_min_depth_m(0.2F);
    options.set_max_depth_m(8.0F);
    options.set_min_depth_samples(12);
    options.set_inner_box_scale(0.5F);
    options.set_depth_outlier_m(0.25F);
    options.set_camera_frame("camera_optical");
    options.set_association_iou_threshold(0.3F);
    options.set_lost_timeout_sec(1.5F);
    options.set_merge_iou_threshold(0.5F);
    options.set_detections_2d_topic("/a");
    options.set_detections_3d_topic("/b");
    options.set_max_input_skew_sec(0.05F);
    options.set_nms_iou_threshold(0.0F);
    options.set_tf_timeout_sec(0.05F);

    std::string error;
    ASSERT_TRUE(ValidateHestiaOptions(options, &error)) << error;
    auto detector = OpenDetector::Create(options, &error);
    ASSERT_NE(detector, nullptr) << error;

    automsgs::msgs::sensor_msgs::Image image;
    image.set_encoding("rgb8");
    image.set_width(640);
    image.set_height(640);
    image.set_step(640 * 3);
    image.mutable_data()->assign(640 * 640 * 3, '\0');
    automsgs::msgs::vision_msgs::Detection2DArray detections;
    EXPECT_TRUE(detector->Detect(image, &detections, &error)) << error;
}

}  // namespace
}  // namespace hestia
}  // namespace perception
}  // namespace autonomy
