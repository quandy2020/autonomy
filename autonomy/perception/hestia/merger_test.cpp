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
 * @file merger_test.cpp
 * @brief Tests for dual-mode detection merge.
 */

#include "autonomy/perception/hestia/merger.hpp"

#include <gtest/gtest.h>

namespace autonomy {
namespace perception {
namespace hestia {
namespace {

proto::HestiaOptions Options() {
    proto::HestiaOptions o;
    o.set_mode("dual");
    o.set_open_model_path("/m/open.onnx");
    o.set_home_model_path("/m/home.onnx");
    o.set_backend("onnx");
    o.set_open_width(640);
    o.set_open_height(640);
    o.set_home_width(640);
    o.set_home_height(640);
    o.set_max_detections(10);
    o.set_confidence_threshold(0.25F);
    o.add_open_prompts("cup");
    o.add_home_labels("chair");
    o.set_depth_scale(0.001F);
    o.set_min_depth_m(0.2F);
    o.set_max_depth_m(8.0F);
    o.set_min_depth_samples(3);
    o.set_inner_box_scale(0.5F);
    o.set_depth_outlier_m(0.25F);
    o.set_camera_frame("camera_optical");
    o.set_association_iou_threshold(0.3F);
    o.set_lost_timeout_sec(1.5F);
    o.set_merge_iou_threshold(0.5F);
    o.set_detections_2d_topic("/a");
    o.set_detections_3d_topic("/b");
    o.set_max_input_skew_sec(0.05F);
    o.set_max_data_age_sec(0.2F);
    return o;
}

automsgs::msgs::vision_msgs::Detection2D Det(double cx, double cy, double score,
                                             const std::string& label) {
    automsgs::msgs::vision_msgs::Detection2D d;
    d.add_results()->mutable_hypothesis()->set_class_id(label);
    d.mutable_results(0)->mutable_hypothesis()->set_score(score);
    d.mutable_bbox()->mutable_center()->mutable_position()->set_x(cx);
    d.mutable_bbox()->mutable_center()->mutable_position()->set_y(cy);
    d.mutable_bbox()->set_size_x(40);
    d.mutable_bbox()->set_size_y(40);
    return d;
}

TEST(DetectionMergerTest, KeepsHigherScoreOnOverlap) {
    DetectionMerger merger(Options());
    automsgs::msgs::vision_msgs::Detection2DArray home;
    automsgs::msgs::vision_msgs::Detection2DArray open;
    *home.add_detections() = Det(100, 100, 0.6, "chair");
    *open.add_detections() = Det(102, 101, 0.9, "cup");
    automsgs::msgs::vision_msgs::Detection2DArray out;
    merger.Merge(home, open, &out);
    ASSERT_EQ(out.detections_size(), 1);
    EXPECT_EQ(out.detections(0).results(0).hypothesis().class_id(), "cup");
}

TEST(DetectionMergerTest, ConcatenatesNonOverlapping) {
    DetectionMerger merger(Options());
    automsgs::msgs::vision_msgs::Detection2DArray home;
    automsgs::msgs::vision_msgs::Detection2DArray open;
    *home.add_detections() = Det(100, 100, 0.6, "chair");
    *open.add_detections() = Det(400, 400, 0.7, "cup");
    automsgs::msgs::vision_msgs::Detection2DArray out;
    merger.Merge(home, open, &out);
    EXPECT_EQ(out.detections_size(), 2);
}

}  // namespace
}  // namespace hestia
}  // namespace perception
}  // namespace autonomy
