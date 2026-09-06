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
 * @file tracker_test.cpp
 * @brief Contract tests for Hestia multi-class object tracking.
 */

#include "autonomy/perception/hestia/tracker.hpp"

#include <gtest/gtest.h>

#include <string>

namespace autonomy {
namespace perception {
namespace hestia {
namespace {

proto::HestiaOptions Options() {
    proto::HestiaOptions o;
    o.set_mode(proto::MODE_OPEN);
    o.set_open_model_path("/models/hestia_open.onnx");
    o.set_backend(proto::BACKEND_ONNX);
    o.set_open_width(640);
    o.set_open_height(640);
    o.set_max_detections(10);
    o.set_confidence_threshold(0.25F);
    o.add_open_prompts("chair");
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
    o.set_nms_iou_threshold(0.0F);
    o.set_tf_timeout_sec(0.05F);
    return o;
}

automsgs::msgs::vision_msgs::Detection2D MakeDet(double cx, double cy,
                                                 const std::string& label) {
    automsgs::msgs::vision_msgs::Detection2D detection;
    detection.add_results()->mutable_hypothesis()->set_class_id(label);
    detection.mutable_bbox()->mutable_center()->mutable_position()->set_x(cx);
    detection.mutable_bbox()->mutable_center()->mutable_position()->set_y(cy);
    detection.mutable_bbox()->set_size_x(40.0);
    detection.mutable_bbox()->set_size_y(40.0);
    return detection;
}

TEST(TrackerTest, ReusesIdForSameBox) {
    Tracker tracker(Options());
    automsgs::msgs::vision_msgs::Detection2DArray frame;
    *frame.add_detections() = MakeDet(100, 100, "chair");
    tracker.Associate(1.0, &frame);
    const std::string first_id = frame.detections(0).id();
    EXPECT_FALSE(first_id.empty());

    frame.Clear();
    *frame.add_detections() = MakeDet(102, 101, "chair");
    tracker.Associate(1.1, &frame);
    EXPECT_EQ(frame.detections(0).id(), first_id);
}

TEST(TrackerTest, AssignsNewIdForDistantBox) {
    Tracker tracker(Options());
    automsgs::msgs::vision_msgs::Detection2DArray frame;
    *frame.add_detections() = MakeDet(100, 100, "chair");
    tracker.Associate(1.0, &frame);
    const std::string first_id = frame.detections(0).id();

    frame.Clear();
    *frame.add_detections() = MakeDet(400, 400, "chair");
    tracker.Associate(1.1, &frame);
    EXPECT_NE(frame.detections(0).id(), first_id);
}

TEST(TrackerTest, ExpiresTrackAfterTimeout) {
    Tracker tracker(Options());
    automsgs::msgs::vision_msgs::Detection2DArray frame;
    *frame.add_detections() = MakeDet(100, 100, "chair");
    tracker.Associate(1.0, &frame);
    const std::string first_id = frame.detections(0).id();

    frame.Clear();
    tracker.Associate(3.0, &frame);  // lost_timeout 1.5s
    *frame.add_detections() = MakeDet(100, 100, "chair");
    tracker.Associate(3.1, &frame);
    EXPECT_NE(frame.detections(0).id(), first_id);
}

TEST(TrackerTest, HandlesEmptyFrame) {
    Tracker tracker(Options());
    automsgs::msgs::vision_msgs::Detection2DArray frame;
    tracker.Associate(1.0, &frame);
    EXPECT_EQ(frame.detections_size(), 0);
}

TEST(TrackerTest, RejectsDifferentClassOverlap) {
    Tracker tracker(Options());
    automsgs::msgs::vision_msgs::Detection2DArray frame;
    *frame.add_detections() = MakeDet(100, 100, "chair");
    tracker.Associate(1.0, &frame);
    const std::string first_id = frame.detections(0).id();

    frame.Clear();
    *frame.add_detections() = MakeDet(102, 101, "cup");
    tracker.Associate(1.1, &frame);
    EXPECT_NE(frame.detections(0).id(), first_id);
}

}  // namespace
}  // namespace hestia
}  // namespace perception
}  // namespace autonomy
