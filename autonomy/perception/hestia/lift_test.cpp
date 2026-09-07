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
 * @file lift_test.cpp
 * @brief Contract tests for Hestia RGB-D 3D AABB lifting.
 */

#include "autonomy/perception/hestia/lift.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace autonomy {
namespace perception {
namespace hestia {
namespace {

using CameraInfo = automsgs::msgs::sensor_msgs::CameraInfo;
using Detection2D = automsgs::msgs::vision_msgs::Detection2D;
using Detection2DArray = automsgs::msgs::vision_msgs::Detection2DArray;
using Image = automsgs::msgs::sensor_msgs::Image;
using TransformStamped = automsgs::msgs::geometry_msgs::TransformStamped;

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
    o.set_max_depth_m(10.0F);
    o.set_min_depth_samples(3);
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

Detection2D Box(double cx, double cy, double w, double h) {
    Detection2D detection;
    detection.mutable_header()->set_frame_id("camera_optical");
    detection.set_id("t1");
    auto* hyp = detection.add_results()->mutable_hypothesis();
    hyp->set_class_id("chair");
    hyp->set_score(0.9);
    auto* bbox = detection.mutable_bbox();
    bbox->mutable_center()->mutable_position()->set_x(cx);
    bbox->mutable_center()->mutable_position()->set_y(cy);
    bbox->set_size_x(w);
    bbox->set_size_y(h);
    return detection;
}

CameraInfo Camera(uint32_t width = 6, uint32_t height = 6, double fx = 1.0,
                  double fy = 1.0, double cx = 3.0, double cy = 3.0) {
    CameraInfo camera;
    camera.mutable_header()->set_frame_id("camera_optical");
    camera.set_width(width);
    camera.set_height(height);
    camera.add_k(fx);
    camera.add_k(0.0);
    camera.add_k(cx);
    camera.add_k(0.0);
    camera.add_k(fy);
    camera.add_k(cy);
    camera.add_k(0.0);
    camera.add_k(0.0);
    camera.add_k(1.0);
    return camera;
}

template <typename T>
Image DepthImage(const std::string& encoding, uint32_t width, uint32_t height,
                 const std::vector<T>& values) {
    Image image;
    image.mutable_header()->set_frame_id("camera_optical");
    image.set_encoding(encoding);
    image.set_width(width);
    image.set_height(height);
    image.set_is_bigendian(false);
    image.set_step(width * static_cast<uint32_t>(sizeof(T)));
    std::string data(static_cast<size_t>(height) * image.step(), '\0');
    for (uint32_t row = 0; row < height; ++row) {
        std::memcpy(data.data() + static_cast<size_t>(row) * image.step(),
                    values.data() + static_cast<size_t>(row) * width,
                    static_cast<size_t>(width) * sizeof(T));
    }
    image.set_data(data);
    return image;
}

TEST(LifterTest, LiftsConstantDepthPlane) {
    Lifter lifter(Options());
    Detection2DArray detections_2d;
    *detections_2d.add_detections() = Box(3.0, 3.0, 4.0, 4.0);
    const auto depth =
        DepthImage<float>("32FC1", 6, 6, std::vector<float>(36, 2.0F));
    automsgs::msgs::vision_msgs::Detection3DArray detections_3d;

    ASSERT_TRUE(lifter.Lift(detections_2d, depth, Camera(), nullptr,
                            &detections_3d));
    ASSERT_EQ(detections_3d.detections_size(), 1);
    EXPECT_NEAR(detections_3d.detections(0).bbox().center().position().z(), 2.0,
                1e-3);
    EXPECT_GT(detections_3d.detections(0).bbox().size().x(), 0.0);
    EXPECT_GT(detections_3d.detections(0).bbox().size().y(), 0.0);
    EXPECT_GT(detections_3d.detections(0).bbox().size().z(), 0.0);
    EXPECT_EQ(detections_3d.detections(0).id(), "t1");
    EXPECT_EQ(detections_3d.detections(0).results(0).hypothesis().class_id(),
              "chair");
    EXPECT_EQ(detections_3d.header().frame_id(), "camera_optical");
}

TEST(LifterTest, OmitsBoxesWithTooFewSamples) {
    auto options = Options();
    options.set_min_depth_samples(1000);
    Lifter lifter(options);
    Detection2DArray detections_2d;
    *detections_2d.add_detections() = Box(3.0, 3.0, 4.0, 4.0);
    const auto depth =
        DepthImage<float>("32FC1", 6, 6, std::vector<float>(36, 2.0F));
    automsgs::msgs::vision_msgs::Detection3DArray detections_3d;

    ASSERT_TRUE(lifter.Lift(detections_2d, depth, Camera(), nullptr,
                            &detections_3d));
    EXPECT_EQ(detections_3d.detections_size(), 0);
}

TEST(LifterTest, Scales16BitDepth) {
    Lifter lifter(Options());
    Detection2DArray detections_2d;
    *detections_2d.add_detections() = Box(3.0, 3.0, 4.0, 4.0);
    const auto depth =
        DepthImage<uint16_t>("16UC1", 6, 6, std::vector<uint16_t>(36, 2000));
    automsgs::msgs::vision_msgs::Detection3DArray detections_3d;

    ASSERT_TRUE(lifter.Lift(detections_2d, depth, Camera(), nullptr,
                            &detections_3d));
    ASSERT_EQ(detections_3d.detections_size(), 1);
    EXPECT_NEAR(detections_3d.detections(0).bbox().center().position().z(), 2.0,
                1e-3);
}

TEST(LifterTest, UsesBaseFrameWhenTransformValid) {
    Lifter lifter(Options());
    Detection2DArray detections_2d;
    *detections_2d.add_detections() = Box(3.0, 3.0, 4.0, 4.0);
    const auto depth =
        DepthImage<float>("32FC1", 6, 6, std::vector<float>(36, 2.0F));
    TransformStamped transform;
    transform.mutable_header()->set_frame_id("base_link");
    transform.set_child_frame_id("camera_optical");
    transform.mutable_transform()->mutable_rotation()->set_w(1.0);
    automsgs::msgs::vision_msgs::Detection3DArray detections_3d;

    ASSERT_TRUE(lifter.Lift(detections_2d, depth, Camera(), &transform,
                            &detections_3d));
    EXPECT_EQ(detections_3d.header().frame_id(), "base_link");
    EXPECT_EQ(detections_3d.detections(0).header().frame_id(), "base_link");
}

TEST(LifterTest, FallsBackToCameraWithoutTransform) {
    Lifter lifter(Options());
    Detection2DArray detections_2d;
    *detections_2d.add_detections() = Box(3.0, 3.0, 4.0, 4.0);
    const auto depth =
        DepthImage<float>("32FC1", 6, 6, std::vector<float>(36, 2.0F));
    automsgs::msgs::vision_msgs::Detection3DArray detections_3d;

    ASSERT_TRUE(lifter.Lift(detections_2d, depth, Camera(), nullptr,
                            &detections_3d));
    EXPECT_EQ(detections_3d.header().frame_id(), "camera_optical");
}

}  // namespace
}  // namespace hestia
}  // namespace perception
}  // namespace autonomy
