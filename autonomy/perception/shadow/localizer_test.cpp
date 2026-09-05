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
 * @file localizer_test.cpp
 * @brief Contract tests for robust Shadow RGB-D target localization.
 */

#include "autonomy/perception/shadow/localizer.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

namespace autonomy {
namespace perception {
namespace shadow {
namespace {

using CameraInfo = automsgs::msgs::sensor_msgs::CameraInfo;
using Detection2D = automsgs::msgs::vision_msgs::Detection2D;
using Image = automsgs::msgs::sensor_msgs::Image;
using PoseStamped = automsgs::msgs::geometry_msgs::PoseStamped;
using TransformStamped = automsgs::msgs::geometry_msgs::TransformStamped;
using TwistStamped = automsgs::msgs::geometry_msgs::TwistStamped;

proto::ShadowOptions ValidOptions() {
    proto::ShadowOptions options;
    options.set_inner_box_scale(0.50F);
    options.set_min_depth_m(0.20F);
    options.set_max_depth_m(10.0F);
    options.set_min_depth_samples(3);
    options.set_depth_outlier_m(0.25F);
    options.set_depth_scale(0.001F);
    options.set_map_frame("map");
    options.set_camera_frame("camera_optical");
    return options;
}

Detection2D Box(double center_x = 3.0, double center_y = 3.0,
                double width = 4.0, double height = 4.0) {
    Detection2D detection;
    detection.mutable_header()->set_frame_id("camera_optical");
    auto* bbox = detection.mutable_bbox();
    bbox->mutable_center()->mutable_position()->set_x(center_x);
    bbox->mutable_center()->mutable_position()->set_y(center_y);
    bbox->set_size_x(width);
    bbox->set_size_y(height);
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
                 const std::vector<T>& values, uint32_t row_padding = 0) {
    Image image;
    image.mutable_header()->set_frame_id("camera_optical");
    image.set_encoding(encoding);
    image.set_width(width);
    image.set_height(height);
    image.set_is_bigendian(false);
    image.set_step(width * static_cast<uint32_t>(sizeof(T)) + row_padding);
    std::string data(static_cast<size_t>(height) * image.step(), '\0');
    for (uint32_t row = 0; row < height; ++row) {
        const size_t source_offset = static_cast<size_t>(row) * width;
        const size_t destination_offset =
            static_cast<size_t>(row) * image.step();
        std::memcpy(data.data() + destination_offset,
                    values.data() + source_offset,
                    static_cast<size_t>(width) * sizeof(T));
    }
    image.set_data(data);
    return image;
}

Image DepthAt(float depth_m) {
    return DepthImage<float>("32FC1", 6, 6, std::vector<float>(36, depth_m));
}

TransformStamped CameraToMap(int64_t stamp_ns, double x = 0.0, double y = 0.0,
                             double z = 0.0, double qx = 0.0, double qy = 0.0,
                             double qz = 0.0, double qw = 1.0) {
    TransformStamped transform;
    transform.mutable_header()->set_frame_id("map");
    transform.set_child_frame_id("camera_optical");
    transform.mutable_header()->mutable_stamp()->set_sec(
        static_cast<int32_t>(stamp_ns / 1'000'000'000));
    transform.mutable_header()->mutable_stamp()->set_nanosec(
        static_cast<uint32_t>(stamp_ns % 1'000'000'000));
    transform.mutable_transform()->mutable_translation()->set_x(x);
    transform.mutable_transform()->mutable_translation()->set_y(y);
    transform.mutable_transform()->mutable_translation()->set_z(z);
    transform.mutable_transform()->mutable_rotation()->set_x(qx);
    transform.mutable_transform()->mutable_rotation()->set_y(qy);
    transform.mutable_transform()->mutable_rotation()->set_z(qz);
    transform.mutable_transform()->mutable_rotation()->set_w(qw);
    return transform;
}

TEST(TargetLocalizerTest, Scales16BitDepthAndHonorsRowStep) {
    const auto options = ValidOptions();
    TargetLocalizer localizer(options);
    const auto depth =
        DepthImage<uint16_t>("16UC1", 6, 6, std::vector<uint16_t>(36, 2250), 8);
    float range_m = 0.0F;
    std::string error;

    ASSERT_TRUE(
        localizer.EstimateRange(Box(), depth, Camera(), &range_m, &error))
        << error;
    EXPECT_FLOAT_EQ(range_m, 2.25F);
}

TEST(TargetLocalizerTest, Treats32BitFloatDepthAsMetric) {
    TargetLocalizer localizer(ValidOptions());
    float range_m = 0.0F;

    ASSERT_TRUE(
        localizer.EstimateRange(Box(), DepthAt(2.5F), Camera(), &range_m));
    EXPECT_FLOAT_EQ(range_m, 2.5F);
}

TEST(TargetLocalizerTest, SamplesOnlyConfiguredInnerBox) {
    TargetLocalizer localizer(ValidOptions());
    std::vector<float> values(36, 9.0F);
    values[2 * 6 + 2] = 2.0F;
    values[2 * 6 + 3] = 2.0F;
    values[3 * 6 + 2] = 2.0F;
    values[3 * 6 + 3] = 2.0F;
    const auto depth = DepthImage<float>("32FC1", 6, 6, values);
    float range_m = 0.0F;

    ASSERT_TRUE(localizer.EstimateRange(Box(), depth, Camera(), &range_m));
    EXPECT_FLOAT_EQ(range_m, 2.0F);
}

TEST(TargetLocalizerTest, DiscardsInvalidAndOutOfRangeSamples) {
    auto options = ValidOptions();
    options.set_inner_box_scale(1.0F);
    options.set_min_depth_samples(1);
    TargetLocalizer localizer(options);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();
    std::vector<float> values(7);
    values[0] = nan;
    values[1] = infinity;
    values[2] = -1.0F;
    values[3] = 0.0F;
    values[4] = 0.1F;
    values[5] = 2.0F;
    values[6] = 11.0F;
    const auto depth = DepthImage<float>("32FC1", 7, 1, values);
    float range_m = 0.0F;

    ASSERT_TRUE(localizer.EstimateRange(Box(3.5, 0.5, 7.0, 1.0), depth,
                                        Camera(7, 1, 1.0, 1.0, 3.5, 0.5),
                                        &range_m));
    EXPECT_FLOAT_EQ(range_m, 2.0F);
}

TEST(TargetLocalizerTest, RejectsDiscontinuityOutliersAndRecomputesMedian) {
    auto options = ValidOptions();
    options.set_inner_box_scale(1.0F);
    TargetLocalizer localizer(options);
    const auto depth = DepthImage<float>(
        "32FC1", 5, 1, std::vector<float>{1.9F, 2.0F, 2.1F, 8.0F, 8.0F});
    float range_m = 0.0F;

    ASSERT_TRUE(localizer.EstimateRange(Box(2.5, 0.5, 5.0, 1.0), depth,
                                        Camera(5, 1, 1.0, 1.0, 2.5, 0.5),
                                        &range_m));
    EXPECT_FLOAT_EQ(range_m, 2.0F);
}

TEST(TargetLocalizerTest, RejectsTooFewSurvivingDepthSamples) {
    auto options = ValidOptions();
    options.set_inner_box_scale(1.0F);
    TargetLocalizer localizer(options);
    const auto depth = DepthImage<float>(
        "32FC1", 4, 1,
        std::vector<float>{2.0F, 2.1F, std::numeric_limits<float>::quiet_NaN(),
                           9.0F});
    float range_m = 99.0F;
    std::string error;

    EXPECT_FALSE(localizer.EstimateRange(Box(2.0, 0.5, 4.0, 1.0), depth,
                                         Camera(4, 1, 1.0, 1.0, 2.0, 0.5),
                                         &range_m, &error));
    EXPECT_FLOAT_EQ(range_m, 0.0F);
    EXPECT_FALSE(error.empty());
}

TEST(TargetLocalizerTest, RejectsInvalidCameraIntrinsics) {
    TargetLocalizer localizer(ValidOptions());
    float range_m = 99.0F;
    std::string error;

    EXPECT_FALSE(localizer.EstimateRange(
        Box(), DepthAt(2.0F), Camera(6, 6, 0.0, 1.0), &range_m, &error));
    EXPECT_FLOAT_EQ(range_m, 0.0F);
    EXPECT_FALSE(error.empty());
}

TEST(TargetLocalizerTest, BackProjectsAndAppliesKnownRigidTransform) {
    TargetLocalizer localizer(ValidOptions());
    PoseStamped pose;
    TwistStamped velocity;
    constexpr double kHalfSqrtTwo = 0.7071067811865476;
    constexpr int64_t kStampNs = 2'500'000'000;

    ASSERT_TRUE(localizer.Locate("1", kStampNs, Box(4.0, 3.0), DepthAt(2.0F),
                                 Camera(),
                                 CameraToMap(kStampNs, 10.0, 20.0, 1.0, 0.0,
                                             0.0, kHalfSqrtTwo, kHalfSqrtTwo),
                                 &pose, &velocity));

    EXPECT_EQ(pose.header().frame_id(), "map");
    EXPECT_EQ(pose.header().stamp().sec(), 2);
    EXPECT_EQ(pose.header().stamp().nanosec(), 500'000'000U);
    EXPECT_NEAR(pose.pose().position().x(), 10.0, 1.0e-12);
    EXPECT_NEAR(pose.pose().position().y(), 22.0, 1.0e-12);
    EXPECT_NEAR(pose.pose().position().z(), 3.0, 1.0e-12);
    EXPECT_DOUBLE_EQ(pose.pose().orientation().x(), 0.0);
    EXPECT_DOUBLE_EQ(pose.pose().orientation().y(), 0.0);
    EXPECT_DOUBLE_EQ(pose.pose().orientation().z(), 0.0);
    EXPECT_DOUBLE_EQ(pose.pose().orientation().w(), 1.0);
    EXPECT_EQ(velocity.header().frame_id(), "map");
    EXPECT_EQ(velocity.header().stamp().sec(), 2);
    EXPECT_EQ(velocity.header().stamp().nanosec(), 500'000'000U);
    EXPECT_DOUBLE_EQ(velocity.twist().linear().x(), 0.0);
    EXPECT_DOUBLE_EQ(velocity.twist().linear().y(), 0.0);
}

TEST(TargetLocalizerTest, FiltersPositionAndDerivesTwoFramePlanarVelocity) {
    TargetLocalizer localizer(ValidOptions());
    PoseStamped pose;
    TwistStamped velocity;

    ASSERT_TRUE(localizer.Locate("1", 1'000'000'000, Box(), DepthAt(2.0F),
                                 Camera(), CameraToMap(1'000'000'000), &pose,
                                 &velocity));
    const auto moved_transform = CameraToMap(2'000'000'000, 2.0, 4.0);
    ASSERT_TRUE(localizer.Locate("1", 2'000'000'000, Box(), DepthAt(2.0F),
                                 Camera(), moved_transform, &pose, &velocity));

    EXPECT_DOUBLE_EQ(pose.pose().position().x(), 1.0);
    EXPECT_DOUBLE_EQ(pose.pose().position().y(), 2.0);
    EXPECT_DOUBLE_EQ(velocity.twist().linear().x(), 1.0);
    EXPECT_DOUBLE_EQ(velocity.twist().linear().y(), 2.0);
    EXPECT_DOUBLE_EQ(velocity.twist().linear().z(), 0.0);
}

TEST(TargetLocalizerTest, RejectsNonMonotonicTimeAndResetsHistory) {
    TargetLocalizer localizer(ValidOptions());
    PoseStamped pose;
    TwistStamped velocity;
    ASSERT_TRUE(localizer.Locate("1", 2'000'000'000, Box(), DepthAt(2.0F),
                                 Camera(), CameraToMap(2'000'000'000), &pose,
                                 &velocity));
    pose.mutable_header()->set_frame_id("stale");
    velocity.mutable_header()->set_frame_id("stale");
    std::string error;

    EXPECT_FALSE(localizer.Locate("1", 1'000'000'000, Box(), DepthAt(2.0F),
                                  Camera(), CameraToMap(1'000'000'000, 2.0),
                                  &pose, &velocity, &error));
    EXPECT_EQ(pose.ByteSizeLong(), 0U);
    EXPECT_EQ(velocity.ByteSizeLong(), 0U);
    EXPECT_FALSE(error.empty());

    ASSERT_TRUE(localizer.Locate("1", 3'000'000'000, Box(), DepthAt(2.0F),
                                 Camera(), CameraToMap(3'000'000'000, 4.0),
                                 &pose, &velocity));
    EXPECT_DOUBLE_EQ(pose.pose().position().x(), 4.0);
    EXPECT_DOUBLE_EQ(velocity.twist().linear().x(), 0.0);
}

TEST(TargetLocalizerTest, ResetsVelocityForNewTarget) {
    TargetLocalizer localizer(ValidOptions());
    PoseStamped pose;
    TwistStamped velocity;
    ASSERT_TRUE(localizer.Locate("1", 1'000'000'000, Box(), DepthAt(2.0F),
                                 Camera(), CameraToMap(1'000'000'000), &pose,
                                 &velocity));
    ASSERT_TRUE(localizer.Locate("2", 2'000'000'000, Box(), DepthAt(3.0F),
                                 Camera(), CameraToMap(2'000'000'000, 4.0),
                                 &pose, &velocity));

    EXPECT_DOUBLE_EQ(pose.pose().position().x(), 4.0);
    EXPECT_DOUBLE_EQ(velocity.twist().linear().x(), 0.0);
    EXPECT_DOUBLE_EQ(velocity.twist().linear().y(), 0.0);
}

TEST(TargetLocalizerTest, EstimateRangeDoesNotChangeVelocityHistory) {
    TargetLocalizer localizer(ValidOptions());
    PoseStamped pose;
    TwistStamped velocity;
    ASSERT_TRUE(localizer.Locate("1", 1'000'000'000, Box(), DepthAt(2.0F),
                                 Camera(), CameraToMap(1'000'000'000), &pose,
                                 &velocity));
    float near_range_m = 0.0F;
    float far_range_m = 0.0F;

    ASSERT_TRUE(localizer.EstimateRange(Box(), DepthAt(1.25F), Camera(),
                                        &near_range_m));
    const auto far_depth = DepthAt(4.5F);
    ASSERT_TRUE(
        localizer.EstimateRange(Box(), far_depth, Camera(), &far_range_m));
    EXPECT_FLOAT_EQ(near_range_m, 1.25F);
    EXPECT_FLOAT_EQ(far_range_m, 4.5F);

    ASSERT_TRUE(localizer.Locate("1", 2'000'000'000, Box(), DepthAt(2.0F),
                                 Camera(), CameraToMap(2'000'000'000, 2.0),
                                 &pose, &velocity));
    EXPECT_DOUBLE_EQ(pose.pose().position().x(), 1.0);
    EXPECT_DOUBLE_EQ(velocity.twist().linear().x(), 1.0);
}

TEST(TargetLocalizerTest, ClearRemovesVelocityHistory) {
    TargetLocalizer localizer(ValidOptions());
    PoseStamped pose;
    TwistStamped velocity;
    ASSERT_TRUE(localizer.Locate("1", 1'000'000'000, Box(), DepthAt(2.0F),
                                 Camera(), CameraToMap(1'000'000'000), &pose,
                                 &velocity));
    localizer.Clear();

    ASSERT_TRUE(localizer.Locate("1", 2'000'000'000, Box(), DepthAt(2.0F),
                                 Camera(), CameraToMap(2'000'000'000, 3.0),
                                 &pose, &velocity));
    EXPECT_DOUBLE_EQ(pose.pose().position().x(), 3.0);
    EXPECT_DOUBLE_EQ(velocity.twist().linear().x(), 0.0);
}

TEST(TargetLocalizerTest, ClearsAllOutputsBeforeFailure) {
    TargetLocalizer localizer(ValidOptions());
    auto depth = DepthAt(2.0F);
    depth.set_encoding("mono16");
    float range_m = 99.0F;
    std::string error = "stale error";

    EXPECT_FALSE(
        localizer.EstimateRange(Box(), depth, Camera(), &range_m, &error));
    EXPECT_FLOAT_EQ(range_m, 0.0F);
    EXPECT_FALSE(error.empty());
    EXPECT_NE(error, "stale error");

    PoseStamped pose;
    pose.mutable_header()->set_frame_id("stale");
    TwistStamped velocity;
    velocity.mutable_header()->set_frame_id("stale");
    const auto transform = CameraToMap(1'000'000'000);
    EXPECT_FALSE(localizer.Locate("1", 1'000'000'000, Box(), depth, Camera(),
                                  transform, &pose, &velocity, &error));
    EXPECT_EQ(pose.ByteSizeLong(), 0U);
    EXPECT_EQ(velocity.ByteSizeLong(), 0U);
    EXPECT_FALSE(error.empty());
}

}  // namespace
}  // namespace shadow
}  // namespace perception
}  // namespace autonomy
