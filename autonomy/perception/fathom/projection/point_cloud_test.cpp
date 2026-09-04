/*
 * Copyright 2025 The OpenRobotic Beginner Authors (duyongquan)
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

#include "autonomy/perception/fathom/projection/point_cloud.hpp"

#include <automsgs/msgs/sensor_msgs/point_cloud2_iterator.hpp>

#include "gtest/gtest.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

namespace autonomy {
namespace perception {
namespace fathom {

namespace {

automsgs::msgs::sensor_msgs::Image MakeImage(const std::string& encoding,
                                              uint32_t width, uint32_t height,
                                              uint32_t step) {
    automsgs::msgs::sensor_msgs::Image image;
    image.set_encoding(encoding);
    image.set_width(width);
    image.set_height(height);
    image.set_step(step);
    image.set_is_bigendian(false);
    image.mutable_data()->resize(static_cast<size_t>(height) * step);
    return image;
}

automsgs::msgs::sensor_msgs::CameraInfo MakeCameraInfo(float fx, float fy) {
    automsgs::msgs::sensor_msgs::CameraInfo info;
    const double matrix[] = {fx, 0.0, 0.0, 0.0, fy, 0.0, 0.0, 0.0, 1.0};
    for (double value : matrix) {
        info.add_k(value);
    }
    return info;
}

}  // namespace

TEST(ProjectDepthTest, ProjectsValidMetricDepthInCameraCoordinates) {
    auto depth_m = MakeImage("32FC1", 2, 1, 8);
    const float depths[] = {2.0F, 2.0F};
    std::memcpy(depth_m.mutable_data()->data(), depths, sizeof(depths));
    depth_m.mutable_header()->set_frame_id("camera_optical");
    auto mask = MakeImage("mono8", 2, 1, 2);
    mask.mutable_data()->at(0) = 1;
    mask.mutable_data()->at(1) = 1;
    automsgs::msgs::sensor_msgs::PointCloud2 cloud;
    std::string error;

    ASSERT_TRUE(ProjectDepth(depth_m, mask, MakeCameraInfo(2.0F, 2.0F),
                             &cloud, &error))
        << error;

    EXPECT_EQ(cloud.height(), 1U);
    EXPECT_EQ(cloud.width(), 2U);
    EXPECT_EQ(cloud.fields_size(), 3);
    EXPECT_EQ(cloud.header().frame_id(), "camera_optical");
    automsgs::msgs::sensor_msgs::PointCloud2ConstIterator<float> x(cloud,
                                                                      "x");
    automsgs::msgs::sensor_msgs::PointCloud2ConstIterator<float> y(cloud,
                                                                      "y");
    automsgs::msgs::sensor_msgs::PointCloud2ConstIterator<float> z(cloud,
                                                                      "z");
    EXPECT_FLOAT_EQ(*x, 0.0F);
    EXPECT_FLOAT_EQ(*y, 0.0F);
    EXPECT_FLOAT_EQ(*z, 2.0F);
    ++x;
    ++y;
    ++z;
    EXPECT_FLOAT_EQ(*x, 1.0F);
    EXPECT_FLOAT_EQ(*y, 0.0F);
    EXPECT_FLOAT_EQ(*z, 2.0F);
}

TEST(ProjectDepthTest, MarksInvalidMaskPositionsAsNan) {
    auto depth_m = MakeImage("32FC1", 2, 1, 8);
    const float depths[] = {2.0F, 2.0F};
    std::memcpy(depth_m.mutable_data()->data(), depths, sizeof(depths));
    auto mask = MakeImage("mono8", 2, 1, 2);
    mask.mutable_data()->at(0) = 1;
    mask.mutable_data()->at(1) = 0;
    automsgs::msgs::sensor_msgs::PointCloud2 cloud;

    ASSERT_TRUE(ProjectDepth(depth_m, mask, MakeCameraInfo(2.0F, 2.0F),
                             &cloud));

    automsgs::msgs::sensor_msgs::PointCloud2ConstIterator<float> x(cloud,
                                                                      "x");
    automsgs::msgs::sensor_msgs::PointCloud2ConstIterator<float> y(cloud,
                                                                      "y");
    automsgs::msgs::sensor_msgs::PointCloud2ConstIterator<float> z(cloud,
                                                                      "z");
    ++x;
    ++y;
    ++z;
    EXPECT_TRUE(std::isnan(*x));
    EXPECT_TRUE(std::isnan(*y));
    EXPECT_TRUE(std::isnan(*z));
}

TEST(ProjectDepthTest, RejectsNonPositiveFocalLengths) {
    auto depth_m = MakeImage("32FC1", 1, 1, 4);
    const float depth = 1.0F;
    std::memcpy(depth_m.mutable_data()->data(), &depth, sizeof(depth));
    auto mask = MakeImage("mono8", 1, 1, 1);
    mask.mutable_data()->at(0) = 1;
    automsgs::msgs::sensor_msgs::PointCloud2 cloud;
    std::string error;

    EXPECT_FALSE(ProjectDepth(depth_m, mask, MakeCameraInfo(0.0F, 1.0F),
                              &cloud, &error));
    EXPECT_FALSE(error.empty());
}

TEST(ProjectDepthTest, RejectsMaskMessageWithInsufficientData) {
    auto depth_m = MakeImage("32FC1", 1, 1, 4);
    const float depth = 1.0F;
    std::memcpy(depth_m.mutable_data()->data(), &depth, sizeof(depth));
    auto mask = MakeImage("mono8", 1, 1, 1);
    mask.mutable_data()->clear();
    automsgs::msgs::sensor_msgs::PointCloud2 cloud;
    std::string error;

    EXPECT_FALSE(ProjectDepth(depth_m, mask, MakeCameraInfo(1.0F, 1.0F),
                              &cloud, &error));
    EXPECT_FALSE(error.empty());
}

}  // namespace fathom
}  // namespace perception
}  // namespace autonomy
