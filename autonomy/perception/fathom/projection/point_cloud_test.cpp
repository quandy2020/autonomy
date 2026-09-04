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

#include "gtest/gtest.h"

#include <cmath>
#include <cstdint>
#include <string>

namespace autonomy {
namespace perception {
namespace fathom {

TEST(ProjectDepthTest, ProjectsValidMetricDepthInCameraCoordinates) {
    cv::Mat depth_m(1, 2, CV_32FC1, cv::Scalar(2.0F));
    cv::Mat mask(1, 2, CV_8UC1, cv::Scalar(255));
    cv::Mat xyz;
    std::string error;

    ASSERT_TRUE(ProjectDepth(depth_m, mask, {2.0F, 2.0F, 0.0F, 0.0F}, &xyz,
                             &error))
        << error;

    ASSERT_EQ(xyz.type(), CV_32FC3);
    const cv::Vec3f first = xyz.at<cv::Vec3f>(0, 0);
    const cv::Vec3f second = xyz.at<cv::Vec3f>(0, 1);
    EXPECT_FLOAT_EQ(first[0], 0.0F);
    EXPECT_FLOAT_EQ(first[1], 0.0F);
    EXPECT_FLOAT_EQ(first[2], 2.0F);
    EXPECT_FLOAT_EQ(second[0], 1.0F);
    EXPECT_FLOAT_EQ(second[1], 0.0F);
    EXPECT_FLOAT_EQ(second[2], 2.0F);
}

TEST(ProjectDepthTest, MarksInvalidMaskPositionsAsNan) {
    cv::Mat depth_m(1, 2, CV_32FC1, cv::Scalar(2.0F));
    cv::Mat mask(1, 2, CV_8UC1);
    mask.at<uint8_t>(0, 0) = 255;
    mask.at<uint8_t>(0, 1) = 0;
    cv::Mat xyz;

    ASSERT_TRUE(ProjectDepth(depth_m, mask, {2.0F, 2.0F, 0.0F, 0.0F}, &xyz));

    const cv::Vec3f invalid = xyz.at<cv::Vec3f>(0, 1);
    EXPECT_TRUE(std::isnan(invalid[0]));
    EXPECT_TRUE(std::isnan(invalid[1]));
    EXPECT_TRUE(std::isnan(invalid[2]));
}

TEST(ProjectDepthTest, RejectsNonPositiveFocalLengths) {
    cv::Mat depth_m(1, 1, CV_32FC1, cv::Scalar(1.0F));
    cv::Mat mask(1, 1, CV_8UC1, cv::Scalar(255));
    cv::Mat xyz;
    std::string error;

    EXPECT_FALSE(ProjectDepth(depth_m, mask, {0.0F, 1.0F, 0.0F, 0.0F},
                              &xyz, &error));
    EXPECT_FALSE(error.empty());
}

}  // namespace fathom
}  // namespace perception
}  // namespace autonomy
