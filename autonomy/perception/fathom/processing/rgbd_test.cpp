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

#include "autonomy/perception/fathom/processing/rgbd.hpp"

#include "gtest/gtest.h"

#include <cstdint>
#include <string>

namespace autonomy {
namespace perception {
namespace fathom {

TEST(PrepareRgbdTest, ConvertsBgrAndMillimeterDepthToModelTensors) {
    cv::Mat bgr(1, 2, CV_8UC3);
    bgr.at<cv::Vec3b>(0, 0) = cv::Vec3b(0, 128, 255);
    bgr.at<cv::Vec3b>(0, 1) = cv::Vec3b(255, 64, 0);
    cv::Mat raw_depth(1, 2, CV_16UC1);
    raw_depth.at<uint16_t>(0, 0) = 1000;
    raw_depth.at<uint16_t>(0, 1) = 0;

    common::network::TensorMap tensors;
    std::string error;
    ASSERT_TRUE(PrepareRgbd({bgr, raw_depth, {2.0F, 2.0F, 0.0F, 0.0F}},
                            2, 1, 0.001F, &tensors, &error))
        << error;

    ASSERT_EQ(tensors.size(), 2U);
    EXPECT_EQ(tensors.at("image").element_type(),
              common::network::ElementType::kFloat32);
    EXPECT_EQ(tensors.at("image").element_count(), 6U);
    EXPECT_EQ(tensors.at("raw_depth").element_type(),
              common::network::ElementType::kFloat32);
    EXPECT_EQ(tensors.at("raw_depth").element_count(), 2U);
    const float* image = tensors.at("image").data_as<float>();
    const float* depth = tensors.at("raw_depth").data_as<float>();
    ASSERT_NE(image, nullptr);
    ASSERT_NE(depth, nullptr);
    EXPECT_FLOAT_EQ(image[0], 1.0F);
    EXPECT_FLOAT_EQ(image[1], 0.0F);
    EXPECT_FLOAT_EQ(image[2], 128.0F / 255.0F);
    EXPECT_FLOAT_EQ(image[3], 64.0F / 255.0F);
    EXPECT_FLOAT_EQ(image[4], 0.0F);
    EXPECT_FLOAT_EQ(image[5], 1.0F);
    EXPECT_FLOAT_EQ(depth[0], 1.0F);
    EXPECT_FLOAT_EQ(depth[1], 0.0F);
}

TEST(PrepareRgbdTest, RejectsMismatchedRgbAndDepthDimensions) {
    cv::Mat bgr(1, 2, CV_8UC3, cv::Scalar());
    cv::Mat raw_depth(1, 1, CV_16UC1, cv::Scalar());
    common::network::TensorMap tensors;
    std::string error;

    EXPECT_FALSE(PrepareRgbd({bgr, raw_depth, {}}, 2, 1, 0.001F, &tensors,
                             &error));
    EXPECT_FALSE(error.empty());
}

}  // namespace fathom
}  // namespace perception
}  // namespace autonomy
