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

}  // namespace

TEST(PrepareRgbdTest, ConvertsBgrAndMillimeterDepthToModelTensors) {
    auto rgb = MakeImage("bgr8", 2, 1, 6);
    rgb.mutable_data()->at(0) = 0;
    rgb.mutable_data()->at(1) = static_cast<char>(128);
    rgb.mutable_data()->at(2) = static_cast<char>(255);
    rgb.mutable_data()->at(3) = static_cast<char>(255);
    rgb.mutable_data()->at(4) = 64;
    rgb.mutable_data()->at(5) = 0;
    auto raw_depth = MakeImage("16UC1", 2, 1, 4);
    const uint16_t raw_values[] = {1000, 0};
    std::memcpy(raw_depth.mutable_data()->data(), raw_values,
                sizeof(raw_values));

    common::network::TensorMap tensors;
    std::string error;
    ASSERT_TRUE(PrepareRgbd(rgb, raw_depth, 2, 1, 0.001F, &tensors, &error))
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
    const auto rgb = MakeImage("bgr8", 2, 1, 6);
    const auto raw_depth = MakeImage("16UC1", 1, 1, 2);
    common::network::TensorMap tensors;
    std::string error;

    EXPECT_FALSE(PrepareRgbd(rgb, raw_depth, 2, 1, 0.001F, &tensors,
                             &error));
    EXPECT_FALSE(error.empty());
}

TEST(PrepareRgbdTest, RejectsDepthMessageWithAnInvalidStep) {
    const auto rgb = MakeImage("bgr8", 1, 1, 3);
    auto raw_depth = MakeImage("16UC1", 1, 1, 2);
    raw_depth.set_step(1);
    common::network::TensorMap tensors;
    std::string error;

    EXPECT_FALSE(PrepareRgbd(rgb, raw_depth, 1, 1, 0.001F, &tensors,
                             &error));
    EXPECT_FALSE(error.empty());
}

TEST(PrepareRgbdTest, RejectsRgbMessageWithInsufficientData) {
    auto rgb = MakeImage("bgr8", 1, 1, 3);
    rgb.mutable_data()->clear();
    const auto raw_depth = MakeImage("16UC1", 1, 1, 2);
    common::network::TensorMap tensors;
    std::string error;

    EXPECT_FALSE(PrepareRgbd(rgb, raw_depth, 1, 1, 0.001F, &tensors,
                             &error));
    EXPECT_FALSE(error.empty());
}

TEST(PrepareRgbdTest, RejectsUnsupportedRgbEncoding) {
    const auto rgb = MakeImage("rgb8", 1, 1, 3);
    const auto raw_depth = MakeImage("16UC1", 1, 1, 2);
    common::network::TensorMap tensors;
    std::string error;

    EXPECT_FALSE(PrepareRgbd(rgb, raw_depth, 1, 1, 0.001F, &tensors,
                             &error));
    EXPECT_FALSE(error.empty());
}

TEST(PrepareRgbdTest, RejectsBigEndianDepthMessage) {
    const auto rgb = MakeImage("bgr8", 1, 1, 3);
    auto raw_depth = MakeImage("16UC1", 1, 1, 2);
    raw_depth.set_is_bigendian(true);
    common::network::TensorMap tensors;
    std::string error;

    EXPECT_FALSE(PrepareRgbd(rgb, raw_depth, 1, 1, 0.001F, &tensors,
                             &error));
    EXPECT_FALSE(error.empty());
}

TEST(PrepareRgbdTest, ClearsStaleOutputBeforeValidationFailure) {
    const auto rgb = MakeImage("rgb8", 1, 1, 3);
    const auto raw_depth = MakeImage("16UC1", 1, 1, 2);
    common::network::TensorMap tensors;
    tensors.emplace("stale",
                    common::network::Tensor::FromFloat32({123.0F}));
    std::string error = "stale error";

    EXPECT_FALSE(PrepareRgbd(rgb, raw_depth, 1, 1, 0.001F, &tensors,
                             &error));
    EXPECT_TRUE(tensors.empty());
    EXPECT_FALSE(error.empty());
    EXPECT_NE(error, "stale error");
}

TEST(PrepareRgbdTest, ClearsStaleErrorAndReplacesOutputOnSuccess) {
    const auto rgb = MakeImage("bgr8", 1, 1, 3);
    const auto raw_depth = MakeImage("16UC1", 1, 1, 2);
    common::network::TensorMap tensors;
    tensors.emplace("stale",
                    common::network::Tensor::FromFloat32({123.0F}));
    std::string error = "stale error";

    ASSERT_TRUE(PrepareRgbd(rgb, raw_depth, 1, 1, 0.001F, &tensors,
                            &error))
        << error;
    EXPECT_TRUE(error.empty());
    EXPECT_EQ(tensors.size(), 2U);
    EXPECT_EQ(tensors.count("stale"), 0U);
}

}  // namespace fathom
}  // namespace perception
}  // namespace autonomy
