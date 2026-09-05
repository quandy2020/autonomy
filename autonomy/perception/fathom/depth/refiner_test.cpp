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

/**
 * @file refiner_test.cpp
 * @brief Unit tests for backend-independent Fathom depth refinement.
 */

#include "autonomy/perception/fathom/depth/refiner.hpp"
#include "autonomy/perception/fathom/config.hpp"

#include <automsgs/msgs/sensor_msgs/point_cloud2_iterator.hpp>

#include "gtest/gtest.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <utility>

namespace autonomy {
namespace perception {
namespace fathom {
namespace {

FathomConfig MakeConfig() {
    FathomConfig config;
    config.model_path = "fathom.onnx";
    config.backend = "onnx";
    config.input_width = 2;
    config.input_height = 1;
    config.depth_scale = 0.001F;
    config.mask_threshold = 0.5F;
    return config;
}

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

automsgs::msgs::sensor_msgs::CameraInfo MakeCameraInfo(uint32_t width,
                                                       uint32_t height) {
    automsgs::msgs::sensor_msgs::CameraInfo info;
    info.set_width(width);
    info.set_height(height);
    const double matrix[] = {2.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 1.0};
    for (double value : matrix) {
        info.add_k(value);
    }
    return info;
}

class StaticRunner final : public FathomModelRunner
{
public:
    bool Run(const common::network::TensorMap& inputs,
             common::network::TensorMap* outputs, std::string* error) override {
        if (outputs == nullptr || inputs.count("image") == 0 ||
            inputs.count("raw_depth") == 0) {
            if (error != nullptr) {
                *error = "StaticRunner received invalid tensors.";
            }
            return false;
        }
        common::network::TensorMap result;
        result.emplace("refined_depth",
                       common::network::Tensor::FromFloat32({2.0F, 4.0F}));
        result.emplace("validity",
                       common::network::Tensor::FromFloat32({0.2F, 0.8F}));
        *outputs = std::move(result);
        return true;
    }
};

class FailingRunner final : public FathomModelRunner
{
public:
    bool Run(const common::network::TensorMap&, common::network::TensorMap*,
             std::string* error) override {
        if (error != nullptr) {
            *error = "inference failed";
        }
        return false;
    }
};

}  // namespace

TEST(FathomConfigTest, RejectsInvalidDeploymentValues) {
    std::string error;

    FathomConfig config = MakeConfig();
    config.model_path.clear();
    EXPECT_FALSE(ValidateFathomConfig(config, &error));
    EXPECT_FALSE(error.empty());

    config = MakeConfig();
    config.input_width = 0;
    EXPECT_FALSE(ValidateFathomConfig(config, &error));

    config = MakeConfig();
    config.input_height = -1;
    EXPECT_FALSE(ValidateFathomConfig(config, &error));

    config = MakeConfig();
    config.depth_scale = 0.0F;
    EXPECT_FALSE(ValidateFathomConfig(config, &error));

    config = MakeConfig();
    config.mask_threshold = -0.01F;
    EXPECT_FALSE(ValidateFathomConfig(config, &error));

    config = MakeConfig();
    config.mask_threshold = 1.01F;
    EXPECT_FALSE(ValidateFathomConfig(config, &error));
}

TEST(FathomConfigTest, AcceptsSupportedBackends) {
    std::string error = "stale error";
    FathomConfig config = MakeConfig();

    config.backend = "onnx";
    EXPECT_TRUE(ValidateFathomConfig(config, &error)) << error;
    EXPECT_TRUE(error.empty());
    error = "stale error";
    config.backend = "tensorrt";
    EXPECT_TRUE(ValidateFathomConfig(config, &error)) << error;
    EXPECT_TRUE(error.empty());
}

TEST(DepthRefinerTest, RestoresImagesThresholdsValidityAndProjectsCloud) {
    const FathomConfig config = MakeConfig();
    std::string error;
    auto refiner =
        DepthRefiner::Create(config, std::make_unique<StaticRunner>(), &error);
    ASSERT_NE(refiner, nullptr) << error;

    auto rgb = MakeImage("bgr8", 4, 2, 12);
    auto raw_depth = MakeImage("16UC1", 4, 2, 8);
    const uint16_t raw_values[] = {1000, 1000, 1000, 1000,
                                   1000, 1000, 1000, 1000};
    std::memcpy(raw_depth.mutable_data()->data(), raw_values,
                sizeof(raw_values));
    raw_depth.mutable_header()->set_frame_id("camera_optical");
    automsgs::msgs::sensor_msgs::Image refined_depth;
    automsgs::msgs::sensor_msgs::PointCloud2 cloud;
    error = "stale error";

    ASSERT_TRUE(refiner->Refine(rgb, raw_depth, MakeCameraInfo(4, 2),
                                &refined_depth, &cloud, &error))
        << error;
    EXPECT_TRUE(error.empty());

    EXPECT_EQ(refined_depth.encoding(), "32FC1");
    EXPECT_EQ(refined_depth.width(), 4U);
    EXPECT_EQ(refined_depth.height(), 2U);
    EXPECT_EQ(refined_depth.step(), 4U * sizeof(float));
    EXPECT_EQ(refined_depth.header().frame_id(), "camera_optical");
    EXPECT_EQ(cloud.width(), 4U);
    EXPECT_EQ(cloud.height(), 2U);
    EXPECT_EQ(cloud.fields_size(), 3);
    EXPECT_EQ(cloud.header().frame_id(), "camera_optical");

    automsgs::msgs::sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
    automsgs::msgs::sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
    EXPECT_TRUE(std::isnan(*x));
    EXPECT_TRUE(std::isnan(*z));
    x += 3;
    z += 3;
    EXPECT_FLOAT_EQ(*z, 4.0F);
    EXPECT_FLOAT_EQ(*x, 6.0F);
}

TEST(DepthRefinerTest, ClearsOutputsWhenInferenceFails) {
    const FathomConfig config = MakeConfig();
    std::string error;
    auto refiner =
        DepthRefiner::Create(config, std::make_unique<FailingRunner>(), &error);
    ASSERT_NE(refiner, nullptr) << error;

    auto rgb = MakeImage("bgr8", 2, 1, 6);
    auto raw_depth = MakeImage("16UC1", 2, 1, 4);
    automsgs::msgs::sensor_msgs::Image refined_depth;
    refined_depth.set_width(9);
    automsgs::msgs::sensor_msgs::PointCloud2 cloud;
    cloud.set_width(9);

    EXPECT_FALSE(refiner->Refine(rgb, raw_depth, MakeCameraInfo(2, 1),
                                 &refined_depth, &cloud, &error));
    EXPECT_FALSE(error.empty());
    EXPECT_EQ(refined_depth.width(), 0U);
    EXPECT_EQ(cloud.width(), 0U);
}

TEST(DepthRefinerTest, ClearsRefinedDepthWhenPointCloudOutputIsNull) {
    const FathomConfig config = MakeConfig();
    std::string error;
    auto refiner =
        DepthRefiner::Create(config, std::make_unique<StaticRunner>(), &error);
    ASSERT_NE(refiner, nullptr) << error;

    automsgs::msgs::sensor_msgs::Image refined_depth;
    refined_depth.set_width(9);

    EXPECT_FALSE(refiner->Refine(automsgs::msgs::sensor_msgs::Image(),
                                 automsgs::msgs::sensor_msgs::Image(),
                                 MakeCameraInfo(1, 1), &refined_depth, nullptr,
                                 &error));
    EXPECT_FALSE(error.empty());
    EXPECT_EQ(refined_depth.width(), 0U);
}

TEST(DepthRefinerTest, ClearsPointCloudWhenRefinedDepthOutputIsNull) {
    const FathomConfig config = MakeConfig();
    std::string error;
    auto refiner =
        DepthRefiner::Create(config, std::make_unique<StaticRunner>(), &error);
    ASSERT_NE(refiner, nullptr) << error;

    automsgs::msgs::sensor_msgs::PointCloud2 cloud;
    cloud.set_width(9);

    EXPECT_FALSE(refiner->Refine(automsgs::msgs::sensor_msgs::Image(),
                                 automsgs::msgs::sensor_msgs::Image(),
                                 MakeCameraInfo(1, 1), nullptr, &cloud,
                                 &error));
    EXPECT_FALSE(error.empty());
    EXPECT_EQ(cloud.width(), 0U);
}

}  // namespace fathom
}  // namespace perception
}  // namespace autonomy
