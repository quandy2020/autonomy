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

#include "autonomy/perception/fathom/component_config.hpp"

#include <gtest/gtest.h>

#include <string>

namespace autonomy::perception::fathom {
namespace {

proto::FathomComponentConfig ValidComponentConfig() {
    proto::FathomComponentConfig config;
    config.set_model_path("/models/fathom.onnx");
    config.set_backend("onnx");
    config.set_input_width(640);
    config.set_input_height(480);
    config.set_depth_scale(0.001F);
    config.set_mask_threshold(0.5F);
    config.set_refined_depth_topic("/perception/fathom/refined_depth");
    config.set_point_cloud_topic("/perception/fathom/points");
    return config;
}

TEST(FathomComponentConfigTest, TranslatesValidatedDeploymentProfile) {
    const auto config = ValidComponentConfig();
    FathomConfig fathom_config;
    FathomComponentTopics topics;
    std::string error;

    ASSERT_TRUE(TranslateFathomComponentConfig(config, &fathom_config, &topics,
                                                &error));

    EXPECT_EQ(fathom_config.model_path, "/models/fathom.onnx");
    EXPECT_EQ(fathom_config.backend, "onnx");
    EXPECT_EQ(fathom_config.input_width, 640);
    EXPECT_EQ(fathom_config.input_height, 480);
    EXPECT_FLOAT_EQ(fathom_config.depth_scale, 0.001F);
    EXPECT_FLOAT_EQ(fathom_config.mask_threshold, 0.5F);
    EXPECT_EQ(topics.refined_depth, "/perception/fathom/refined_depth");
    EXPECT_EQ(topics.point_cloud, "/perception/fathom/points");
    EXPECT_TRUE(error.empty());
}

TEST(FathomComponentConfigTest, RejectsEmptyOutputTopics) {
    auto config = ValidComponentConfig();
    config.clear_refined_depth_topic();
    FathomConfig fathom_config;
    FathomComponentTopics topics;
    std::string error;

    EXPECT_FALSE(TranslateFathomComponentConfig(config, &fathom_config, &topics,
                                                 &error));
    EXPECT_EQ(error, "Fathom component: refined_depth_topic must not be empty.");

    config = ValidComponentConfig();
    config.clear_point_cloud_topic();
    EXPECT_FALSE(TranslateFathomComponentConfig(config, &fathom_config, &topics,
                                                 &error));
    EXPECT_EQ(error, "Fathom component: point_cloud_topic must not be empty.");
}

TEST(FathomComponentConfigTest, RejectsInvalidInferenceProfile) {
    auto config = ValidComponentConfig();
    config.set_input_width(0);
    FathomConfig fathom_config;
    FathomComponentTopics topics;
    std::string error;

    EXPECT_FALSE(TranslateFathomComponentConfig(config, &fathom_config, &topics,
                                                 &error));
    EXPECT_EQ(error,
              "Fathom: input_width and input_height must be positive.");
}

}  // namespace
}  // namespace autonomy::perception::fathom
