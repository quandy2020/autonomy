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

#include "autonomy/perception/fathom/component.hpp"
#include "autonomy/perception/fathom/component_config.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <utility>

namespace autonomy::perception::fathom {
class FathomComponentTestApi {
public:
    static void SetCallbacks(FathomComponent* component,
                             FathomComponent::RunnerFunction runner,
                             FathomComponent::ImagePublisher depth_publisher,
                             FathomComponent::PointCloudPublisher cloud_publisher) {
        component->runner_process_ = std::move(runner);
        component->refined_depth_publish_ = std::move(depth_publisher);
        component->point_cloud_publish_ = std::move(cloud_publisher);
    }

    static bool Process(FathomComponent* component,
                        const std::shared_ptr<FathomComponent::Image>& rgb,
                        const std::shared_ptr<FathomComponent::Image>& depth,
                        const std::shared_ptr<FathomComponent::CameraInfo>& info) {
        return component->Proc(rgb, depth, info);
    }

    static void Clear(FathomComponent* component) { component->Clear(); }
};

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

std::shared_ptr<FathomComponent::Image> MakeImage() {
    return std::make_shared<FathomComponent::Image>();
}

std::shared_ptr<FathomComponent::CameraInfo> MakeCameraInfo() {
    return std::make_shared<FathomComponent::CameraInfo>();
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

TEST(FathomComponentConfigTest, RejectsEqualOutputTopics) {
    auto config = ValidComponentConfig();
    config.set_point_cloud_topic(config.refined_depth_topic());
    FathomConfig fathom_config;
    FathomComponentTopics topics;
    std::string error;

    EXPECT_FALSE(TranslateFathomComponentConfig(config, &fathom_config, &topics,
                                                 &error));
    EXPECT_EQ(error,
              "Fathom component: refined_depth_topic and point_cloud_topic "
              "must differ.");
}

TEST(FathomComponentTest, RejectsNullInputsWithoutRunningTheRunner) {
    FathomComponent component;
    int runner_calls = 0;
    FathomComponentTestApi::SetCallbacks(
        &component,
        [&runner_calls](const FathomComponent::Image&,
                        const FathomComponent::Image&,
                        const FathomComponent::CameraInfo&,
                        FathomComponent::Image*, FathomComponent::PointCloud2*,
                        std::string*) {
            ++runner_calls;
            return true;
        },
        [](const FathomComponent::Image&) { return true; },
        [](const FathomComponent::PointCloud2&) { return true; });

    EXPECT_FALSE(FathomComponentTestApi::Process(
        &component, nullptr, MakeImage(), MakeCameraInfo()));
    EXPECT_EQ(runner_calls, 0);
}

TEST(FathomComponentTest, RejectsUninitializedState) {
    FathomComponent component;

    EXPECT_FALSE(FathomComponentTestApi::Process(
        &component, MakeImage(), MakeImage(), MakeCameraInfo()));
}

TEST(FathomComponentTest, InvokesRunnerOnceAndReportsDepthPublishFailure) {
    FathomComponent component;
    int runner_calls = 0;
    int depth_publish_calls = 0;
    int cloud_publish_calls = 0;
    FathomComponentTestApi::SetCallbacks(
        &component,
        [&runner_calls](const FathomComponent::Image&,
                        const FathomComponent::Image&,
                        const FathomComponent::CameraInfo&,
                        FathomComponent::Image*, FathomComponent::PointCloud2*,
                        std::string*) {
            ++runner_calls;
            return true;
        },
        [&depth_publish_calls](const FathomComponent::Image&) {
            ++depth_publish_calls;
            return false;
        },
        [&cloud_publish_calls](const FathomComponent::PointCloud2&) {
            ++cloud_publish_calls;
            return true;
        });

    EXPECT_FALSE(FathomComponentTestApi::Process(
        &component, MakeImage(), MakeImage(), MakeCameraInfo()));
    EXPECT_EQ(runner_calls, 1);
    EXPECT_EQ(depth_publish_calls, 1);
    EXPECT_EQ(cloud_publish_calls, 1);
}

TEST(FathomComponentTest, ReportsPointCloudPublishFailure) {
    FathomComponent component;
    int depth_publish_calls = 0;
    int cloud_publish_calls = 0;
    FathomComponentTestApi::SetCallbacks(
        &component,
        [](const FathomComponent::Image&, const FathomComponent::Image&,
           const FathomComponent::CameraInfo&, FathomComponent::Image*,
           FathomComponent::PointCloud2*, std::string*) { return true; },
        [&depth_publish_calls](const FathomComponent::Image&) {
            ++depth_publish_calls;
            return true;
        },
        [&cloud_publish_calls](const FathomComponent::PointCloud2&) {
            ++cloud_publish_calls;
            return false;
        });

    EXPECT_FALSE(FathomComponentTestApi::Process(
        &component, MakeImage(), MakeImage(), MakeCameraInfo()));
    EXPECT_EQ(depth_publish_calls, 1);
    EXPECT_EQ(cloud_publish_calls, 1);
}

TEST(FathomComponentTest, ClearIsIdempotentAndRemovesCallbacks) {
    FathomComponent component;
    FathomComponentTestApi::SetCallbacks(
        &component,
        [](const FathomComponent::Image&, const FathomComponent::Image&,
           const FathomComponent::CameraInfo&, FathomComponent::Image*,
           FathomComponent::PointCloud2*, std::string*) { return true; },
        [](const FathomComponent::Image&) { return true; },
        [](const FathomComponent::PointCloud2&) { return true; });

    FathomComponentTestApi::Clear(&component);
    FathomComponentTestApi::Clear(&component);

    EXPECT_FALSE(FathomComponentTestApi::Process(
        &component, MakeImage(), MakeImage(), MakeCameraInfo()));
}

}  // namespace
}  // namespace autonomy::perception::fathom
