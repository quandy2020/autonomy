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
 * @file fathom_component_test.cpp
 * @brief Unit tests for Fathom component lifecycle and publication behavior.
 */

#include "autonomy/perception/fathom/fathom_component.hpp"
#include "autonomy/perception/fathom/options.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <utility>

namespace autonomy::perception::fathom {
class FathomComponentTestApi
{
public:
    static void SetCallbacks(
        FathomComponent* component, FathomComponent::RefineFunction refine,
        FathomComponent::ImagePublisher depth_publisher,
        FathomComponent::PointCloudPublisher cloud_publisher) {
        component->refine_ = std::move(refine);
        component->refined_depth_publish_ = std::move(depth_publisher);
        component->point_cloud_publish_ = std::move(cloud_publisher);
    }

    static bool Process(
        FathomComponent* component,
        const std::shared_ptr<FathomComponent::Image>& rgb,
        const std::shared_ptr<FathomComponent::Image>& depth,
        const std::shared_ptr<FathomComponent::CameraInfo>& info) {
        return component->Proc(rgb, depth, info);
    }

    static void Clear(FathomComponent* component) {
        component->Clear();
    }
};

namespace {

proto::FathomOptions ValidOptions() {
    proto::FathomOptions options;
    options.set_model_path("/models/fathom.onnx");
    options.set_backend("onnx");
    options.set_input_width(640);
    options.set_input_height(480);
    options.set_depth_scale(0.001F);
    options.set_mask_threshold(0.5F);
    options.set_refined_depth_topic("/perception/fathom/refined_depth");
    options.set_point_cloud_topic("/perception/fathom/points");
    return options;
}

std::shared_ptr<FathomComponent::Image> MakeImage() {
    return std::make_shared<FathomComponent::Image>();
}

std::shared_ptr<FathomComponent::CameraInfo> MakeCameraInfo() {
    return std::make_shared<FathomComponent::CameraInfo>();
}

TEST(FathomOptionsTest, AcceptsCompleteComponentOptions) {
    const auto options = ValidOptions();
    std::string error = "stale error";

    EXPECT_TRUE(ValidateFathomOptions(options, &error)) << error;
    EXPECT_TRUE(error.empty());
}

TEST(FathomOptionsTest, RejectsEmptyOutputTopics) {
    auto options = ValidOptions();
    options.clear_refined_depth_topic();
    std::string error;

    EXPECT_FALSE(ValidateFathomOptions(options, &error));
    EXPECT_EQ(error, "Fathom: refined_depth_topic must not be empty.");

    options = ValidOptions();
    options.clear_point_cloud_topic();
    EXPECT_FALSE(ValidateFathomOptions(options, &error));
    EXPECT_EQ(error, "Fathom: point_cloud_topic must not be empty.");
}

TEST(FathomOptionsTest, RejectsInvalidInferenceProfile) {
    auto options = ValidOptions();
    options.set_input_width(0);
    std::string error;

    EXPECT_FALSE(ValidateFathomOptions(options, &error));
    EXPECT_EQ(error, "Fathom: input_width and input_height must be positive.");
}

TEST(FathomOptionsTest, RejectsEqualOutputTopics) {
    auto options = ValidOptions();
    options.set_point_cloud_topic(options.refined_depth_topic());
    std::string error;

    EXPECT_FALSE(ValidateFathomOptions(options, &error));
    EXPECT_EQ(error,
              "Fathom: refined_depth_topic and point_cloud_topic "
              "must differ.");
}

TEST(FathomComponentTest, RejectsNullInputsWithoutRefining) {
    FathomComponent component;
    int refine_calls = 0;
    FathomComponentTestApi::SetCallbacks(
        &component,
        [&refine_calls](
            const FathomComponent::Image&, const FathomComponent::Image&,
            const FathomComponent::CameraInfo&, FathomComponent::Image*,
            FathomComponent::PointCloud2*, std::string*) {
            ++refine_calls;
            return true;
        },
        [](const FathomComponent::Image&) { return true; },
        [](const FathomComponent::PointCloud2&) { return true; });

    EXPECT_FALSE(FathomComponentTestApi::Process(
        &component, nullptr, MakeImage(), MakeCameraInfo()));
    EXPECT_EQ(refine_calls, 0);
}

TEST(FathomComponentTest, RejectsUninitializedState) {
    FathomComponent component;

    EXPECT_FALSE(FathomComponentTestApi::Process(
        &component, MakeImage(), MakeImage(), MakeCameraInfo()));
}

TEST(FathomComponentTest, RefinesOnceAndReportsDepthPublishFailure) {
    FathomComponent component;
    int refine_calls = 0;
    int depth_publish_calls = 0;
    int cloud_publish_calls = 0;
    FathomComponentTestApi::SetCallbacks(
        &component,
        [&refine_calls](
            const FathomComponent::Image&, const FathomComponent::Image&,
            const FathomComponent::CameraInfo&, FathomComponent::Image*,
            FathomComponent::PointCloud2*, std::string*) {
            ++refine_calls;
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
    EXPECT_EQ(refine_calls, 1);
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
