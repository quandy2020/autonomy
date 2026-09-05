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
 * @file hestia_component_test.cpp
 * @brief Unit tests for Hestia component lifecycle and publication seams.
 */

#include "autonomy/perception/hestia/hestia_component.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <utility>

namespace autonomy::perception::hestia {

class HestiaComponentTestApi
{
public:
    static void SetCallbacks(HestiaComponent* component,
                             HestiaComponent::FrameFunction frame,
                             HestiaComponent::Detection2dPublisher publish_2d,
                             HestiaComponent::Detection3dPublisher publish_3d) {
        component->frame_ = std::move(frame);
        component->publish_2d_ = std::move(publish_2d);
        component->publish_3d_ = std::move(publish_3d);
    }

    static bool Process(HestiaComponent* component,
                        const std::shared_ptr<HestiaComponent::Image>& rgb,
                        const std::shared_ptr<HestiaComponent::Image>& depth,
                        const std::shared_ptr<HestiaComponent::CameraInfo>& info) {
        return component->Proc(rgb, depth, info);
    }

    static void Clear(HestiaComponent* component) { component->Clear(); }
};

namespace {

std::shared_ptr<HestiaComponent::Image> MakeImage() {
    return std::make_shared<HestiaComponent::Image>();
}

std::shared_ptr<HestiaComponent::CameraInfo> MakeCameraInfo() {
    return std::make_shared<HestiaComponent::CameraInfo>();
}

TEST(HestiaComponentTest, RejectsNullInputs) {
    HestiaComponent component;
    int frame_calls = 0;
    HestiaComponentTestApi::SetCallbacks(
        &component,
        [&](const HestiaComponent::Image&, const HestiaComponent::Image&,
            const HestiaComponent::CameraInfo&,
            HestiaComponent::Detection2DArray*,
            HestiaComponent::Detection3DArray*, std::string*) {
            ++frame_calls;
            return true;
        },
        [](const HestiaComponent::Detection2DArray&) { return true; },
        [](const HestiaComponent::Detection3DArray&) { return true; });

    EXPECT_FALSE(HestiaComponentTestApi::Process(
        &component, nullptr, MakeImage(), MakeCameraInfo()));
    EXPECT_EQ(frame_calls, 0);
}

TEST(HestiaComponentTest, RejectsUninitializedState) {
    HestiaComponent component;
    EXPECT_FALSE(HestiaComponentTestApi::Process(
        &component, MakeImage(), MakeImage(), MakeCameraInfo()));
}

TEST(HestiaComponentTest, PublishesBothTopicsOnSuccess) {
    HestiaComponent component;
    int frame_calls = 0;
    int pub_2d = 0;
    int pub_3d = 0;
    HestiaComponentTestApi::SetCallbacks(
        &component,
        [&](const HestiaComponent::Image&, const HestiaComponent::Image&,
            const HestiaComponent::CameraInfo&,
            HestiaComponent::Detection2DArray* d2,
            HestiaComponent::Detection3DArray* d3, std::string*) {
            ++frame_calls;
            d2->Clear();
            d3->Clear();
            return true;
        },
        [&](const HestiaComponent::Detection2DArray&) {
            ++pub_2d;
            return true;
        },
        [&](const HestiaComponent::Detection3DArray&) {
            ++pub_3d;
            return true;
        });

    EXPECT_TRUE(HestiaComponentTestApi::Process(
        &component, MakeImage(), MakeImage(), MakeCameraInfo()));
    EXPECT_EQ(frame_calls, 1);
    EXPECT_EQ(pub_2d, 1);
    EXPECT_EQ(pub_3d, 1);
}

TEST(HestiaComponentTest, DoesNotPublishWhenFrameFails) {
    HestiaComponent component;
    int pub_2d = 0;
    HestiaComponentTestApi::SetCallbacks(
        &component,
        [&](const HestiaComponent::Image&, const HestiaComponent::Image&,
            const HestiaComponent::CameraInfo&,
            HestiaComponent::Detection2DArray*,
            HestiaComponent::Detection3DArray*, std::string* error) {
            if (error != nullptr) {
                *error = "boom";
            }
            return false;
        },
        [&](const HestiaComponent::Detection2DArray&) {
            ++pub_2d;
            return true;
        },
        [](const HestiaComponent::Detection3DArray&) { return true; });

    EXPECT_FALSE(HestiaComponentTestApi::Process(
        &component, MakeImage(), MakeImage(), MakeCameraInfo()));
    EXPECT_EQ(pub_2d, 0);
}

TEST(HestiaComponentTest, ReportsPublishFailure) {
    HestiaComponent component;
    HestiaComponentTestApi::SetCallbacks(
        &component,
        [&](const HestiaComponent::Image&, const HestiaComponent::Image&,
            const HestiaComponent::CameraInfo&,
            HestiaComponent::Detection2DArray*,
            HestiaComponent::Detection3DArray*, std::string*) {
            return true;
        },
        [](const HestiaComponent::Detection2DArray&) { return false; },
        [](const HestiaComponent::Detection3DArray&) { return true; });

    EXPECT_FALSE(HestiaComponentTestApi::Process(
        &component, MakeImage(), MakeImage(), MakeCameraInfo()));
}

}  // namespace
}  // namespace autonomy::perception::hestia
