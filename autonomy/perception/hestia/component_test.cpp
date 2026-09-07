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
 * @file component_test.cpp
 * @brief Unit tests for Hestia component lifecycle and publication seams.
 */

#include "autonomy/perception/hestia/component.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <utility>

namespace autonomy::perception::hestia {

class ComponentTestApi
{
public:
    static void SetCallbacks(Component* component,
                             Component::FrameFunction frame,
                             Component::Detection2dPublisher publish_2d,
                             Component::Detection3dPublisher publish_3d) {
        component->frame_ = std::move(frame);
        component->publish_2d_ = std::move(publish_2d);
        component->publish_3d_ = std::move(publish_3d);
    }

    static bool Process(Component* component,
                        const std::shared_ptr<Component::Image>& rgb,
                        const std::shared_ptr<Component::Image>& depth,
                        const std::shared_ptr<Component::CameraInfo>& info) {
        return component->Proc(rgb, depth, info);
    }

    static void Clear(Component* component) { component->Clear(); }
};

namespace {

std::shared_ptr<Component::Image> MakeImage() {
    return std::make_shared<Component::Image>();
}

std::shared_ptr<Component::CameraInfo> MakeCameraInfo() {
    return std::make_shared<Component::CameraInfo>();
}

TEST(ComponentTest, RejectsNullInputs) {
    Component component;
    int frame_calls = 0;
    ComponentTestApi::SetCallbacks(
        &component,
        [&](const Component::Image&, const Component::Image&,
            const Component::CameraInfo&,
            Component::Detection2DArray*,
            Component::Detection3DArray*, std::string*) {
            ++frame_calls;
            return true;
        },
        [](const Component::Detection2DArray&) { return true; },
        [](const Component::Detection3DArray&) { return true; });

    EXPECT_FALSE(ComponentTestApi::Process(
        &component, nullptr, MakeImage(), MakeCameraInfo()));
    EXPECT_EQ(frame_calls, 0);
}

TEST(ComponentTest, RejectsUninitializedState) {
    Component component;
    EXPECT_FALSE(ComponentTestApi::Process(
        &component, MakeImage(), MakeImage(), MakeCameraInfo()));
}

TEST(ComponentTest, PublishesBothTopicsOnSuccess) {
    Component component;
    int frame_calls = 0;
    int pub_2d = 0;
    int pub_3d = 0;
    ComponentTestApi::SetCallbacks(
        &component,
        [&](const Component::Image&, const Component::Image&,
            const Component::CameraInfo&,
            Component::Detection2DArray* d2,
            Component::Detection3DArray* d3, std::string*) {
            ++frame_calls;
            d2->Clear();
            d3->Clear();
            return true;
        },
        [&](const Component::Detection2DArray&) {
            ++pub_2d;
            return true;
        },
        [&](const Component::Detection3DArray&) {
            ++pub_3d;
            return true;
        });

    EXPECT_TRUE(ComponentTestApi::Process(
        &component, MakeImage(), MakeImage(), MakeCameraInfo()));
    EXPECT_EQ(frame_calls, 1);
    EXPECT_EQ(pub_2d, 1);
    EXPECT_EQ(pub_3d, 1);
}

TEST(ComponentTest, DoesNotPublishWhenFrameFails) {
    Component component;
    int pub_2d = 0;
    ComponentTestApi::SetCallbacks(
        &component,
        [&](const Component::Image&, const Component::Image&,
            const Component::CameraInfo&,
            Component::Detection2DArray*,
            Component::Detection3DArray*, std::string* error) {
            if (error != nullptr) {
                *error = "boom";
            }
            return false;
        },
        [&](const Component::Detection2DArray&) {
            ++pub_2d;
            return true;
        },
        [](const Component::Detection3DArray&) { return true; });

    EXPECT_FALSE(ComponentTestApi::Process(
        &component, MakeImage(), MakeImage(), MakeCameraInfo()));
    EXPECT_EQ(pub_2d, 0);
}

TEST(ComponentTest, ReportsPublishFailure) {
    Component component;
    ComponentTestApi::SetCallbacks(
        &component,
        [&](const Component::Image&, const Component::Image&,
            const Component::CameraInfo&,
            Component::Detection2DArray*,
            Component::Detection3DArray*, std::string*) {
            return true;
        },
        [](const Component::Detection2DArray&) { return false; },
        [](const Component::Detection3DArray&) { return true; });

    EXPECT_FALSE(ComponentTestApi::Process(
        &component, MakeImage(), MakeImage(), MakeCameraInfo()));
}

}  // namespace
}  // namespace autonomy::perception::hestia
