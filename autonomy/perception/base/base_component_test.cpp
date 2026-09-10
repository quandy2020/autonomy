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
 * @file base_component_test.cpp
 * @brief Unit tests for base component publication seams.
 */

#include "autonomy/perception/base/base_component.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <string>

namespace autonomy::perception::base {

class ComponentTestApi
{
public:
    static void SetCallbacks(Component* component, Component::FrameFunction frame,
                             Component::DetectionPublisher publish_detections,
                             Component::DetectionPublisher publish_masks,
                             Component::ClassificationPublisher publish_classification,
                             Component::DetectionPublisher publish_poses,
                             Component::DetectionPublisher publish_obb,
                             Component::DetectionPublisher publish_tracks,
                             Component::DepthPublisher publish_depth) {
        component->frame_ = std::move(frame);
        component->publish_detections_ = std::move(publish_detections);
        component->publish_masks_ = std::move(publish_masks);
        component->publish_classification_ = std::move(publish_classification);
        component->publish_poses_ = std::move(publish_poses);
        component->publish_obb_ = std::move(publish_obb);
        component->publish_tracks_ = std::move(publish_tracks);
        component->publish_depth_ = std::move(publish_depth);
        auto enable = [&](proto::TaskKind kind) {
            auto* task = component->options_.add_tasks();
            task->set_kind(kind);
            task->set_enable(true);
            task->set_topic("/test");
            task->set_model_path("/models/test.engine");
        };
        enable(proto::TASK_DETECT);
        enable(proto::TASK_SEGMENT);
        enable(proto::TASK_CLASSIFY);
        enable(proto::TASK_POSE);
        enable(proto::TASK_OBB);
        enable(proto::TASK_TRACK);
        enable(proto::TASK_DEPTH);
    }

    static bool Process(Component* component,
                        const std::shared_ptr<Component::Image>& rgb) {
        return component->Proc(rgb);
    }
};

namespace {

TEST(BaseComponentTest, RejectsNullRgb) {
    Component component;
    int frames = 0;
    ComponentTestApi::SetCallbacks(
        &component,
        [&](const Component::Image&, Outputs*, std::string*) {
            ++frames;
            return true;
        },
        [](const Component::Detection2DArray&) { return true; },
        [](const Component::Detection2DArray&) { return true; },
        [](const Component::Classification&) { return true; },
        [](const Component::Detection2DArray&) { return true; },
        [](const Component::Detection2DArray&) { return true; },
        [](const Component::Detection2DArray&) { return true; },
        [](const Component::Image&) { return true; });

    EXPECT_FALSE(ComponentTestApi::Process(&component, nullptr));
    EXPECT_EQ(frames, 0);
}

TEST(BaseComponentTest, PublishesEnabledTasks) {
    Component component;
    int detections = 0;
    int masks = 0;
    int classify = 0;
    int poses = 0;
    int obb = 0;
    int tracks = 0;
    int depth = 0;
    ComponentTestApi::SetCallbacks(
        &component,
        [](const Component::Image&, Outputs*, std::string*) { return true; },
        [&](const Component::Detection2DArray&) {
            ++detections;
            return true;
        },
        [&](const Component::Detection2DArray&) {
            ++masks;
            return true;
        },
        [&](const Component::Classification&) {
            ++classify;
            return true;
        },
        [&](const Component::Detection2DArray&) {
            ++poses;
            return true;
        },
        [&](const Component::Detection2DArray&) {
            ++obb;
            return true;
        },
        [&](const Component::Detection2DArray&) {
            ++tracks;
            return true;
        },
        [&](const Component::Image&) {
            ++depth;
            return true;
        });

    auto rgb = std::make_shared<Component::Image>();
    EXPECT_TRUE(ComponentTestApi::Process(&component, rgb));
    EXPECT_EQ(detections, 1);
    EXPECT_EQ(masks, 1);
    EXPECT_EQ(classify, 1);
    EXPECT_EQ(poses, 1);
    EXPECT_EQ(obb, 1);
    EXPECT_EQ(tracks, 1);
    EXPECT_EQ(depth, 1);
}

}  // namespace
}  // namespace autonomy::perception::base
