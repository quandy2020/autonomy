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
 * @file shadow_component_test.cpp
 * @brief Lifecycle and fail-closed publication tests for ShadowComponent.
 */

#include "autonomy/perception/shadow/shadow_component.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

namespace autonomy::perception::shadow {

class ShadowComponentTestApi
{
public:
    using Outputs = ShadowComponent::FrameOutputs;
    using ProcessFunction = ShadowComponent::ProcessFunction;
    using DetectionPublisher = ShadowComponent::DetectionPublisher;
    using TargetPublisher = ShadowComponent::TargetPublisher;
    using PathPublisher = ShadowComponent::PathPublisher;
    using GridPublisher = ShadowComponent::GridPublisher;

    static void Configure(ShadowComponent* component,
                          const proto::ShadowOptions& options,
                          ShadowComponent::ClockFunction now,
                          ShadowComponent::SelectionFunction select,
                          ShadowComponent::TransformLookup lookup,
                          ShadowComponent::ProcessFunction process,
                          ShadowComponent::DetectionPublisher detections,
                          ShadowComponent::TargetPublisher target,
                          ShadowComponent::PathPublisher path,
                          ShadowComponent::GridPublisher grid) {
        component->options_ = options;
        component->now_ = std::move(now);
        component->select_ = std::move(select);
        component->lookup_transform_ = std::move(lookup);
        component->process_ = std::move(process);
        component->publish_detections_ = std::move(detections);
        component->publish_target_ = std::move(target);
        component->publish_path_ = std::move(path);
        component->publish_grid_ = std::move(grid);
        component->initialized_ = true;
    }

    static bool Process(
        ShadowComponent* component,
        const std::shared_ptr<ShadowComponent::Image>& rgb,
        const std::shared_ptr<ShadowComponent::Image>& depth,
        const std::shared_ptr<ShadowComponent::CameraInfo>& camera,
        const std::shared_ptr<ShadowComponent::Odometry>& odometry) {
        return component->Proc(rgb, depth, camera, odometry);
    }

    static void Select(
        ShadowComponent* component,
        const std::shared_ptr<ShadowComponent::Selection>& selection) {
        component->HandleSelection(selection);
    }

    static void Clear(ShadowComponent* component) {
        component->Clear();
    }
};

namespace {

constexpr int64_t kStampNs = 10'000'000'000LL;
constexpr uint64_t kFreshNowNs = 10'100'000'000ULL;

struct Inputs {
    std::shared_ptr<ShadowComponent::Image> rgb;
    std::shared_ptr<ShadowComponent::Image> depth;
    std::shared_ptr<ShadowComponent::CameraInfo> camera;
    std::shared_ptr<ShadowComponent::Odometry> odometry;
};

void SetStamp(int64_t stamp_ns, automsgs::msgs::std_msgs::Header* header) {
    header->mutable_stamp()->set_sec(
        static_cast<int32_t>(stamp_ns / 1'000'000'000LL));
    header->mutable_stamp()->set_nanosec(
        static_cast<uint32_t>(stamp_ns % 1'000'000'000LL));
}

proto::ShadowOptions ComponentOptions() {
    proto::ShadowOptions options;
    options.set_map_frame("map");
    options.set_base_frame("base_link");
    options.set_camera_frame("camera_link");
    options.set_max_input_skew_sec(0.10F);
    options.set_max_data_age_sec(0.50F);
    return options;
}

Inputs ValidInputs(int64_t stamp_ns = kStampNs) {
    Inputs inputs;
    inputs.rgb = std::make_shared<ShadowComponent::Image>();
    SetStamp(stamp_ns, inputs.rgb->mutable_header());
    inputs.rgb->mutable_header()->set_frame_id("camera_link");
    inputs.rgb->set_width(2);
    inputs.rgb->set_height(2);
    inputs.rgb->set_encoding("rgb8");
    inputs.rgb->set_step(6);
    inputs.rgb->set_data(std::string(12, '\0'));

    inputs.depth = std::make_shared<ShadowComponent::Image>();
    SetStamp(stamp_ns, inputs.depth->mutable_header());
    inputs.depth->mutable_header()->set_frame_id("camera_link");
    inputs.depth->set_width(2);
    inputs.depth->set_height(2);
    inputs.depth->set_encoding("32FC1");
    inputs.depth->set_step(8);
    inputs.depth->set_data(std::string(16, '\0'));

    inputs.camera = std::make_shared<ShadowComponent::CameraInfo>();
    SetStamp(stamp_ns, inputs.camera->mutable_header());
    inputs.camera->mutable_header()->set_frame_id("camera_link");
    inputs.camera->set_width(2);
    inputs.camera->set_height(2);
    inputs.camera->add_k(100.0);
    inputs.camera->add_k(0.0);
    inputs.camera->add_k(0.5);
    inputs.camera->add_k(0.0);
    inputs.camera->add_k(100.0);
    inputs.camera->add_k(0.5);
    inputs.camera->add_k(0.0);
    inputs.camera->add_k(0.0);
    inputs.camera->add_k(1.0);

    inputs.odometry = std::make_shared<ShadowComponent::Odometry>();
    SetStamp(stamp_ns, inputs.odometry->mutable_header());
    inputs.odometry->mutable_header()->set_frame_id("map");
    inputs.odometry->set_child_frame_id("base_link");
    inputs.odometry->mutable_pose()
        ->mutable_pose()
        ->mutable_pose()
        ->mutable_orientation()
        ->set_w(1.0);
    return inputs;
}

bool IdentityLookup(const ShadowComponent::Image& rgb,
                    ShadowComponent::TransformStamped* transform,
                    std::string*) {
    transform->Clear();
    *transform->mutable_header()->mutable_stamp() = rgb.header().stamp();
    transform->mutable_header()->set_frame_id("map");
    transform->set_child_frame_id("camera_link");
    transform->mutable_transform()->mutable_rotation()->set_w(1.0);
    return true;
}

void PopulateAllOutputs(int64_t stamp_ns,
                        ShadowComponentTestApi::Outputs* outputs) {
    outputs->Clear();
    SetStamp(stamp_ns, outputs->detections.mutable_header());
    outputs->detections.mutable_header()->set_frame_id("camera_link");
    outputs->detections.add_detections()->set_id("track-17");
    outputs->has_detections = true;

    SetStamp(stamp_ns, outputs->target.mutable_header());
    outputs->target.mutable_header()->set_frame_id("map");
    outputs->target.mutable_pose()->mutable_position()->set_x(2.5);
    outputs->target.mutable_pose()->mutable_orientation()->set_w(1.0);
    outputs->has_target = true;

    SetStamp(stamp_ns, outputs->path.mutable_header());
    outputs->path.mutable_header()->set_frame_id("map");
    outputs->path.add_poses()->mutable_pose()->mutable_orientation()->set_w(
        1.0);
    outputs->has_path = true;

    SetStamp(stamp_ns, outputs->grid.mutable_info()->mutable_header());
    outputs->grid.mutable_info()->mutable_header()->set_frame_id("map");
    outputs->grid.add_layers("elevation");
    outputs->has_grid = true;
}

bool ProcessSuccessfully(const ShadowComponent::Image&,
                         const ShadowComponent::Image&,
                         const ShadowComponent::CameraInfo&,
                         const ShadowComponent::Odometry&,
                         const ShadowComponent::TransformStamped&,
                         ShadowComponentTestApi::Outputs* outputs,
                         std::string*) {
    PopulateAllOutputs(kStampNs, outputs);
    return true;
}

ShadowComponentTestApi::ProcessFunction SuccessfulProcess() {
    return ProcessSuccessfully;
}

bool PublishDetections(const ShadowComponent::Detection2DArray&) {
    return true;
}

bool PublishTarget(const ShadowComponent::PoseStamped&) {
    return true;
}

bool PublishPath(const ShadowComponent::Path&) {
    return true;
}

bool PublishGrid(const ShadowComponent::GridMap&) {
    return true;
}

bool FailLookup(const ShadowComponent::Image&,
                ShadowComponent::TransformStamped*, std::string* error) {
    *error = "injected TF lookup failure";
    return false;
}

bool FailProcessing(const ShadowComponent::Image&,
                    const ShadowComponent::Image&,
                    const ShadowComponent::CameraInfo&,
                    const ShadowComponent::Odometry&,
                    const ShadowComponent::TransformStamped&,
                    ShadowComponentTestApi::Outputs*, std::string* error) {
    *error = "injected processing failure";
    return false;
}

void ConfigureSuccess(
    ShadowComponent* component,
    ShadowComponentTestApi::DetectionPublisher detections = PublishDetections,
    ShadowComponentTestApi::TargetPublisher target = PublishTarget,
    ShadowComponentTestApi::PathPublisher path = PublishPath,
    ShadowComponentTestApi::GridPublisher grid = PublishGrid,
    ShadowComponentTestApi::ProcessFunction process = SuccessfulProcess()) {
    ShadowComponentTestApi::Configure(
        component, ComponentOptions(), [] { return kFreshNowNs; },
        [](const std::string&) {}, IdentityLookup, std::move(process),
        std::move(detections), std::move(target), std::move(path),
        std::move(grid));
}

TEST(ShadowComponentTest, RejectsNullInputsBeforeLookupOrProcessing) {
    ShadowComponent component;
    int process_calls = 0;
    int path_calls = 0;
    ShadowComponentTestApi::Configure(
        &component, ComponentOptions(), [] { return kFreshNowNs; },
        [](const std::string&) {}, IdentityLookup,
        [&](const ShadowComponent::Image&, const ShadowComponent::Image&,
            const ShadowComponent::CameraInfo&,
            const ShadowComponent::Odometry&,
            const ShadowComponent::TransformStamped&,
            ShadowComponentTestApi::Outputs*, std::string*) {
            ++process_calls;
            return true;
        },
        [](const ShadowComponent::Detection2DArray&) { return true; },
        [](const ShadowComponent::PoseStamped&) { return true; },
        [&](const ShadowComponent::Path&) {
            ++path_calls;
            return true;
        },
        [](const ShadowComponent::GridMap&) { return true; });
    auto inputs = ValidInputs();

    EXPECT_FALSE(ShadowComponentTestApi::Process(
        &component, nullptr, inputs.depth, inputs.camera, inputs.odometry));
    EXPECT_EQ(process_calls, 0);
    EXPECT_EQ(path_calls, 0);
}

TEST(ShadowComponentTest, RejectsUninitializedState) {
    ShadowComponent component;
    const auto inputs = ValidInputs();

    EXPECT_FALSE(ShadowComponentTestApi::Process(
        &component, inputs.rgb, inputs.depth, inputs.camera, inputs.odometry));
}

TEST(ShadowComponentTest, RejectsInputSkewBeforeLookupOrPublication) {
    ShadowComponent component;
    int path_calls = 0;
    ShadowComponentTestApi::Configure(
        &component, ComponentOptions(), [] { return kFreshNowNs; },
        [](const std::string&) {}, IdentityLookup, SuccessfulProcess(),
        [](const ShadowComponent::Detection2DArray&) { return true; },
        [](const ShadowComponent::PoseStamped&) { return true; },
        [&](const ShadowComponent::Path&) {
            ++path_calls;
            return true;
        },
        [](const ShadowComponent::GridMap&) { return true; });
    auto inputs = ValidInputs();
    SetStamp(kStampNs + 200'000'000LL, inputs.depth->mutable_header());

    EXPECT_FALSE(ShadowComponentTestApi::Process(
        &component, inputs.rgb, inputs.depth, inputs.camera, inputs.odometry));
    EXPECT_EQ(path_calls, 0);
}

TEST(ShadowComponentTest, TfLookupFailurePublishesCurrentEmptyPath) {
    ShadowComponent component;
    int path_calls = 0;
    ShadowComponent::Path published;
    ShadowComponentTestApi::Configure(
        &component, ComponentOptions(), [] { return kFreshNowNs; },
        [](const std::string&) {}, FailLookup, SuccessfulProcess(),
        PublishDetections, PublishTarget,
        [&](const ShadowComponent::Path& path) {
            ++path_calls;
            published = path;
            return true;
        },
        PublishGrid);
    const auto inputs = ValidInputs();

    EXPECT_FALSE(ShadowComponentTestApi::Process(
        &component, inputs.rgb, inputs.depth, inputs.camera, inputs.odometry));
    EXPECT_EQ(path_calls, 1);
    EXPECT_EQ(published.header().frame_id(), "map");
    EXPECT_EQ(published.header().stamp().sec(), 10);
    EXPECT_EQ(published.poses_size(), 0);
}

TEST(ShadowComponentTest, PublishesCurrentStampedEmptyPathForStaleData) {
    ShadowComponent component;
    int process_calls = 0;
    int path_calls = 0;
    ShadowComponent::Path published;
    ShadowComponentTestApi::Configure(
        &component, ComponentOptions(), [] { return 11'000'000'000ULL; },
        [](const std::string&) {}, IdentityLookup,
        [&](const ShadowComponent::Image&, const ShadowComponent::Image&,
            const ShadowComponent::CameraInfo&,
            const ShadowComponent::Odometry&,
            const ShadowComponent::TransformStamped&,
            ShadowComponentTestApi::Outputs*, std::string*) {
            ++process_calls;
            return true;
        },
        [](const ShadowComponent::Detection2DArray&) { return true; },
        [](const ShadowComponent::PoseStamped&) { return true; },
        [&](const ShadowComponent::Path& path) {
            ++path_calls;
            published = path;
            return true;
        },
        [](const ShadowComponent::GridMap&) { return true; });
    const auto inputs = ValidInputs();

    EXPECT_FALSE(ShadowComponentTestApi::Process(
        &component, inputs.rgb, inputs.depth, inputs.camera, inputs.odometry));
    EXPECT_EQ(process_calls, 0);
    EXPECT_EQ(path_calls, 1);
    EXPECT_EQ(published.header().frame_id(), "map");
    EXPECT_EQ(published.header().stamp().sec(), 10);
    EXPECT_EQ(published.header().stamp().nanosec(), 0U);
    EXPECT_EQ(published.poses_size(), 0);
}

TEST(ShadowComponentTest, AppliesSelectedTargetBeforeFrameProcessing) {
    ShadowComponent component;
    std::string applied_target;
    int selection_calls = 0;
    auto process = [&](const ShadowComponent::Image&,
                       const ShadowComponent::Image&,
                       const ShadowComponent::CameraInfo&,
                       const ShadowComponent::Odometry&,
                       const ShadowComponent::TransformStamped&,
                       ShadowComponentTestApi::Outputs* outputs, std::string*) {
        EXPECT_EQ(applied_target, "track-17");
        PopulateAllOutputs(kStampNs, outputs);
        return true;
    };
    ShadowComponentTestApi::Configure(
        &component, ComponentOptions(), [] { return kFreshNowNs; },
        [&](const std::string& target) {
            ++selection_calls;
            applied_target = target;
        },
        IdentityLookup, std::move(process),
        [](const ShadowComponent::Detection2DArray&) { return true; },
        [](const ShadowComponent::PoseStamped&) { return true; },
        [](const ShadowComponent::Path&) { return true; },
        [](const ShadowComponent::GridMap&) { return true; });
    auto selection = std::make_shared<ShadowComponent::Selection>();
    selection->set_data("track-17");
    ShadowComponentTestApi::Select(&component, selection);
    const auto inputs = ValidInputs();

    EXPECT_TRUE(ShadowComponentTestApi::Process(
        &component, inputs.rgb, inputs.depth, inputs.camera, inputs.odometry));
    EXPECT_EQ(selection_calls, 1);
    EXPECT_EQ(applied_target, "track-17");
}

TEST(ShadowComponentTest, ProcessingFailureInvalidatesPathOnly) {
    ShadowComponent component;
    int detections_calls = 0;
    int target_calls = 0;
    int grid_calls = 0;
    int path_calls = 0;
    ShadowComponent::Path published;
    ConfigureSuccess(
        &component,
        [&](const ShadowComponent::Detection2DArray&) {
            ++detections_calls;
            return true;
        },
        [&](const ShadowComponent::PoseStamped&) {
            ++target_calls;
            return true;
        },
        [&](const ShadowComponent::Path& path) {
            ++path_calls;
            published = path;
            return true;
        },
        [&](const ShadowComponent::GridMap&) {
            ++grid_calls;
            return true;
        },
        FailProcessing);
    const auto inputs = ValidInputs();

    EXPECT_FALSE(ShadowComponentTestApi::Process(
        &component, inputs.rgb, inputs.depth, inputs.camera, inputs.odometry));
    EXPECT_EQ(detections_calls, 0);
    EXPECT_EQ(target_calls, 0);
    EXPECT_EQ(grid_calls, 0);
    EXPECT_EQ(path_calls, 1);
    EXPECT_EQ(published.header().frame_id(), "map");
    EXPECT_EQ(published.poses_size(), 0);
}

TEST(ShadowComponentTest, PublishesAllFourOutputsFromOneSuccessfulFrame) {
    ShadowComponent component;
    int detections_calls = 0;
    int target_calls = 0;
    int path_calls = 0;
    int grid_calls = 0;
    ConfigureSuccess(
        &component,
        [&](const ShadowComponent::Detection2DArray& detections) {
            ++detections_calls;
            EXPECT_EQ(detections.detections(0).id(), "track-17");
            return true;
        },
        [&](const ShadowComponent::PoseStamped& target) {
            ++target_calls;
            EXPECT_DOUBLE_EQ(target.pose().position().x(), 2.5);
            return true;
        },
        [&](const ShadowComponent::Path& path) {
            ++path_calls;
            EXPECT_EQ(path.poses_size(), 1);
            return true;
        },
        [&](const ShadowComponent::GridMap& grid) {
            ++grid_calls;
            EXPECT_EQ(grid.layers(0), "elevation");
            return true;
        });
    const auto inputs = ValidInputs();

    EXPECT_TRUE(ShadowComponentTestApi::Process(
        &component, inputs.rgb, inputs.depth, inputs.camera, inputs.odometry));
    EXPECT_EQ(detections_calls, 1);
    EXPECT_EQ(target_calls, 1);
    EXPECT_EQ(path_calls, 1);
    EXPECT_EQ(grid_calls, 1);
}

TEST(ShadowComponentTest, ReportsEachWriterFailureAfterAttemptingAllOutputs) {
    for (int failing_writer = 0; failing_writer < 4; ++failing_writer) {
        SCOPED_TRACE(failing_writer);
        ShadowComponent component;
        std::array<int, 4> calls{};
        ConfigureSuccess(
            &component,
            [&](const ShadowComponent::Detection2DArray&) {
                ++calls[0];
                return failing_writer != 0;
            },
            [&](const ShadowComponent::PoseStamped&) {
                ++calls[1];
                return failing_writer != 1;
            },
            [&](const ShadowComponent::Path&) {
                ++calls[2];
                return failing_writer != 2;
            },
            [&](const ShadowComponent::GridMap&) {
                ++calls[3];
                return failing_writer != 3;
            });
        const auto inputs = ValidInputs();

        EXPECT_FALSE(ShadowComponentTestApi::Process(
            &component, inputs.rgb, inputs.depth, inputs.camera,
            inputs.odometry));
        EXPECT_EQ(calls, (std::array<int, 4>{1, 1, 1, 1}));
    }
}

TEST(ShadowComponentTest, ClearRemovesCallbacksAndPendingSelection) {
    ShadowComponent component;
    ConfigureSuccess(&component);
    auto selection = std::make_shared<ShadowComponent::Selection>();
    selection->set_data("track-17");
    ShadowComponentTestApi::Select(&component, selection);

    ShadowComponentTestApi::Clear(&component);
    ShadowComponentTestApi::Clear(&component);

    int selection_calls = 0;
    ShadowComponentTestApi::Configure(
        &component, ComponentOptions(), [] { return kFreshNowNs; },
        [&](const std::string&) { ++selection_calls; }, IdentityLookup,
        SuccessfulProcess(),
        [](const ShadowComponent::Detection2DArray&) { return true; },
        [](const ShadowComponent::PoseStamped&) { return true; },
        [](const ShadowComponent::Path&) { return true; },
        [](const ShadowComponent::GridMap&) { return true; });
    const auto inputs = ValidInputs();
    EXPECT_TRUE(ShadowComponentTestApi::Process(
        &component, inputs.rgb, inputs.depth, inputs.camera, inputs.odometry));
    EXPECT_EQ(selection_calls, 0);

    ShadowComponentTestApi::Clear(&component);
    EXPECT_FALSE(ShadowComponentTestApi::Process(
        &component, inputs.rgb, inputs.depth, inputs.camera, inputs.odometry));
}

}  // namespace
}  // namespace autonomy::perception::shadow
