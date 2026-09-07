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
#include "autonomy/transform/buffer_utils.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autonomy::perception::shadow {

class ShadowComponentTestApi
{
public:
    using Outputs = ShadowComponent::FrameOutputs;
    using ProcessFunction = ShadowComponent::ProcessFunction;
    using ConfirmedTracksFunction = ShadowComponent::ConfirmedTracksFunction;
    using ListenerStartFunction = ShadowComponent::ListenerStartFunction;
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
        component->confirmed_tracks_ =
            [](ShadowComponent::Detection2DArray* out) { out->Clear(); };
        component->lookup_transform_ = std::move(lookup);
        component->process_ = std::move(process);
        component->publish_detections_ = std::move(detections);
        component->publish_target_ = std::move(target);
        component->publish_path_ = std::move(path);
        component->publish_grid_ = std::move(grid);
        component->initialized_ = true;
    }

    static bool CanonicalizeDepth(ShadowComponent* component,
                                  const ShadowComponent::Image& input,
                                  ShadowComponent::Image* output,
                                  std::string* error) {
        return component->CanonicalizeDepth(input, output, error);
    }

    static void ConfigureAutomaticSelection(
        ShadowComponent* component, ConfirmedTracksFunction confirmed,
        ShadowComponent::SelectionFunction select) {
        component->confirmed_tracks_ = std::move(confirmed);
        component->select_ = std::move(select);
        component->localizer_ =
            std::make_unique<TargetLocalizer>(component->options_);
    }

    static void ApplyPendingSelection(ShadowComponent* component) {
        component->ApplyPendingSelection();
    }

    static bool ResolveAutomaticSelection(
        ShadowComponent* component, const ShadowComponent::Image& depth,
        const ShadowComponent::CameraInfo& camera, std::string* error) {
        return component->ResolveAutomaticSelection(depth, camera, error);
    }

    static void ConfigureListenerStart(ShadowComponent* component,
                                       ListenerStartFunction start) {
        component->listener_start_ = std::move(start);
    }

    static bool StartTransformListeners(ShadowComponent* component,
                                        std::string* error) {
        return component->StartTransformListeners(error);
    }

    static void UseTransformBuffer(ShadowComponent* component,
                                   transform::Buffer* buffer) {
        component->tf_buffer_ = buffer;
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
    options.set_inner_box_scale(1.0F);
    options.set_min_depth_m(0.20F);
    options.set_max_depth_m(10.0F);
    options.set_min_depth_samples(2);
    options.set_depth_outlier_m(0.10F);
    options.set_depth_scale(0.001F);
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

std::string Bytes(std::initializer_list<uint8_t> values) {
    std::string bytes;
    bytes.reserve(values.size());
    for (const uint8_t value : values) {
        bytes.push_back(static_cast<char>(value));
    }
    return bytes;
}

ShadowComponent::Image DepthBytes(const std::string& encoding, bool big_endian,
                                  uint32_t width, uint32_t height,
                                  uint32_t step, const std::string& data) {
    ShadowComponent::Image image;
    SetStamp(kStampNs, image.mutable_header());
    image.mutable_header()->set_frame_id("camera_link");
    image.set_encoding(encoding);
    image.set_is_bigendian(big_endian);
    image.set_width(width);
    image.set_height(height);
    image.set_step(step);
    image.set_data(data);
    return image;
}

automsgs::msgs::vision_msgs::Detection2D DetectionBox(const std::string& id,
                                                      double center_x) {
    automsgs::msgs::vision_msgs::Detection2D detection;
    detection.set_id(id);
    detection.mutable_header()->set_frame_id("camera_link");
    detection.mutable_bbox()->mutable_center()->mutable_position()->set_x(
        center_x);
    detection.mutable_bbox()->mutable_center()->mutable_position()->set_y(0.5);
    detection.mutable_bbox()->set_size_x(2.0);
    detection.mutable_bbox()->set_size_y(1.0);
    return detection;
}

ShadowComponent::CameraInfo CameraForDepth(uint32_t width, uint32_t height) {
    ShadowComponent::CameraInfo camera;
    SetStamp(kStampNs, camera.mutable_header());
    camera.mutable_header()->set_frame_id("camera_link");
    camera.set_width(width);
    camera.set_height(height);
    camera.add_k(100.0);
    camera.add_k(0.0);
    camera.add_k(static_cast<double>(width) * 0.5);
    camera.add_k(0.0);
    camera.add_k(100.0);
    camera.add_k(static_cast<double>(height) * 0.5);
    camera.add_k(0.0);
    camera.add_k(0.0);
    camera.add_k(1.0);
    return camera;
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

TEST(ShadowComponentTest, CanonicalizesSixteenBitDepthInEitherByteOrder) {
    ShadowComponent component;
    ConfigureSuccess(&component);
    const std::array<std::string, 2> encoded = {
        Bytes({0xe8, 0x03, 0xc4, 0x09, 0xaa}),
        Bytes({0x03, 0xe8, 0x09, 0xc4, 0xaa}),
    };
    const std::string expected =
        Bytes({0x00, 0x00, 0x80, 0x3f, 0x00, 0x00, 0x20, 0x40});

    for (size_t index = 0; index < encoded.size(); ++index) {
        SCOPED_TRACE(index);
        const auto input =
            DepthBytes("16UC1", index == 1, 2, 1, 5, encoded[index]);
        ShadowComponent::Image output;
        std::string error;

        ASSERT_TRUE(ShadowComponentTestApi::CanonicalizeDepth(&component, input,
                                                              &output, &error))
            << error;
        EXPECT_EQ(output.header().frame_id(), "camera_link");
        EXPECT_EQ(output.header().stamp().sec(), 10);
        EXPECT_EQ(output.width(), 2U);
        EXPECT_EQ(output.height(), 1U);
        EXPECT_EQ(output.encoding(), "32FC1");
        EXPECT_FALSE(output.is_bigendian());
        EXPECT_EQ(output.step(), 8U);
        EXPECT_EQ(output.data(), expected);
    }
}

TEST(ShadowComponentTest, CanonicalizesFloatDepthInEitherByteOrder) {
    ShadowComponent component;
    ConfigureSuccess(&component);
    const std::array<std::string, 2> encoded = {
        Bytes({0x00, 0x00, 0xa0, 0x3f, 0x00, 0x00, 0x20, 0x40, 0xaa}),
        Bytes({0x3f, 0xa0, 0x00, 0x00, 0x40, 0x20, 0x00, 0x00, 0xaa}),
    };
    const std::string expected =
        Bytes({0x00, 0x00, 0xa0, 0x3f, 0x00, 0x00, 0x20, 0x40});

    for (size_t index = 0; index < encoded.size(); ++index) {
        SCOPED_TRACE(index);
        const auto input =
            DepthBytes("32FC1", index == 1, 2, 1, 9, encoded[index]);
        ShadowComponent::Image output;
        std::string error;

        ASSERT_TRUE(ShadowComponentTestApi::CanonicalizeDepth(&component, input,
                                                              &output, &error))
            << error;
        EXPECT_EQ(output.header().frame_id(), "camera_link");
        EXPECT_EQ(output.header().stamp().sec(), 10);
        EXPECT_EQ(output.width(), 2U);
        EXPECT_EQ(output.height(), 1U);
        EXPECT_EQ(output.encoding(), "32FC1");
        EXPECT_FALSE(output.is_bigendian());
        EXPECT_EQ(output.step(), 8U);
        EXPECT_EQ(output.data(), expected);
    }
}

TEST(ShadowComponentTest,
     RejectsMalformedDepthBeforeSelectionAndPublishesEmptyPath) {
    ShadowComponent component;
    int selection_calls = 0;
    int lookup_calls = 0;
    int process_calls = 0;
    int path_calls = 0;
    ShadowComponent::Path published;
    ShadowComponentTestApi::Configure(
        &component, ComponentOptions(), [] { return kFreshNowNs; },
        [&](const std::string&) { ++selection_calls; },
        [&](const ShadowComponent::Image&, ShadowComponent::TransformStamped*,
            std::string*) {
            ++lookup_calls;
            return true;
        },
        [&](const ShadowComponent::Image&, const ShadowComponent::Image&,
            const ShadowComponent::CameraInfo&,
            const ShadowComponent::Odometry&,
            const ShadowComponent::TransformStamped&,
            ShadowComponentTestApi::Outputs*, std::string*) {
            ++process_calls;
            return true;
        },
        PublishDetections, PublishTarget,
        [&](const ShadowComponent::Path& path) {
            ++path_calls;
            published = path;
            return true;
        },
        PublishGrid);
    auto inputs = ValidInputs();
    inputs.depth->set_encoding("16UC1");
    inputs.depth->set_step(4);
    inputs.depth->set_data(std::string(7, '\0'));
    auto selection = std::make_shared<ShadowComponent::Selection>();
    selection->set_data("track-17");
    ShadowComponentTestApi::Select(&component, selection);

    EXPECT_FALSE(ShadowComponentTestApi::Process(
        &component, inputs.rgb, inputs.depth, inputs.camera, inputs.odometry));
    EXPECT_EQ(selection_calls, 0);
    EXPECT_EQ(lookup_calls, 0);
    EXPECT_EQ(process_calls, 0);
    EXPECT_EQ(path_calls, 1);
    EXPECT_EQ(published.header().frame_id(), "map");
    EXPECT_EQ(published.header().stamp().sec(), 10);
    EXPECT_EQ(published.poses_size(), 0);
}

TEST(ShadowComponentTest, PassesCanonicalDepthIntoFrameTransaction) {
    ShadowComponent component;
    bool saw_canonical_depth = false;
    const std::string expected = Bytes({
        0x00,
        0x00,
        0x80,
        0x3f,
        0x00,
        0x00,
        0x20,
        0x40,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x3f,
    });
    auto process = [&](const ShadowComponent::Image&,
                       const ShadowComponent::Image& depth,
                       const ShadowComponent::CameraInfo&,
                       const ShadowComponent::Odometry&,
                       const ShadowComponent::TransformStamped&,
                       ShadowComponentTestApi::Outputs* outputs, std::string*) {
        EXPECT_EQ(depth.encoding(), "32FC1");
        EXPECT_FALSE(depth.is_bigendian());
        EXPECT_EQ(depth.step(), 8U);
        EXPECT_EQ(depth.data(), expected);
        saw_canonical_depth = true;
        PopulateAllOutputs(kStampNs, outputs);
        return true;
    };
    ShadowComponentTestApi::Configure(
        &component, ComponentOptions(), [] { return kFreshNowNs; },
        [](const std::string&) {}, IdentityLookup, std::move(process),
        PublishDetections, PublishTarget, PublishPath, PublishGrid);
    auto inputs = ValidInputs();
    inputs.depth->set_encoding("16UC1");
    inputs.depth->set_is_bigendian(true);
    inputs.depth->set_step(5);
    inputs.depth->set_data(Bytes({
        0x03,
        0xe8,
        0x09,
        0xc4,
        0xaa,
        0x00,
        0x00,
        0x01,
        0xf4,
        0xbb,
    }));

    EXPECT_TRUE(ShadowComponentTestApi::Process(
        &component, inputs.rgb, inputs.depth, inputs.camera, inputs.odometry));
    EXPECT_TRUE(saw_canonical_depth);
}

TEST(ShadowComponentTest,
     EmptySelectionChoosesNearestValidRangeAndKeepsStableIdOnTie) {
    ShadowComponent component;
    ConfigureSuccess(&component);
    ShadowComponent::Detection2DArray confirmed;
    *confirmed.add_detections() = DetectionBox("track-00", 1.0);
    *confirmed.add_detections() = DetectionBox("track-01", 3.0);
    *confirmed.add_detections() = DetectionBox("track-02", 5.0);
    *confirmed.add_detections() = DetectionBox("track-03", 7.0);
    std::vector<std::string> selections;
    ShadowComponentTestApi::ConfigureAutomaticSelection(
        &component,
        [confirmed](ShadowComponent::Detection2DArray* output) {
            *output = confirmed;
        },
        [&](const std::string& id) { selections.push_back(id); });
    const auto raw_depth = DepthBytes("16UC1", true, 8, 1, 16,
                                      Bytes({
                                          0x00,
                                          0x00,
                                          0x00,
                                          0x00,
                                          0x05,
                                          0xdc,
                                          0x05,
                                          0xdc,
                                          0x05,
                                          0xdc,
                                          0x05,
                                          0xdc,
                                          0x0b,
                                          0xb8,
                                          0x0b,
                                          0xb8,
                                      }));
    ShadowComponent::Image depth;
    std::string error;
    ASSERT_TRUE(ShadowComponentTestApi::CanonicalizeDepth(&component, raw_depth,
                                                          &depth, &error))
        << error;
    const auto camera = CameraForDepth(8, 1);
    auto selection = std::make_shared<ShadowComponent::Selection>();
    selection->set_data("");

    ShadowComponentTestApi::Select(&component, selection);
    ShadowComponentTestApi::ApplyPendingSelection(&component);
    ASSERT_TRUE(ShadowComponentTestApi::ResolveAutomaticSelection(
        &component, depth, camera, &error))
        << error;

    ASSERT_EQ(selections.size(), 2U);
    EXPECT_EQ(selections[0], "");
    EXPECT_EQ(selections[1], "track-01");
}

TEST(ShadowComponentTest, RequiresDynamicAndStaticTfReadersIndependently) {
    for (int failing_call = 0; failing_call < 2; ++failing_call) {
        SCOPED_TRACE(failing_call);
        ShadowComponent component;
        std::vector<std::array<std::string, 3>> calls;
        ShadowComponentTestApi::ConfigureListenerStart(
            &component, [&](transform::AutolinkTfListener* listener,
                            const std::shared_ptr<autolink::Node>&,
                            const std::string& dynamic_topic,
                            const std::string& legacy_topic,
                            const std::string& static_topic) {
                EXPECT_NE(listener, nullptr);
                calls.push_back({dynamic_topic, legacy_topic, static_topic});
                return static_cast<int>(calls.size()) - 1 != failing_call;
            });
        std::string error;

        EXPECT_FALSE(ShadowComponentTestApi::StartTransformListeners(&component,
                                                                     &error));
        ASSERT_EQ(calls.size(), static_cast<size_t>(failing_call + 1));
        EXPECT_EQ(calls[0], (std::array<std::string, 3>{"/tf", "", ""}));
        if (failing_call == 1) {
            EXPECT_EQ(calls[1],
                      (std::array<std::string, 3>{"", "", "/tf_static"}));
        }
        ShadowComponentTestApi::Clear(&component);
    }

    ShadowComponent component;
    std::vector<std::array<std::string, 3>> calls;
    ShadowComponentTestApi::ConfigureListenerStart(
        &component,
        [&](transform::AutolinkTfListener*,
            const std::shared_ptr<autolink::Node>&,
            const std::string& dynamic_topic, const std::string& legacy_topic,
            const std::string& static_topic) {
            calls.push_back({dynamic_topic, legacy_topic, static_topic});
            return true;
        });
    std::string error;

    EXPECT_TRUE(
        ShadowComponentTestApi::StartTransformListeners(&component, &error));
    ASSERT_EQ(calls.size(), 2U);
    EXPECT_EQ(calls[0], (std::array<std::string, 3>{"/tf", "", ""}));
    EXPECT_EQ(calls[1], (std::array<std::string, 3>{"", "", "/tf_static"}));
    ShadowComponentTestApi::Clear(&component);
}

TEST(ShadowComponentTest, ClearRemovesTransformCacheBeforeReconfiguration) {
    auto* buffer = transform::Buffer::Instance();
    buffer->clear();
    ShadowComponent::TransformStamped stale;
    SetStamp(kStampNs, stale.mutable_header());
    stale.mutable_header()->set_frame_id("shadow_clear_parent");
    stale.set_child_frame_id("shadow_clear_child");
    stale.mutable_transform()->mutable_rotation()->set_w(1.0);
    transform::ApplyTransformStampedToBuffer(buffer, stale,
                                             "shadow_component_test", true);
    std::string error;
    ASSERT_TRUE(buffer->canTransform("shadow_clear_parent",
                                     "shadow_clear_child",
                                     stale.header().stamp(), 0.0F, &error))
        << error;

    ShadowComponent component;
    ConfigureSuccess(&component);
    ShadowComponentTestApi::UseTransformBuffer(&component, buffer);
    ShadowComponentTestApi::Clear(&component);
    EXPECT_FALSE(buffer->canTransform("shadow_clear_parent",
                                      "shadow_clear_child",
                                      stale.header().stamp(), 0.0F, &error));

    ConfigureSuccess(&component);
    ShadowComponentTestApi::UseTransformBuffer(&component, buffer);
    EXPECT_FALSE(buffer->canTransform("shadow_clear_parent",
                                      "shadow_clear_child",
                                      stale.header().stamp(), 0.0F, &error));
    ShadowComponentTestApi::Clear(&component);
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
