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
 * @file policy_test.cpp
 * @brief Contract tests for the Shadow ground motion-primitive policy.
 */

#include "autonomy/perception/shadow/policy.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace autonomy {
namespace perception {
namespace shadow {
namespace {

using ElementType = common::network::ElementType;
using ModelTensorInfo = common::network::ModelTensorInfo;
using Tensor = common::network::Tensor;
using TensorMap = common::network::TensorMap;
using TensorShape = common::network::TensorShape;
using Image = automsgs::msgs::sensor_msgs::Image;
using Odometry = automsgs::msgs::nav_msgs::Odometry;
using Path = automsgs::msgs::nav_msgs::Path;
using PoseStamped = automsgs::msgs::geometry_msgs::PoseStamped;
using TwistStamped = automsgs::msgs::geometry_msgs::TwistStamped;

constexpr double kPi = 3.14159265358979323846;

template <typename T, typename = void>
struct HasPublicMetadataAccess : std::false_type {
};

template <typename T>
struct HasPublicMetadataAccess<
    T, std::void_t<decltype(std::declval<const T&>().GetInputInfos()),
                   decltype(std::declval<const T&>().GetOutputInfos())>>
    : std::true_type {
};

static_assert(!HasPublicMetadataAccess<PolicyRunner>::value,
              "PolicyRunner metadata must not be part of its public API");

proto::ShadowOptions Options(uint32_t candidate_count = 1,
                             uint32_t trajectory_steps = 2) {
    proto::ShadowOptions options;
    options.set_detector_model_path("/models/shadow_detector.onnx");
    options.set_policy_model_path("/models/shadow_policy.onnx");
    options.set_detector_backend("onnx");
    options.set_policy_backend("onnx");
    options.set_detector_width(640);
    options.set_detector_height(640);
    options.set_max_detections(300);
    options.set_person_class_id(0);
    options.set_confidence_threshold(0.35F);
    options.set_track_high_threshold(0.50F);
    options.set_track_low_threshold(0.10F);
    options.set_association_iou_threshold(0.30F);
    options.set_min_confirmed_hits(3);
    options.set_prediction_timeout_sec(0.35F);
    options.set_lost_timeout_sec(1.5F);
    options.set_inner_box_scale(0.50F);
    options.set_min_depth_m(0.20F);
    options.set_max_depth_m(8.0F);
    options.set_min_depth_samples(12);
    options.set_depth_outlier_m(0.25F);
    options.set_map_frame("map");
    options.set_base_frame("base_link");
    options.set_camera_frame("camera_link");
    options.set_map_length_x(10.0F);
    options.set_map_length_y(10.0F);
    options.set_map_resolution(0.05F);
    options.set_map_roll_threshold(0.35F);
    options.set_cell_ttl_sec(1.0F);
    options.set_max_step_height(0.20F);
    options.set_max_slope_rad(0.35F);
    options.set_obstacle_min_height(0.10F);
    options.set_robot_radius(0.25F);
    options.set_inflation_radius(0.35F);
    options.set_policy_width(2);
    options.set_policy_height(2);
    options.set_candidate_count(candidate_count);
    options.set_trajectory_steps(trajectory_steps);
    options.set_max_linear_speed(0.8F);
    options.set_max_angular_speed(1.2F);
    options.set_follow_distance(1.5F);
    options.set_trajectory_step_sec(0.1F);
    options.set_learned_weight(1.0F);
    options.set_clearance_weight(1.0F);
    options.set_traversability_weight(1.0F);
    options.set_curvature_weight(1.0F);
    options.set_progress_weight(1.0F);
    options.set_distance_weight(1.0F);
    options.set_visibility_weight(1.0F);
    options.set_select_topic("/perception/shadow/select");
    options.set_detections_topic("/perception/shadow/detections");
    options.set_target_topic("/perception/shadow/target");
    options.set_path_topic("/perception/shadow/path");
    options.set_grid_topic("/perception/shadow/grid");
    options.set_max_input_skew_sec(0.10F);
    options.set_max_data_age_sec(0.50F);
    options.set_depth_scale(0.001F);
    return options;
}

std::vector<ModelTensorInfo> InputInfos(const proto::ShadowOptions& options) {
    const int64_t height = static_cast<int64_t>(options.policy_height());
    const int64_t width = static_cast<int64_t>(options.policy_width());
    return {
        {"depth", TensorShape({1, 1, height, width}), ElementType::kFloat32},
        {"robot_state", TensorShape({1, 2}), ElementType::kFloat32},
        {"target_state", TensorShape({1, 4}), ElementType::kFloat32}};
}

std::vector<ModelTensorInfo> OutputInfos(const proto::ShadowOptions& options) {
    const int64_t candidates = static_cast<int64_t>(options.candidate_count());
    const int64_t steps = static_cast<int64_t>(options.trajectory_steps());
    return {{"trajectories", TensorShape({1, candidates, steps, 3}),
             ElementType::kFloat32},
            {"scores", TensorShape({1, candidates}), ElementType::kFloat32}};
}

class FakePolicyRunner final : public PolicyRunner
{
public:
    FakePolicyRunner(std::vector<ModelTensorInfo> input_infos,
                     std::vector<ModelTensorInfo> output_infos,
                     std::vector<float> trajectories, std::vector<float> scores,
                     bool succeeds = true)
        : PolicyRunner(std::move(input_infos), std::move(output_infos)),
          trajectories_(std::move(trajectories)),
          scores_(std::move(scores)),
          succeeds_(succeeds) {}

    bool Run(const TensorMap& inputs, TensorMap* outputs,
             std::string* error) override {
        captured_inputs_ = inputs;
        if (outputs != nullptr) {
            outputs->clear();
        }
        if (!succeeds_) {
            if (error != nullptr) {
                *error = "fake inference failure";
            }
            return false;
        }
        if (outputs != nullptr) {
            outputs->emplace("trajectories",
                             Tensor::FromFloat32(trajectories_));
            outputs->emplace("scores", Tensor::FromFloat32(scores_));
        }
        return true;
    }

    const TensorMap& captured_inputs() const {
        return captured_inputs_;
    }

private:
    std::vector<float> trajectories_;
    std::vector<float> scores_;
    bool succeeds_;
    TensorMap captured_inputs_;
};

std::unique_ptr<FakePolicyRunner> MakePolicyRunner(
    const proto::ShadowOptions& options, std::vector<float> trajectories,
    std::vector<float> scores, bool succeeds = true) {
    return std::make_unique<FakePolicyRunner>(
        InputInfos(options), OutputInfos(options), std::move(trajectories),
        std::move(scores), succeeds);
}

Image Depth(const std::vector<float>& values = {1.0F, 2.0F}, uint32_t width = 2,
            uint32_t height = 1, uint32_t padding_bytes = 0) {
    Image depth;
    depth.mutable_header()->set_frame_id("camera_link");
    depth.mutable_header()->mutable_stamp()->set_sec(7);
    depth.mutable_header()->mutable_stamp()->set_nanosec(11);
    depth.set_encoding("32FC1");
    depth.set_width(width);
    depth.set_height(height);
    depth.set_is_bigendian(false);
    depth.set_step(width * sizeof(float) + padding_bytes);
    depth.mutable_data()->assign(static_cast<size_t>(height) * depth.step(),
                                 '\0');
    for (uint32_t row = 0; row < height; ++row) {
        std::memcpy(depth.mutable_data()->data() +
                        static_cast<size_t>(row) * depth.step(),
                    values.data() + static_cast<size_t>(row) * width,
                    static_cast<size_t>(width) * sizeof(float));
    }
    return depth;
}

Odometry Odom(double x = 0.0, double y = 0.0, double yaw = 0.0,
              double linear_speed = 0.4, double angular_speed = -0.3) {
    Odometry odometry;
    odometry.mutable_header()->set_frame_id("map");
    odometry.set_child_frame_id("base_link");
    odometry.mutable_pose()
        ->mutable_pose()
        ->mutable_pose()
        ->mutable_position()
        ->set_x(x);
    odometry.mutable_pose()
        ->mutable_pose()
        ->mutable_pose()
        ->mutable_position()
        ->set_y(y);
    auto* orientation = odometry.mutable_pose()
                            ->mutable_pose()
                            ->mutable_pose()
                            ->mutable_orientation();
    orientation->set_z(std::sin(yaw * 0.5));
    orientation->set_w(std::cos(yaw * 0.5));
    odometry.mutable_twist()->mutable_twist()->mutable_linear()->set_x(
        linear_speed);
    odometry.mutable_twist()->mutable_twist()->mutable_angular()->set_z(
        angular_speed);
    return odometry;
}

PoseStamped Target(double x = 3.0, double y = 4.0) {
    PoseStamped target;
    target.mutable_header()->set_frame_id("map");
    target.mutable_pose()->mutable_position()->set_x(x);
    target.mutable_pose()->mutable_position()->set_y(y);
    target.mutable_pose()->mutable_orientation()->set_w(1.0);
    return target;
}

TwistStamped TargetVelocity(double x = 1.0, double y = 2.0) {
    TwistStamped velocity;
    velocity.mutable_header()->set_frame_id("map");
    velocity.mutable_twist()->mutable_linear()->set_x(x);
    velocity.mutable_twist()->mutable_linear()->set_y(y);
    return velocity;
}

void SeedOutputs(std::vector<Path>* paths, std::vector<float>* scores) {
    paths->emplace_back().mutable_header()->set_frame_id("stale");
    scores->push_back(99.0F);
}

TEST(YopoPolicyTest, BuildsExactGroundStateInputsAndSanitizesMetricDepth) {
    auto options = Options();
    options.set_policy_width(4);
    auto runner = MakePolicyRunner(
        options, {0.0F, 0.0F, 0.0F, 0.5F, 0.1F, 0.2F}, {0.25F});
    FakePolicyRunner* runner_ptr = runner.get();
    auto policy = YopoPolicy::Create(options, std::move(runner));
    std::vector<Path> paths;
    std::vector<float> scores;

    ASSERT_NE(policy, nullptr);
    ASSERT_TRUE(policy->Generate(
        Depth({1.0F, std::numeric_limits<float>::quiet_NaN(), 0.1F, 9.0F}, 4, 1,
              8),
        Odom(1.0, 2.0, kPi / 2.0), Target(), TargetVelocity(), &paths,
        &scores));

    const TensorMap& inputs = runner_ptr->captured_inputs();
    ASSERT_EQ(inputs.size(), 3U);
    ASSERT_EQ(inputs.at("depth").element_count(), 8U);
    ASSERT_EQ(inputs.at("robot_state").element_count(), 2U);
    ASSERT_EQ(inputs.at("target_state").element_count(), 4U);
    const float* depth = inputs.at("depth").data_as<float>();
    const float* robot = inputs.at("robot_state").data_as<float>();
    const float* target = inputs.at("target_state").data_as<float>();
    ASSERT_NE(depth, nullptr);
    ASSERT_NE(robot, nullptr);
    ASSERT_NE(target, nullptr);
    EXPECT_FLOAT_EQ(depth[0], 1.0F);
    EXPECT_FLOAT_EQ(depth[1], 0.0F);
    EXPECT_FLOAT_EQ(depth[2], 0.0F);
    EXPECT_FLOAT_EQ(depth[3], 0.0F);
    EXPECT_FLOAT_EQ(depth[4], 1.0F);
    EXPECT_FLOAT_EQ(depth[5], 0.0F);
    EXPECT_FLOAT_EQ(depth[6], 0.0F);
    EXPECT_FLOAT_EQ(depth[7], 0.0F);
    EXPECT_FLOAT_EQ(robot[0], 0.4F);
    EXPECT_FLOAT_EQ(robot[1], -0.3F);
    EXPECT_NEAR(target[0], 2.0F, 1.0e-6F);
    EXPECT_NEAR(target[1], -2.0F, 1.0e-6F);
    EXPECT_NEAR(target[2], 2.0F, 1.0e-6F);
    EXPECT_NEAR(target[3], -1.0F, 1.0e-6F);
}

TEST(YopoPolicyTest, DecodesGroundTrajectoryInBaseLink) {
    auto options = Options();
    auto runner = MakePolicyRunner(
        options, {0.0F, 0.0F, 0.0F, 0.5F, 0.1F, 0.2F}, {0.25F});
    auto policy = YopoPolicy::Create(options, std::move(runner));
    std::vector<Path> paths;
    std::vector<float> scores;

    ASSERT_NE(policy, nullptr);
    ASSERT_TRUE(policy->Generate(Depth(), Odom(), Target(), TargetVelocity(),
                                 &paths, &scores));
    ASSERT_EQ(paths.size(), 1U);
    ASSERT_EQ(scores.size(), 1U);
    ASSERT_EQ(paths.front().poses_size(), 2);
    EXPECT_EQ(paths.front().header().frame_id(), "base_link");
    EXPECT_EQ(paths.front().header().stamp().sec(), 7);
    EXPECT_EQ(paths.front().header().stamp().nanosec(), 11U);
    EXPECT_EQ(paths.front().poses(1).header().frame_id(), "base_link");
    EXPECT_DOUBLE_EQ(paths.front().poses(0).pose().position().x(), 0.0);
    EXPECT_DOUBLE_EQ(paths.front().poses(1).pose().position().x(), 0.5);
    EXPECT_NEAR(paths.front().poses(1).pose().position().y(), 0.1, 1.0e-6);
    EXPECT_DOUBLE_EQ(paths.front().poses(1).pose().position().z(), 0.0);
    EXPECT_DOUBLE_EQ(paths.front().poses(1).pose().orientation().x(), 0.0);
    EXPECT_DOUBLE_EQ(paths.front().poses(1).pose().orientation().y(), 0.0);
    EXPECT_NEAR(paths.front().poses(1).pose().orientation().z(), std::sin(0.1),
                1.0e-7);
    EXPECT_NEAR(paths.front().poses(1).pose().orientation().w(), std::cos(0.1),
                1.0e-7);
    EXPECT_FLOAT_EQ(scores.front(), 0.25F);
}

TEST(YopoPolicyTest, PreservesCandidateAndStepOrderWithLowerScoresUnchanged) {
    auto options = Options(2, 2);
    auto runner = MakePolicyRunner(options,
                                   {1.0F, 1.1F, 0.1F, 1.2F, 1.3F, 0.2F, 2.0F,
                                    2.1F, -0.1F, 2.2F, 2.3F, -0.2F},
                                   {0.75F, -0.25F});
    auto policy = YopoPolicy::Create(options, std::move(runner));
    std::vector<Path> paths;
    std::vector<float> scores;

    ASSERT_NE(policy, nullptr);
    ASSERT_TRUE(policy->Generate(Depth(), Odom(), Target(), TargetVelocity(),
                                 &paths, &scores));
    ASSERT_EQ(paths.size(), 2U);
    ASSERT_EQ(paths[0].poses_size(), 2);
    ASSERT_EQ(paths[1].poses_size(), 2);
    EXPECT_DOUBLE_EQ(paths[0].poses(0).pose().position().x(), 1.0);
    EXPECT_NEAR(paths[0].poses(1).pose().position().x(), 1.2, 1.0e-6);
    EXPECT_DOUBLE_EQ(paths[1].poses(0).pose().position().x(), 2.0);
    EXPECT_NEAR(paths[1].poses(1).pose().position().x(), 2.2, 1.0e-6);
    ASSERT_EQ(scores.size(), 2U);
    EXPECT_FLOAT_EQ(scores[0], 0.75F);
    EXPECT_FLOAT_EQ(scores[1], -0.25F);
    EXPECT_LT(scores[1], scores[0]);
}

TEST(YopoPolicyTest, RejectsWrongTrajectoryMetadataRankAtCreation) {
    const auto options = Options();
    auto output_infos = OutputInfos(options);
    output_infos[0].shape = TensorShape({1, 1, 6});
    auto runner = std::make_unique<FakePolicyRunner>(
        InputInfos(options), std::move(output_infos),
        std::vector<float>(6, 0.0F), std::vector<float>{0.0F});
    std::string error;

    EXPECT_EQ(YopoPolicy::Create(options, std::move(runner), &error), nullptr);
    EXPECT_FALSE(error.empty());
}

TEST(YopoPolicyTest, RejectsWrongCandidateMetadataCountAtCreation) {
    const auto options = Options();
    auto output_infos = OutputInfos(options);
    output_infos[0].shape = TensorShape({1, 2, 2, 3});
    auto runner = std::make_unique<FakePolicyRunner>(
        InputInfos(options), std::move(output_infos),
        std::vector<float>(12, 0.0F), std::vector<float>{0.0F});
    std::string error;

    EXPECT_EQ(YopoPolicy::Create(options, std::move(runner), &error), nullptr);
    EXPECT_FALSE(error.empty());
}

TEST(YopoPolicyTest, RejectsNonFiniteTrajectoryAndClearsOutputs) {
    auto options = Options();
    auto runner = MakePolicyRunner(
        options,
        {0.0F, 0.0F, 0.0F, std::numeric_limits<float>::infinity(), 0.0F, 0.0F},
        {0.25F});
    auto policy = YopoPolicy::Create(options, std::move(runner));
    std::vector<Path> paths;
    std::vector<float> scores;
    SeedOutputs(&paths, &scores);
    std::string error;

    ASSERT_NE(policy, nullptr);
    EXPECT_FALSE(policy->Generate(Depth(), Odom(), Target(), TargetVelocity(),
                                  &paths, &scores, &error));
    EXPECT_TRUE(paths.empty());
    EXPECT_TRUE(scores.empty());
    EXPECT_FALSE(error.empty());
}

TEST(YopoPolicyTest, RejectsNonFiniteScoreAndClearsOutputs) {
    auto options = Options();
    auto runner = MakePolicyRunner(options, std::vector<float>(6, 0.0F),
                                   {std::numeric_limits<float>::quiet_NaN()});
    auto policy = YopoPolicy::Create(options, std::move(runner));
    std::vector<Path> paths;
    std::vector<float> scores;
    SeedOutputs(&paths, &scores);

    ASSERT_NE(policy, nullptr);
    EXPECT_FALSE(policy->Generate(Depth(), Odom(), Target(), TargetVelocity(),
                                  &paths, &scores));
    EXPECT_TRUE(paths.empty());
    EXPECT_TRUE(scores.empty());
}

TEST(YopoPolicyTest, RejectsRuntimeCandidateSizeAndClearsOutputs) {
    auto options = Options(2, 2);
    auto runner =
        MakePolicyRunner(options, std::vector<float>(6, 0.0F), {0.0F, 1.0F});
    auto policy = YopoPolicy::Create(options, std::move(runner));
    std::vector<Path> paths;
    std::vector<float> scores;
    SeedOutputs(&paths, &scores);

    ASSERT_NE(policy, nullptr);
    EXPECT_FALSE(policy->Generate(Depth(), Odom(), Target(), TargetVelocity(),
                                  &paths, &scores));
    EXPECT_TRUE(paths.empty());
    EXPECT_TRUE(scores.empty());
}

TEST(YopoPolicyTest, RejectsRuntimeScoreCountAndClearsOutputs) {
    auto options = Options(2, 2);
    auto runner =
        MakePolicyRunner(options, std::vector<float>(12, 0.0F), {0.0F});
    auto policy = YopoPolicy::Create(options, std::move(runner));
    std::vector<Path> paths;
    std::vector<float> scores;
    SeedOutputs(&paths, &scores);
    std::string error;

    ASSERT_NE(policy, nullptr);
    EXPECT_FALSE(policy->Generate(Depth(), Odom(), Target(), TargetVelocity(),
                                  &paths, &scores, &error));
    EXPECT_TRUE(paths.empty());
    EXPECT_TRUE(scores.empty());
    EXPECT_FALSE(error.empty());
}

TEST(YopoPolicyTest, ClearsOutputsWhenRunnerFails) {
    auto options = Options();
    auto runner = MakePolicyRunner(options, {}, {}, false);
    auto policy = YopoPolicy::Create(options, std::move(runner));
    std::vector<Path> paths;
    std::vector<float> scores;
    SeedOutputs(&paths, &scores);
    std::string error;

    ASSERT_NE(policy, nullptr);
    EXPECT_FALSE(policy->Generate(Depth(), Odom(), Target(), TargetVelocity(),
                                  &paths, &scores, &error));
    EXPECT_TRUE(paths.empty());
    EXPECT_TRUE(scores.empty());
    EXPECT_EQ(error, "Shadow policy: fake inference failure");
}

}  // namespace
}  // namespace shadow
}  // namespace perception
}  // namespace autonomy
