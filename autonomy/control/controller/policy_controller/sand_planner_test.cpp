/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#include "autonomy/common/math/bspline.hpp"
#include "autonomy/control/controller/policy_controller/config.hpp"
#include "autonomy/control/controller/policy_controller/observation/depth_encoder.hpp"
#include "autonomy/control/controller/policy_controller/sand_planner/critic.hpp"
#include "autonomy/control/controller/policy_controller/sand_planner/local_esdf.hpp"
#include "autonomy/control/controller/policy_controller/sand_planner/options.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>

#include "gtest/gtest.h"

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {

namespace math = ::autonomy::common::math;

TEST(SandPlannerBSpline, InterpolatesEndpoints) {
    std::vector<Eigen::Vector3d> controls = {
        Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(1.0, 0.0, 0.0),
        Eigen::Vector3d(2.0, 1.0, 0.0), Eigen::Vector3d(3.0, 1.0, 0.0)};
    const math::ClampedCubicBSpline<Eigen::Vector3d> curve(controls);
    const Eigen::Vector3d start = curve.Evaluate(0.0);
    const Eigen::Vector3d finish = curve.Evaluate(1.0);
    EXPECT_NEAR(start.x(), 0.0, 1e-6);
    EXPECT_NEAR(start.y(), 0.0, 1e-6);
    EXPECT_NEAR(finish.x(), 3.0, 1e-6);
    EXPECT_NEAR(finish.y(), 1.0, 1e-6);
}

TEST(SandPlannerBSpline, ArcLengthResampleCount) {
    std::vector<Eigen::Vector3d> points = {
        Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(1.0, 0.0, 0.0),
        Eigen::Vector3d(1.0, 1.0, 0.0)};
    const auto samples = math::ResamplePolylineArcLength(points, 5);
    ASSERT_EQ(samples.size(), 5u);
    EXPECT_NEAR(samples.front().x(), 0.0, 1e-6);
    EXPECT_NEAR(samples.back().x(), 1.0, 1e-6);
    EXPECT_NEAR(samples.back().y(), 1.0, 1e-6);
}

TEST(SandPlannerControlPoints, ForcesOrigin) {
    std::vector<float> raw = {9.0f, 8.0f, 7.0f, 1.0f, 0.0f, 0.0f,
                              2.0f, 0.0f, 0.0f, 3.0f, 0.0f, 0.0f};
    const auto points = ControlPointsFromFlat(raw, 4);
    ASSERT_EQ(points.size(), 4u);
    EXPECT_NEAR(points[0].norm(), 0.0, 1e-9);
    EXPECT_NEAR(points[1].x(), 1.0, 1e-6);
}

TEST(PolicyConfigTest, SandPlannerExpandsDepthGoalVelocity) {
    proto::PolicyConfig proto;
    proto.set_name("sand");
    proto.set_mode(proto::INFERENCE_DIFFUSION);
    proto.mutable_observation()->set_depth(true);
    proto.mutable_observation()->set_goal(true);
    proto.mutable_observation()->set_previous_velocity(true);
    proto.mutable_action()->set_type(proto::ACTION_SPACE_SAND_PLANNER);
    proto.mutable_engine()->set_model_path("/dev/null");

    PolicyConfig config;
    std::string error;
    ASSERT_TRUE(PolicyConfig::LoadFromProto(proto, &config, &error)) << error;
    EXPECT_TRUE(config.is_sand_planner());
    EXPECT_TRUE(config.is_diffusion());
    EXPECT_EQ(config.effective_diffusion().dim(), 3);
    EXPECT_EQ(config.effective_diffusion().horizon(), 8);
    EXPECT_EQ(config.effective_diffusion().prediction_type(),
              proto::DIFFUSION_PRED_VELOCITY);
    EXPECT_GE(config.effective_diffusion().candidates(), 1);
    EXPECT_EQ(config.proto().inputs_size(), 3);
    EXPECT_EQ(config.action_schema().type, proto::ACTION_SPACE_SAND_PLANNER);
}

TEST(PolicyConfigTest, TwistPolicyStillLoadsWithoutSandPlanner) {
    proto::PolicyConfig proto;
    proto.set_name("t");
    proto.mutable_observation()->set_kinematic(true);
    proto.mutable_engine()->set_model_path("/dev/null");
    PolicyConfig config;
    std::string error;
    ASSERT_TRUE(PolicyConfig::LoadFromProto(proto, &config, &error)) << error;
    EXPECT_FALSE(config.is_sand_planner());
    EXPECT_FALSE(config.is_diffusion());
    EXPECT_EQ(config.action_schema().type, proto::ACTION_SPACE_TWIST);
}

TEST(SandPlannerEncode, Depth16BitAndGoal) {
    ObservationSnapshot snapshot;
    automsgs::msgs::geometry_msgs::PoseStamped pose;
    pose.mutable_pose()->mutable_orientation()->set_w(1.0);
    snapshot.pose = pose;
    auto path = std::make_shared<automsgs::msgs::nav_msgs::Path>();
    auto* goal = path->add_poses();
    goal->mutable_pose()->mutable_position()->set_x(1.5);
    snapshot.path = path;

    auto image = std::make_shared<automsgs::msgs::sensor_msgs::Image>();
    image->set_width(4);
    image->set_height(4);
    image->set_encoding("16UC1");
    image->set_step(8);
    const uint16_t millimetres = 2000;
    std::string payload(4 * 4 * 2, 0);
    for (int i = 0; i < 16; ++i) {
        std::memcpy(&payload[static_cast<std::size_t>(i) * 2], &millimetres, 2);
    }
    image->set_data(payload);
    snapshot.depth_history.push_back(image);

    proto::PolicyConfig proto;
    proto.mutable_action()->set_type(proto::ACTION_SPACE_SAND_PLANNER);
    const proto::SandPlanner options = MakeEffectiveSandPlanner(proto);

    proto::ObservationInput depth_input;
    depth_input.set_name("observation_depth");
    depth_input.add_shape(1);
    depth_input.add_shape(4);
    depth_input.add_shape(8);
    depth_input.add_shape(8);
    std::vector<float> depth;
    std::string error;
    ASSERT_TRUE(EncodeDepth(snapshot, depth_input, options, &depth, &error))
        << error;
    ASSERT_EQ(depth.size(), 4u * 8u * 8u);
    EXPECT_NEAR(depth[0], 2.0f, 1e-3);

    proto::ObservationInput goal_input;
    goal_input.set_name("observation_goal");
    std::vector<float> goal_tensor;
    ASSERT_TRUE(EncodeGoal(snapshot, goal_input, &goal_tensor, &error))
        << error;
    ASSERT_GE(goal_tensor.size(), 3u);
    EXPECT_NEAR(goal_tensor[0], 1.5f, 1e-5);
}

TEST(SandPlannerLocalEsdf, OccupiedWallFromDepth) {
    ObservationSnapshot snapshot;
    auto image = std::make_shared<automsgs::msgs::sensor_msgs::Image>();
    const int width = 32;
    const int height = 24;
    image->set_width(width);
    image->set_height(height);
    image->set_encoding("32FC1");
    image->set_step(static_cast<uint32_t>(width * 4));
    std::string payload(static_cast<std::size_t>(width * height * 4), 0);
    const float depth = 1.5f;
    for (int i = 0; i < width * height; ++i) {
        std::memcpy(&payload[static_cast<std::size_t>(i) * 4], &depth, 4);
    }
    image->set_data(payload);
    snapshot.depth_history.push_back(image);

    proto::PolicyConfig proto;
    proto.mutable_action()->set_type(proto::ACTION_SPACE_SAND_PLANNER);
    auto* sand = proto.mutable_sand_planner();
    sand->set_depth_optical_frame(true);
    sand->set_pixel_stride(1);
    sand->set_source_width(static_cast<float>(width));
    sand->set_source_height(static_cast<float>(height));
    sand->set_focal_x(20.0f);
    sand->set_focal_y(20.0f);
    sand->set_principal_x(16.0f);
    sand->set_principal_y(12.0f);
    const proto::SandPlanner options = MakeEffectiveSandPlanner(proto);

    std::string error;
    const auto map = BuildLocalEsdfFromDepth(snapshot, options, &error);
    ASSERT_TRUE(map != nullptr) << error;
    const double on_wall =
        QueryLocalClearance(*map, Eigen::Vector3d(1.5, 0.0, 0.0));
    const double in_front =
        QueryLocalClearance(*map, Eigen::Vector3d(0.4, 0.0, 0.0));
    ASSERT_TRUE(std::isfinite(on_wall));
    ASSERT_TRUE(std::isfinite(in_front));
    EXPECT_LT(on_wall, 0.25);
    EXPECT_GT(in_front, on_wall);
}

TEST(SandPlannerCritic, SelectsForwardCandidate) {
    ObservationSnapshot snapshot;
    automsgs::msgs::geometry_msgs::PoseStamped pose;
    pose.mutable_pose()->mutable_orientation()->set_w(1.0);
    snapshot.pose = pose;
    auto path = std::make_shared<automsgs::msgs::nav_msgs::Path>();
    auto* goal = path->add_poses();
    goal->mutable_pose()->mutable_position()->set_x(4.0);
    snapshot.path = path;

    proto::PolicyConfig proto;
    proto.mutable_action()->set_type(proto::ACTION_SPACE_SAND_PLANNER);
    const proto::SandPlanner options = MakeEffectiveSandPlanner(proto);

    std::vector<float> forward = {0, 0, 0, 1, 0, 0, 2, 0, 0, 3, 0, 0,
                                  4, 0, 0, 5, 0, 0, 6, 0, 0, 7, 0, 0};
    std::vector<float> sideways = {0, 0, 0, 0, 1, 0, 0, 2, 0, 0, 3, 0,
                                   0, 4, 0, 0, 5, 0, 0, 6, 0, 0, 7, 0};
    automsgs::msgs::geometry_msgs::Twist twist;
    Eigen::Vector3d heading;
    std::string error;
    ASSERT_TRUE(SelectSandPlannerTwist({forward, sideways}, snapshot, options,
                                       &twist, &heading, &error))
        << error;
    EXPECT_GT(std::fabs(twist.linear().x()), std::fabs(twist.linear().y()));
    EXPECT_GT(heading.x(), 0.5);
}

TEST(SandPlannerCritic, PrefersClearPathAroundWall) {
    ObservationSnapshot snapshot;
    automsgs::msgs::geometry_msgs::PoseStamped pose;
    pose.mutable_pose()->mutable_orientation()->set_w(1.0);
    snapshot.pose = pose;
    auto path = std::make_shared<automsgs::msgs::nav_msgs::Path>();
    auto* goal = path->add_poses();
    goal->mutable_pose()->mutable_position()->set_x(3.0);
    snapshot.path = path;

    auto image = std::make_shared<automsgs::msgs::sensor_msgs::Image>();
    const int width = 32;
    const int height = 24;
    image->set_width(width);
    image->set_height(height);
    image->set_encoding("32FC1");
    image->set_step(static_cast<uint32_t>(width * 4));
    std::string payload(static_cast<std::size_t>(width * height * 4), 0);
    const float depth = 1.5f;
    for (int i = 0; i < width * height; ++i) {
        std::memcpy(&payload[static_cast<std::size_t>(i) * 4], &depth, 4);
    }
    image->set_data(payload);
    snapshot.depth_history.push_back(image);

    proto::PolicyConfig proto;
    proto.mutable_action()->set_type(proto::ACTION_SPACE_SAND_PLANNER);
    auto* sand = proto.mutable_sand_planner();
    sand->set_depth_optical_frame(true);
    sand->set_pixel_stride(1);
    sand->set_source_width(static_cast<float>(width));
    sand->set_source_height(static_cast<float>(height));
    sand->set_focal_x(20.0f);
    sand->set_focal_y(20.0f);
    sand->set_principal_x(16.0f);
    sand->set_principal_y(12.0f);
    sand->set_preferred_clearance(10.0f);
    const proto::SandPlanner options = MakeEffectiveSandPlanner(proto);

    std::vector<float> through_wall = {0, 0, 0, 1, 0, 0, 2, 0, 0, 3, 0, 0,
                                       3, 0, 0, 3, 0, 0, 3, 0, 0, 3, 0, 0};
    std::vector<float> around = {0, 0, 0, 0.3f, 1.8f, 0, 0.8f, 1.9f, 0,
                                 1.5f, 1.9f, 0, 2.2f, 1.9f, 0, 2.8f, 1.8f, 0,
                                 3.2f, 1.7f, 0, 3.5f, 1.6f, 0};
    std::string error;
    const auto esdf = BuildLocalEsdfFromDepth(snapshot, options, &error);
    ASSERT_TRUE(esdf != nullptr) << error;
    auto cost_of = [&](const std::vector<float>& raw) {
        const auto controls = ControlPointsFromFlat(raw, 8);
        const math::ClampedCubicBSpline<Eigen::Vector3d> curve(controls);
        const auto dense = curve.Sample(32);
        const auto samples = math::ResamplePolylineArcLength(dense, 32);
        return EvaluateSandPlannerCost(samples, Eigen::Vector3d(3.0, 0.0, 0.0),
                                       options, esdf.get());
    };
    const double through_cost = cost_of(through_wall);
    const double around_cost = cost_of(around);
    EXPECT_LT(around_cost, through_cost) << "through=" << through_cost
                                         << " around=" << around_cost;

    automsgs::msgs::geometry_msgs::Twist twist;
    Eigen::Vector3d heading;
    ASSERT_TRUE(SelectSandPlannerTwist({through_wall, around}, snapshot, options,
                                       &twist, &heading, &error))
        << error;
    EXPECT_GT(std::fabs(heading.y()), 0.2) << "through=" << through_cost
                                           << " around=" << around_cost
                                           << " vx=" << twist.linear().x()
                                           << " vy=" << twist.linear().y();
}

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
