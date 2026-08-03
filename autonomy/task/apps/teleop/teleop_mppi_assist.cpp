/*
 * Copyright 2026 The Openbot Authors (duyongquan)
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

#include "autonomy/task/apps/teleop/teleop_mppi_assist.hpp"

#include <algorithm>
#include <cmath>

#include "autonomy/common/logging.hpp"
#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/builtin_interfaces/duration.pb.h>
#include <automsgs/msgs/time_utils.hpp>
#include "autonomy/task/apps/teleop/constants.hpp"
#include "autonomy/transform/buffer_utils.hpp"

namespace autonomy::task::teleop {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;

map::proto::Costmap2DOptions DefaultCostmapOptions(
    const std::string& global_frame,
    const PointCloudObstacleFeeder::Options& point_cloud) {
    map::proto::Costmap2DOptions options;
    options.set_enabled(true);
    options.set_frame_id(global_frame);
    options.set_name("teleop_local_costmap");
    options.set_resolution(0.05);
    options.set_width(6);
    options.set_height(6);
    options.set_update_frequency(20.0);
    options.set_rolling_window(true);
    options.set_robot_radius(0.22);

    options.add_plugins("obstacle_layer");
    options.add_plugins("inflation_layer");

    auto* obstacle = options.mutable_obstacle_layer();
    obstacle->set_enabled(true);
    obstacle->set_footprint_clearing_enabled(true);
    auto& sources = *obstacle->mutable_sensor_sources();
    auto& src = sources["teleop_cloud"];
    src.set_topic(point_cloud.cloud_topic);
    src.set_data_type("PointCloud2");
    src.set_marking(true);
    src.set_clearing(false);
    src.set_min_obstacle_height(-2.0);
    src.set_max_obstacle_height(2.0);

    auto* inflation = options.mutable_inflation_layer();
    inflation->set_enabled(true);
    inflation->set_cost_scaling_factor(3.0);
    inflation->set_inflation_radius(0.55);
    return options;
}

control::proto::MPPIControllerOptions DefaultMppiOptions() {
    control::proto::MPPIControllerOptions options;
    options.add_critics("CostCritic");
    options.add_critics("PathFollowCritic");
    options.add_critics("PreferForwardCritic");
    options.set_model_dt(0.05);
    options.set_time_steps(56);
    options.set_batch_size(2000);
    options.set_iteration_count(1);
    options.set_temperature(0.3);
    options.set_gamma(0.015);
    options.set_vx_max(0.5);
    options.set_vx_min(-0.35);
    options.set_vy_max(0.5);
    options.set_wz_max(1.9);
    options.mutable_cost_critic()->set_consider_footprint(false);
    return options;
}

automsgs::msgs::geometry_msgs::PoseStamped IdentityRobotPose(
    const std::string& frame_id) {
    automsgs::msgs::geometry_msgs::PoseStamped pose;
    pose.mutable_header()->set_frame_id(frame_id);
    *pose.mutable_header()->mutable_stamp() = automsgs::msgs::builtin_interfaces::TimeNow();
    pose.mutable_pose()->mutable_orientation()->set_w(1.0);
    return pose;
}

}  // namespace

map::proto::Costmap2DOptions TeleopMppiAssist::DefaultCostmapOptions(
    const std::string& global_frame,
    const PointCloudObstacleFeeder::Options& point_cloud) {
    return ::autonomy::task::teleop::DefaultCostmapOptions(global_frame,
                                                          point_cloud);
}

control::proto::MPPIControllerOptions TeleopMppiAssist::DefaultMppiOptions() {
    return ::autonomy::task::teleop::DefaultMppiOptions();
}

bool TeleopMppiAssist::Configure(std::shared_ptr<autolink::Node> node,
                                 std::shared_ptr<transform::Buffer> tf,
                                 const Options& options) {
    configured_ = false;
    node_ = std::move(node);
    tf_buffer_ = std::move(tf);
    options_ = options;

    if (!options_.enabled) {
        configured_ = true;
        return true;
    }
    if (!node_) {
        AERROR << "TeleopMppiAssist: node is null";
        return false;
    }

    if (tf_buffer_) {
        transform::SeedBenchmarkTfTree(tf_buffer_.get(), "teleop_mppi_assist");
    }

    map::proto::Costmap2DOptions costmap_opts = options_.costmap;
    if (costmap_opts.plugins_size() == 0 && !costmap_opts.has_obstacle_layer()) {
        costmap_opts = DefaultCostmapOptions(options_.global_frame,
                                             options_.point_cloud);
    }
    if (costmap_opts.frame_id().empty()) {
        costmap_opts.set_frame_id(options_.global_frame);
    }

    costmap_ = std::make_shared<map::costmap_2d::Costmap2DWrapper>(
        costmap_opts, "teleop_local_costmap");
    if (costmap_opts.enabled()) {
        costmap_->Start();
    }

    feeder_.Configure(node_, costmap_, options_.point_cloud);
    feeder_.Start();

    const auto& lib = options_.path_library;
    selector_.GenerateDefaultLibrary(lib.num_dirs, lib.num_lengths,
                                     lib.max_range, lib.ds);

    control::proto::ControllerOptions controller_options;
    if (options_.mppi.critics_size() > 0) {
        *controller_options.mutable_mppi_controller_options() = options_.mppi;
    } else {
        *controller_options.mutable_mppi_controller_options() =
            DefaultMppiOptions();
    }

    mppi_ = std::make_unique<
        control::controller::mppi_controller::MPPIController>();
    mppi_->Configure(controller_options, "teleop_mppi", tf_buffer_, costmap_);
    mppi_->Activate();

    goal_checker_.Initialize("teleop_goal_checker", costmap_);
    goal_checker_.SetTolerances(100.0, 100.0, false);

    configured_ = true;
    AINFO << "TeleopMppiAssist configured (frame=" << options_.global_frame
          << ")";
    return true;
}

TeleopMppiAssist::~TeleopMppiAssist() {
    Shutdown();
}

void TeleopMppiAssist::Shutdown() {
    feeder_.Stop();
    if (mppi_) {
        mppi_->Deactivate();
        mppi_.reset();
    }
    if (costmap_) {
        costmap_->Stop();
        costmap_.reset();
    }
    configured_ = false;
}

bool TeleopMppiAssist::IsPerceptionOk() const {
    return feeder_.IsCloudFresh();
}

bool TeleopMppiAssist::TryGetRobotPose(
    automsgs::msgs::geometry_msgs::PoseStamped* pose) const {
    if (!pose || !costmap_) {
        return false;
    }
    return costmap_->getRobotPose(*pose);
}

void TeleopMppiAssist::FillPassthroughCmd(
    double linear_x, double angular_z,
    automsgs::msgs::geometry_msgs::TwistStamped* cmd_out) const {
    cmd_out->mutable_header()->set_frame_id(kDefaultBaseFrame);
    *cmd_out->mutable_header()->mutable_stamp() = automsgs::msgs::builtin_interfaces::TimeNow();
    cmd_out->mutable_twist()->mutable_linear()->set_x(linear_x);
    cmd_out->mutable_twist()->mutable_angular()->set_z(angular_z);
}

double TeleopMppiAssist::AngularToJoyDirDeg(double angular_z,
                                             double linear_x) const {
    constexpr double kLinearEps = 1e-3;
    if (std::abs(linear_x) < kLinearEps) {
        return std::clamp(angular_z * options_.angular_to_dir_gain * kRadToDeg,
                          -90.0, 90.0);
    }
    return std::atan2(angular_z * options_.angular_to_dir_gain, linear_x) *
           kRadToDeg;
}

bool TeleopMppiAssist::Tick(double joy_linear_x, double joy_angular_z,
                            const automsgs::msgs::geometry_msgs::PoseStamped& robot_pose,
                            const automsgs::msgs::geometry_msgs::Twist& robot_speed,
                            automsgs::msgs::geometry_msgs::TwistStamped* cmd_out) {
    if (!cmd_out) {
        return false;
    }
    if (!configured_) {
        return false;
    }
    if (!options_.enabled) {
        FillPassthroughCmd(joy_linear_x, joy_angular_z, cmd_out);
        return true;
    }
    if (!IsPerceptionOk()) {
        FillPassthroughCmd(0.0, 0.0, cmd_out);
        return false;
    }
    if (!mppi_ || !costmap_) {
        FillPassthroughCmd(0.0, 0.0, cmd_out);
        return false;
    }

    costmap_->updateMap();

    if (std::abs(joy_linear_x) < options_.stopped_linear_epsilon) {
        FillPassthroughCmd(0.0,
                           joy_angular_z * options_.in_place_angular_scale,
                           cmd_out);
        return true;
    }

    const double joy_dir_deg =
        AngularToJoyDirDeg(joy_angular_z, joy_linear_x);
    const auto& costmap = *costmap_->getCostmap();
    auto path = selector_.Select(costmap, joy_dir_deg, joy_linear_x);
    if (path.has_value()) {
        path->mutable_header()->set_frame_id(options_.global_frame);
        for (auto& pose_stamped : *path->mutable_poses()) {
            pose_stamped.mutable_header()->set_frame_id(options_.global_frame);
        }
    }

    if (!path.has_value()) {
        FillPassthroughCmd(0.0,
                           joy_angular_z * options_.in_place_angular_scale,
                           cmd_out);
        return true;
    }

    mppi_->SetPlan(*path);

    automsgs::msgs::geometry_msgs::PoseStamped pose = robot_pose;
    if (pose.header().frame_id().empty()) {
        pose = IdentityRobotPose(options_.global_frame);
    }

    automsgs::msgs::geometry_msgs::TwistStamped velocity;
    *velocity.mutable_twist() = robot_speed;

    std::string message;
    automsgs::msgs::geometry_msgs::TwistStamped cmd;
    const uint32_t result = mppi_->ComputeVelocityCommands(
        pose, velocity, cmd, &goal_checker_, message);
    if (result != 0) {
        AWARN << "TeleopMppiAssist: MPPI failed: " << message;
        FillPassthroughCmd(0.0, 0.0, cmd_out);
        return false;
    }

    *cmd_out = cmd;
    if (cmd_out->header().frame_id().empty()) {
        cmd_out->mutable_header()->set_frame_id(kDefaultBaseFrame);
    }
    return true;
}

}  // namespace autonomy::task::teleop
