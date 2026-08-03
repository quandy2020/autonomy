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

#pragma once

#include <memory>
#include <string>

#include "autonomy/control/checker/simple_goal_checker.hpp"
#include "autonomy/control/controller/mppi_controller/controller.hpp"
#include "autonomy/control/proto/controller_options.pb.h"
#include "autonomy/control/proto/mppi_controller.pb.h"
#include "autonomy/map/proto/map_2d_option.pb.h"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/task/apps/teleop/intent_path_selector.hpp"
#include "autonomy/task/apps/teleop/point_cloud_obstacle_feeder.hpp"
#include "autonomy/transform/buffer.hpp"

namespace autonomy::task::teleop {

/** Joystick intent → path select → MPPI → safe cmd_vel. */
class TeleopMppiAssist
{
public:
    struct PathLibraryOptions {
        int num_dirs{9};
        int num_lengths{3};
        double max_range{3.0};
        double ds{0.1};
    };

    struct Options {
        bool enabled{true};
        std::string global_frame{"base_link"};
        double angular_to_dir_gain{1.0};
        double stopped_linear_epsilon{0.02};
        double in_place_angular_scale{0.5};
        PathLibraryOptions path_library;
        PointCloudObstacleFeeder::Options point_cloud;
        map::proto::Costmap2DOptions costmap;
        control::proto::MPPIControllerOptions mppi;
    };

    bool Configure(std::shared_ptr<autolink::Node> node,
                   std::shared_ptr<transform::Buffer> tf, const Options& options);

    ~TeleopMppiAssist();

    void Shutdown();

    bool Tick(double joy_linear_x, double joy_angular_z,
              const automsgs::msgs::geometry_msgs::PoseStamped& robot_pose,
              const automsgs::msgs::geometry_msgs::Twist& robot_speed,
              automsgs::msgs::geometry_msgs::TwistStamped* cmd_out);

    bool enabled() const { return options_.enabled; }
    bool IsPerceptionOk() const;
    bool TryGetRobotPose(automsgs::msgs::geometry_msgs::PoseStamped* pose) const;

private:
    static map::proto::Costmap2DOptions DefaultCostmapOptions(
        const std::string& global_frame,
        const PointCloudObstacleFeeder::Options& point_cloud);
    static control::proto::MPPIControllerOptions DefaultMppiOptions();

    double AngularToJoyDirDeg(double angular_z, double linear_x) const;
    void FillPassthroughCmd(double linear_x, double angular_z,
                            automsgs::msgs::geometry_msgs::TwistStamped* cmd_out) const;

    Options options_;
    std::shared_ptr<autolink::Node> node_;
    std::shared_ptr<transform::Buffer> tf_buffer_;
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_;
    PointCloudObstacleFeeder feeder_;
    IntentPathSelector selector_;
    std::unique_ptr<control::controller::mppi_controller::MPPIController> mppi_;
    control::checker::SimpleGoalChecker goal_checker_;
    bool configured_{false};
};

}  // namespace autonomy::task::teleop
