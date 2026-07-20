/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include <algorithm>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "autolink/node/writer.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/control/common/controller_interface.hpp"
#include "autonomy/control/controller/graceful_controller/path_handler.hpp"
#include "autonomy/control/controller/graceful_controller/smooth_control_law.hpp"
#include "autonomy/control/controller/graceful_controller/utils.hpp"
#include "autonomy/control/proto/graceful_controller.pb.h"
#include "autonomy/map/costmap_2d/footprint_collision_checker.hpp"

namespace autonomy {
namespace control {
namespace controller {

struct GracefulRuntimeParams {
    double transform_tolerance{0.1};
    double max_lookahead{0.9};
    double min_lookahead{0.25};
    double max_robot_pose_search_dist{3.0};
    double k_phi{3.0};
    double k_delta{2.0};
    double beta{0.4};
    double lambda{2.0};
    double v_linear_min{0.05};
    double v_linear_max{0.5};
    double v_angular_max{1.0};
    double v_angular_min_in_place{0.1};
    double slowdown_radius{0.5};
    double deceleration_max{2.5};
    double initial_rotation_tolerance{0.1};
    double rotation_scaling_factor{1.0};
    double in_place_collision_resolution{0.1};
    double footprint_scaling_linear_vel{0.5};
    double footprint_scaling_factor{0.25};
    double footprint_scaling_step{0.1};
    double final_rotation_search_step{0.1};
    int32_t obstacle_cost_margin{10};
    bool initial_rotation{true};
    bool prefer_final_rotation{false};
    bool allow_backward{false};
    bool use_collision_detection{true};
};

/**
 * @class nav2_graceful_controller::GracefulController
 * @brief Graceful controller plugin
 */
class GracefulController : public common::ControllerInterface
{
public:
    GracefulController() = default;
    ~GracefulController() override = default;

    void Configure(const proto::ControllerOptions& options, std::string name,
                   std::shared_ptr<transform::Buffer> tf,
                   std::shared_ptr<map::costmap_2d::Costmap2DWrapper>
                       costmap_wrapper);

    void Cleanup();
    void Activate();
    void Deactivate();

    uint32 ComputeVelocityCommands(
        const commsgs::geometry_msgs::PoseStamped& pose,
        const commsgs::geometry_msgs::TwistStamped& velocity,
        commsgs::geometry_msgs::TwistStamped& cmd_vel,
        common::GoalChecker* goal_checker, std::string& message) override;

    bool IsGoalReached(double dist_tolerance, double angle_tolerance) override;
    void SetPlan(const commsgs::planning_msgs::Path& path) override;
    void Reset() override;
    void SetSpeedLimit(const double& speed_limit,
                       const bool& percentage) override;

protected:
    bool ValidateTargetPose(
        commsgs::geometry_msgs::PoseStamped& target_pose, double dist_to_target,
        commsgs::planning_msgs::Path& trajectory,
        commsgs::geometry_msgs::TransformStamped& costmap_transform,
        commsgs::geometry_msgs::TwistStamped& cmd_vel);

    bool ValidateTargetPoseOnApproach(
        commsgs::geometry_msgs::PoseStamped& target_pose, double dist_to_target,
        double dist_to_goal, commsgs::planning_msgs::Path& trajectory,
        commsgs::geometry_msgs::TransformStamped& costmap_transform,
        commsgs::geometry_msgs::TwistStamped& cmd_vel);

    bool FindBestApproachTrajectory(
        commsgs::geometry_msgs::PoseStamped& target_pose, double dist_to_target,
        commsgs::geometry_msgs::TransformStamped& costmap_transform,
        double safety_cost, commsgs::planning_msgs::Path& best_trajectory,
        commsgs::geometry_msgs::TwistStamped& best_cmd_vel);

    bool SimulateTrajectory(
        const commsgs::geometry_msgs::PoseStamped& motion_target,
        const commsgs::geometry_msgs::TransformStamped& costmap_transform,
        commsgs::planning_msgs::Path& trajectory,
        commsgs::geometry_msgs::TwistStamped& cmd_vel, bool backward);

    commsgs::geometry_msgs::Twist RotateToTarget(double angle_to_target);

    bool InCollision(const double& x, const double& y, const double& theta,
                     double inflation_scale = 1.0);

    double GetMaxCost(const commsgs::planning_msgs::Path& path,
                      commsgs::geometry_msgs::TransformStamped&
                          costmap_transform);

    void ComputeDistanceAlongPath(
        const std::vector<commsgs::geometry_msgs::PoseStamped>& poses,
        std::vector<double>& distances);

    void ValidateOrientations(
        std::vector<commsgs::geometry_msgs::PoseStamped>& path);

    std::shared_ptr<transform::Buffer> tf_buffer_;
    std::string plugin_name_;
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper_;
    std::unique_ptr<
        map::costmap_2d::FootprintCollisionChecker<map::costmap_2d::Costmap2D*>>
        collision_checker_;

    bool do_initial_rotation_{true};
    std::optional<double> safe_approach_angle_;

    proto::GracefulControllerOptions options_;
    double initial_v_linear_min_{0.05};
    double initial_v_linear_max_{0.5};
    double initial_v_angular_max_{1.0};
    GracefulRuntimeParams params_;

    std::shared_ptr<autolink::Writer<commsgs::planning_msgs::Path>>
        transformed_plan_pub_;
    std::shared_ptr<autolink::Writer<commsgs::planning_msgs::Path>>
        local_plan_pub_;
    std::shared_ptr<autolink::Writer<commsgs::geometry_msgs::PoseStamped>>
        motion_target_pub_;
    std::shared_ptr<autolink::Writer<commsgs::visualization_msgs::Marker>>
        slowdown_pub_;
    std::unique_ptr<PathHandler> path_handler_;
    std::unique_ptr<SmoothControlLaw> control_law_;
};

}  // namespace controller
}  // namespace control
}  // namespace autonomy
