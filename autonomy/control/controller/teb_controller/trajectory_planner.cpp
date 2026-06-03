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

#include <cmath>
#include <limits>
#include <map>
#include <memory>

#include "autonomy/common/logging.hpp"
#include "autonomy/control/controller/teb_controller/cost_functions/optimization_problem.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/control/controller/teb_controller/trajectory_planner.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/costmap_2d.hpp"
#include "autonomy/map/costmap_2d/footprint_collision_checker.hpp"

namespace autonomy {
namespace control {
namespace teb_controller {

namespace {

commsgs::builtin_interfaces::Duration DurationFromSec(double t_sec) {
    commsgs::builtin_interfaces::Duration out;
    int32_t sec = static_cast<int32_t>(std::floor(t_sec));
    uint32_t nsec = static_cast<uint32_t>(std::round((t_sec - sec) * 1e9));
    sec += static_cast<int32_t>(nsec / 1000000000u);
    nsec %= 1000000000u;
    out.sec = sec;
    out.nanosec = nsec;
    return out;
}

}  // namespace

TrajectoryPlanner::TrajectoryPlanner()
    : config_(nullptr),
      obstacles_(nullptr),
      via_points_(nullptr),
      cost_(HUGE_VAL),
      prefer_rotdir_(PreferredRotationDirection::none),
      initialized_(false),
      optimized_(false) {}

TrajectoryPlanner::TrajectoryPlanner(const TimedElasticBandConfig& cfg,
                                     ObstContainer* obstacles,
                                     PlannerVisualizationPtr visual,
                                     const ViaPointContainer* via_points) {
    Initialize(cfg, obstacles, visual, via_points);
}

TrajectoryPlanner::~TrajectoryPlanner() {
    ResetOptimization();
}

uint32 TrajectoryPlanner::ComputeVelocityCommands(
    const commsgs::geometry_msgs::PoseStamped& /*pose*/,
    const commsgs::geometry_msgs::TwistStamped& velocity,
    commsgs::geometry_msgs::TwistStamped& cmd_vel,
    common::GoalChecker* /*goal_checker*/, std::string& message) {
    if (current_plan_.poses.empty()) {
        message = "No plan set";
        return 100U;
    }

    const bool free_goal_velocity =
        config_ ? config_->goal_tolerance.free_goal_velocity : false;
    if (!Plan(current_plan_.poses, &velocity.twist, free_goal_velocity)) {
        message = "TEB planning failed";
        return 100U;
    }

    double vx = 0.0;
    double vy = 0.0;
    double omega = 0.0;
    const int look_ahead = config_ ? config_->trajectory.control_lookahead_pose_count : 1;
    if (!GetVelocityCommand(vx, vy, omega, look_ahead)) {
        message = "Velocity command unavailable";
        return 100U;
    }

    cmd_vel.header = velocity.header;
    cmd_vel.twist.linear.x = vx;
    cmd_vel.twist.linear.y = vy;
    cmd_vel.twist.linear.z = 0.0;
    cmd_vel.twist.angular.x = 0.0;
    cmd_vel.twist.angular.y = 0.0;
    cmd_vel.twist.angular.z = omega;
    message.clear();
    return 0U;
}

bool TrajectoryPlanner::IsGoalReached(double dist_tolerance,
                                      double angle_tolerance) {
    if (current_plan_.poses.empty() || teb_.SizePoses() == 0) {
        return false;
    }

    const Pose2D& current_pose = teb_.BackPose();
    const Pose2D goal_pose = Pose2DFromPose(current_plan_.poses.back().pose);
    const double pos_error =
        Norm(Position(current_pose) - Position(goal_pose));
    const double yaw_error = std::fabs(
        autonomy::common::AngleDiff(current_pose.theta, goal_pose.theta));
    return pos_error <= dist_tolerance && yaw_error <= angle_tolerance;
}

void TrajectoryPlanner::SetPlan(const commsgs::planning_msgs::Path& plan) {
    current_plan_ = plan;
}

void TrajectoryPlanner::SetSpeedLimit(const double& speed_limit,
                                      const bool& percentage) {
    if (config_ == nullptr) {
        return;
    }
    TimedElasticBandConfig* cfg = const_cast<TimedElasticBandConfig*>(config_);
    if (percentage) {
        const double factor = std::max(0.0, speed_limit) / 100.0;
        cfg->robot.max_velocity_x = cfg->robot.base_max_velocity_x * factor;
        cfg->robot.max_velocity_x_backwards =
            cfg->robot.base_max_velocity_x_backwards * factor;
        cfg->robot.max_velocity_y = cfg->robot.base_max_velocity_y * factor;
        cfg->robot.max_angular_velocity = cfg->robot.base_max_angular_velocity * factor;
        return;
    }
    cfg->robot.max_velocity_x = std::max(0.0, speed_limit);
}

void TrajectoryPlanner::Initialize(const TimedElasticBandConfig& cfg,
                                   ObstContainer* obstacles,
                                   PlannerVisualizationPtr visual,
                                   const ViaPointContainer* via_points) {
    config_ = &cfg;
    obstacles_ = obstacles;
    via_points_ = via_points;
    cost_ = HUGE_VAL;
    prefer_rotdir_ = PreferredRotationDirection::none;

    vel_start_.first = true;
    vel_start_.second.linear.x = 0;
    vel_start_.second.linear.y = 0;
    vel_start_.second.angular.z = 0;

    vel_goal_.first = true;
    vel_goal_.second.linear.x = 0;
    vel_goal_.second.linear.y = 0;
    vel_goal_.second.angular.z = 0;
    initialized_ = true;

    SetVisualization(visual);
}

void TrajectoryPlanner::SetVisualization(
    const PlannerVisualizationPtr& visualization) {
    visualization_ = visualization;
}

void TrajectoryPlanner::Visualize() {
    if (!visualization_)
        return;

    visualization_->PublishLocalPlanAndPoses(teb_);

    if (teb_.SizePoses() > 0)
        visualization_->PublishRobotFootprint(teb_.Pose(0),
                                                   *config_->robot_footprint);

    if (config_->trajectory.publish_feedback)
        visualization_->PublishFeedbackMessage(*this, *obstacles_);
}

bool TrajectoryPlanner::OptimizeTimedElasticBand(int iterations_innerloop,
                                    int iterations_outerloop,
                                    bool compute_cost_afterwards,
                                    double obst_cost_Scale,
                                    double viapoint_cost_Scale,
                                    bool alternative_time_cost) {
    if (config_->optimization.optimization_activate == false)
        return false;

    bool success = false;
    optimized_ = false;

    double weight_multiplier = 1.0;

    // TODO(roesmann): we introduced the non-fast mode with the support of
    // dynamic obstacles
    //                (which leads to better results in terms of x-y-t homotopy
    //                planning).
    //                 however, we have not tested this mode intensively yet, so
    //                 we keep the legacy fast mode as default until we finish
    //                 our tests.
    bool fast_mode = !config_->obstacles.include_dynamic_obstacles;

    for (int i = 0; i < iterations_outerloop; ++i) {
        if (config_->trajectory.enable_auto_resize) {
            // teb_.AutoResize(config_->trajectory.reference_time_step,
            // config_->trajectory.time_step_hysteresis, config_->trajectory.min_samples,
            // config_->trajectory.max_samples);
            teb_.AutoResize(config_->trajectory.reference_time_step,
                            config_->trajectory.time_step_hysteresis,
                            config_->trajectory.min_samples,
                            config_->trajectory.max_samples, fast_mode);
        }

        success = BuildOptimizationProblem(weight_multiplier);
        if (!success) {
            ResetOptimization();
            return false;
        }
        success = SolveOptimization(iterations_innerloop, false);
        if (!success) {
            ResetOptimization();
            return false;
        }
        optimized_ = true;

        if (compute_cost_afterwards &&
            i == iterations_outerloop -
                     1)  // compute cost vec only in the last iteration
            ComputeCurrentCost(obst_cost_Scale, viapoint_cost_Scale,
                               alternative_time_cost);

        ResetOptimization();

        weight_multiplier *= config_->optimization.weight_adapt_factor;
    }

    return true;
}

void TrajectoryPlanner::SetVelocityStart(
    const autonomy::commsgs::geometry_msgs::Twist& vel_start) {
    vel_start_.first = true;
    vel_start_.second.linear.x = vel_start.linear.x;
    vel_start_.second.linear.y = vel_start.linear.y;
    vel_start_.second.angular.z = vel_start.angular.z;
}

void TrajectoryPlanner::SetVelocityGoal(
    const autonomy::commsgs::geometry_msgs::Twist& vel_goal) {
    vel_goal_.first = true;
    vel_goal_.second = vel_goal;
}

bool TrajectoryPlanner::Plan(
    const std::vector<autonomy::commsgs::geometry_msgs::PoseStamped>&
        initial_plan,
    const autonomy::commsgs::geometry_msgs::Twist* start_vel,
    bool free_goal_velocity) {
    TEB_ASSERT_MSG(initialized_, "Call Initialize() first.");
    if (!teb_.IsInit()) {
        teb_.InitTrajectoryToGoal(
            initial_plan, config_->robot.max_velocity_x, config_->robot.max_angular_velocity,
            config_->trajectory.global_plan_overwrite_orientation,
            config_->trajectory.min_samples,
            config_->trajectory.allow_init_with_backwards_motion);
    } else  // warm start
    {
        Pose2D start_ = Pose2DFromPose(initial_plan.front().pose);
        Pose2D goal_ = Pose2DFromPose(initial_plan.back().pose);
        if (teb_.SizePoses() > 0 &&
            Norm(Position(goal_) - Position(teb_.BackPose())) <
                config_->trajectory.force_reinit_new_goal_dist &&
            fabs(autonomy::common::AngleDiff(goal_.theta,
                                            teb_.BackPose().theta)) <
                config_->trajectory
                    .force_reinit_new_goal_angular)  // actual warm start!
            teb_.UpdateAndPruneTEB(start_, goal_,
                                   config_->trajectory.min_samples);  // update TEB
        else  // goal too far away -> reinit
        {
            ADEBUG << "New goal: distance to existing goal is higher than the "
                      "specified threshold. Reinitalizing trajectories.";
            teb_.ClearTimedElasticBand();
            teb_.InitTrajectoryToGoal(
                initial_plan, config_->robot.max_velocity_x, config_->robot.max_angular_velocity,
                config_->trajectory.global_plan_overwrite_orientation,
                config_->trajectory.min_samples,
                config_->trajectory.allow_init_with_backwards_motion);
        }
    }
    if (start_vel)
        SetVelocityStart(*start_vel);
    if (free_goal_velocity)
        SetVelocityGoalFree();
    else
        vel_goal_.first =
            true;  // we just reactivate and use the previously set velocity
                   // (should be zero if nothing was modified)

    // now optimize
    return OptimizeTimedElasticBand(config_->optimization.inner_iteration_count,
                       config_->optimization.outer_iteration_count);
}

// bool TrajectoryPlanner::Plan(const tf::Pose& start, const tf::Pose& goal,
// const autonomy::commsgs::geometry_msgs::Twist* start_vel, bool free_goal_velocity)
//{
//   Pose2D start_(start);
//   Pose2D goal_(goal);
//   return Plan(start_, goal_, start_vel);
// }

bool TrajectoryPlanner::Plan(
    const Pose2D& start, const Pose2D& goal,
    const autonomy::commsgs::geometry_msgs::Twist* start_vel,
    bool free_goal_velocity) {
    TEB_ASSERT_MSG(initialized_, "Call Initialize() first.");
    if (!teb_.IsInit()) {
        // init trajectory
        teb_.InitTrajectoryToGoal(
            start, goal, 0, config_->robot.max_velocity_x, config_->trajectory.min_samples,
            config_->trajectory
                .allow_init_with_backwards_motion);  // 0 intermediate samples,
                                                     // but dt=1 -> AutoResize
                                                     // will add more samples
                                                     // before calling first
                                                     // optimization
    } else                                           // warm start
    {
        if (teb_.SizePoses() > 0 &&
            Norm(Position(goal) - Position(teb_.BackPose())) <
                config_->trajectory.force_reinit_new_goal_dist &&
            fabs(autonomy::common::AngleDiff(goal.theta,
                                            teb_.BackPose().theta)) <
                config_->trajectory
                    .force_reinit_new_goal_angular)  // actual warm start!
            teb_.UpdateAndPruneTEB(start, goal, config_->trajectory.min_samples);
        else  // goal too far away -> reinit
        {
            ADEBUG << "New goal: distance to existing goal is higher than the "
                      "specified threshold. Reinitalizing trajectories.";
            teb_.ClearTimedElasticBand();
            teb_.InitTrajectoryToGoal(
                start, goal, 0, config_->robot.max_velocity_x,
                config_->trajectory.min_samples,
                config_->trajectory.allow_init_with_backwards_motion);
        }
    }
    if (start_vel)
        SetVelocityStart(*start_vel);
    if (free_goal_velocity)
        SetVelocityGoalFree();
    else
        vel_goal_.first =
            true;  // we just reactivate and use the previously set velocity
                   // (should be zero if nothing was modified)

    // now optimize
    return OptimizeTimedElasticBand(config_->optimization.inner_iteration_count,
                       config_->optimization.outer_iteration_count, true);
}

bool TrajectoryPlanner::BuildOptimizationProblem(double weight_multiplier) {
    if (optimization_problem_.Problem().NumResidualBlocks() > 0) {
        AWARN << "Cannot build graph, because it is not empty. Call graphClear()!";
        return false;
    }

    RegisterTrajectoryParameters();

    optimization_problem_.SetBuildContext(
        *config_, teb_, obstacles_, via_points_, config_->robot_footprint.get(),
        obstacles_per_vertex_, prefer_rotdir_, vel_start_, vel_goal_);
    return optimization_problem_.Build(weight_multiplier);
}

bool TrajectoryPlanner::SolveOptimization(int no_iterations, bool clear_after) {
    if (config_->robot.max_velocity_x < 0.01) {
        AWARN << "SolveOptimization(): Robot Max Velocity is smaller than 0.01m/s. "
                 "Optimizing aborted...";
        if (clear_after)
            ResetOptimization();
        return false;
    }

    if (!teb_.IsInit() || teb_.SizePoses() < config_->trajectory.min_samples) {
        AWARN << "SolveOptimization(): TEB is empty or has too less elements. "
                 "Skipping optimization.";
        if (clear_after)
            ResetOptimization();
        return false;
    }

    if (!optimization_problem_.Solve(no_iterations,
                                config_->optimization.optimization_verbose)) {
        AERROR << "SolveOptimization(): Ceres optimization failed.";
        return false;
    }

    if (clear_after)
        ResetOptimization();

    return true;
}

void TrajectoryPlanner::ResetOptimization() {
    optimization_problem_.Clear();
}

void TrajectoryPlanner::RegisterTrajectoryParameters() {
    if (config_->optimization.optimization_verbose) {
        ADEBUG << "Preparing TEB Ceres parameter blocks ...";
    }
    obstacles_per_vertex_.resize(teb_.SizePoses());
    if (obstacles_ != nullptr) {
        for (auto& per_vertex : obstacles_per_vertex_) {
            per_vertex.clear();
            per_vertex.reserve(obstacles_->size());
        }
    }
}


bool TrajectoryPlanner::HasDiverged() const {
    return optimization_problem_.HasDiverged(*config_);
}

void TrajectoryPlanner::ComputeCurrentCost(double obst_cost_Scale,
                                           double viapoint_cost_Scale,
                                           bool alternative_time_cost) {
    bool graph_exist_flag = false;
    if (optimization_problem_.Problem().NumResidualBlocks() == 0) {
        BuildOptimizationProblem();
    } else {
        graph_exist_flag = true;
    }

    cost_ = optimization_problem_.EvaluateTotalCost(
        obst_cost_Scale, viapoint_cost_Scale, alternative_time_cost);

    if (alternative_time_cost) {
        cost_ += teb_.GetSumOfAllTimeDiffs();
    }

    if (!graph_exist_flag) {
        ResetOptimization();
    }
}

void TrajectoryPlanner::ExtractVelocity(const Pose2D& pose1,
                                        const Pose2D& pose2, double dt,
                                        double& vx, double& vy,
                                        double& omega) const {
    if (dt == 0) {
        vx = 0;
        vy = 0;
        omega = 0;
        return;
    }

    const Point deltaS = Position(pose2) - Position(pose1);

    if (config_->robot.max_velocity_y == 0)  // nonholonomic robot
    {
        const Point conf1dir(std::cos(pose1.theta), std::sin(pose1.theta));
        // translational velocity
        double dir = Dot(deltaS, conf1dir);
        vx = (double)Sign(dir) * Norm(deltaS) / dt;
        vy = 0;
    } else  // holonomic robot
    {
        // transform pose 2 into the current robot frame (pose1)
        // for velocities only the rotation of the direction vector is
        // necessary. (map->pose1-frame: inverse 2d rotation matrix)
        double cos_theta1 = std::cos(pose1.theta);
        double sin_theta1 = std::sin(pose1.theta);
        double p1_dx = cos_theta1 * deltaS.x + sin_theta1 * deltaS.y;
        double p1_dy = -sin_theta1 * deltaS.x + cos_theta1 * deltaS.y;
        vx = p1_dx / dt;
        vy = p1_dy / dt;
    }

    // rotational velocity
    double orientdiff = autonomy::common::AngleDiff(pose1.theta, pose2.theta);
    omega = orientdiff / dt;
}

bool TrajectoryPlanner::GetVelocityCommand(double& vx, double& vy,
                                           double& omega,
                                           int look_ahead_poses) const {
    if (teb_.SizePoses() < 2) {
        AERROR << "TrajectoryPlanner::GetVelocityCommand(): The trajectory "
                  "contains less than 2 Poses. Make sure to init and "
                  "optimize/Plan the trajectory fist.";
        vx = 0;
        vy = 0;
        omega = 0;
        return false;
    }
    look_ahead_poses =
        std::max(1, std::min(look_ahead_poses, teb_.SizePoses() - 1));
    double dt = 0.0;
    for (int counter = 0; counter < look_ahead_poses; ++counter) {
        dt += teb_.TimeDiff(counter);
        if (dt >= config_->trajectory.reference_time_step *
                      look_ahead_poses)  // TODO: change to look-ahead time?
                                         // Refine trajectory?
        {
            look_ahead_poses = counter + 1;
            break;
        }
    }
    if (dt <= 0) {
        AERROR << "TrajectoryPlanner::GetVelocityCommand() - timediff<=0 is "
                  "invalid!";
        vx = 0;
        vy = 0;
        omega = 0;
        return false;
    }

    // Get velocity from the first two configurations
    ExtractVelocity(teb_.Pose(0), teb_.Pose(look_ahead_poses), dt, vx, vy,
                    omega);
    return true;
}

void TrajectoryPlanner::GetVelocityProfile(
    std::vector<autonomy::commsgs::geometry_msgs::Twist>& velocity_profile)
    const {
    int n = teb_.SizePoses();
    velocity_profile.resize(n + 1);

    // start velocity
    velocity_profile.front().linear.z = 0;
    velocity_profile.front().angular.x = velocity_profile.front().angular.y = 0;
    velocity_profile.front().linear.x = vel_start_.second.linear.x;
    velocity_profile.front().linear.y = vel_start_.second.linear.y;
    velocity_profile.front().angular.z = vel_start_.second.angular.z;

    for (int i = 1; i < n; ++i) {
        velocity_profile[i].linear.z = 0;
        velocity_profile[i].angular.x = velocity_profile[i].angular.y = 0;
        ExtractVelocity(teb_.Pose(i - 1), teb_.Pose(i), teb_.TimeDiff(i - 1),
                        velocity_profile[i].linear.x,
                        velocity_profile[i].linear.y,
                        velocity_profile[i].angular.z);
    }

    // goal velocity
    velocity_profile.back().linear.z = 0;
    velocity_profile.back().angular.x = velocity_profile.back().angular.y = 0;
    velocity_profile.back().linear.x = vel_goal_.second.linear.x;
    velocity_profile.back().linear.y = vel_goal_.second.linear.y;
    velocity_profile.back().angular.z = vel_goal_.second.angular.z;
}

void TrajectoryPlanner::GetFullTrajectory(
    std::vector<TrajectoryPoint>& trajectory) const {
    int n = teb_.SizePoses();

    trajectory.resize(n);

    if (n == 0)
        return;

    double curr_time = 0;

    // start
    TrajectoryPoint& start = trajectory.front();
    ToPose3D(teb_.Pose(0), start.pose);
    start.velocity.linear.z = 0;
    start.velocity.angular.x = start.velocity.angular.y = 0;
    start.velocity.linear.x = vel_start_.second.linear.x;
    start.velocity.linear.y = vel_start_.second.linear.y;
    start.velocity.angular.z = vel_start_.second.angular.z;
    start.time_from_start = DurationFromSec(curr_time);

    curr_time += teb_.TimeDiff(0);

    // intermediate points
    for (int i = 1; i < n - 1; ++i) {
        TrajectoryPoint& point = trajectory[i];
        ToPose3D(teb_.Pose(i), point.pose);
        point.velocity.linear.z = 0;
        point.velocity.angular.x = point.velocity.angular.y = 0;
        double vel1_x, vel1_y, vel2_x, vel2_y, omega1, omega2;
        ExtractVelocity(teb_.Pose(i - 1), teb_.Pose(i), teb_.TimeDiff(i - 1),
                        vel1_x, vel1_y, omega1);
        ExtractVelocity(teb_.Pose(i), teb_.Pose(i + 1), teb_.TimeDiff(i),
                        vel2_x, vel2_y, omega2);
        point.velocity.linear.x = 0.5 * (vel1_x + vel2_x);
        point.velocity.linear.y = 0.5 * (vel1_y + vel2_y);
        point.velocity.angular.z = 0.5 * (omega1 + omega2);
        point.time_from_start = DurationFromSec(curr_time);

        curr_time += teb_.TimeDiff(i);
    }

    // goal
    TrajectoryPoint& goal = trajectory.back();
    ToPose3D(teb_.BackPose(), goal.pose);
    goal.velocity.linear.z = 0;
    goal.velocity.angular.x = goal.velocity.angular.y = 0;
    goal.velocity.linear.x = vel_goal_.second.linear.x;
    goal.velocity.linear.y = vel_goal_.second.linear.y;
    goal.velocity.angular.z = vel_goal_.second.angular.z;
    goal.time_from_start = DurationFromSec(curr_time);
}

bool TrajectoryPlanner::IsTrajectoryFeasible(
    map::costmap_2d::Costmap2D* costmap,
    const std::vector<autonomy::commsgs::geometry_msgs::Point>& footprint_spec,
    double inscribed_radius, double circumscribed_radius, int look_ahead_idx,
    double feasibility_check_lookahead_distance) {
    if (look_ahead_idx < 0 || look_ahead_idx >= teb().SizePoses())
        look_ahead_idx = teb().SizePoses() - 1;

    if (feasibility_check_lookahead_distance > 0) {
        for (int i = 1; i < teb().SizePoses(); ++i) {
            double pose_distance =
                std::hypot(teb().Pose(i).x - teb().Pose(0).x,
                           teb().Pose(i).y - teb().Pose(0).y);
            if (pose_distance > feasibility_check_lookahead_distance) {
                look_ahead_idx = i - 1;
                break;
            }
        }
    }

    autonomy::commsgs::geometry_msgs::Pose2D pose2d;
    for (int i = 0; i <= look_ahead_idx; ++i) {
        pose2d = teb().Pose(i);
        if (!IsPoseValid(pose2d, costmap, footprint_spec)) {
            if (visualization_) {
                visualization_->PublishInfeasibleRobotPose(teb().Pose(i),
                                                           *config_->robot_footprint);
            }
            return false;
        }
        // Checks if the distance between two Poses is higher than the robot
        // radius or the orientation diff is bigger than the specified threshold
        // and interpolates in that case.
        // (if obstacles are pushing two consecutive Poses away, the center
        // between two consecutive Poses might coincide with the obstacle ;-)!
        if (i < look_ahead_idx) {
            double delta_rot = autonomy::common::NormalizeAngle(
                autonomy::common::NormalizeAngle(teb().Pose(i + 1).theta) -
                autonomy::common::NormalizeAngle(teb().Pose(i).theta));
            const Point delta_dist =
                Position(teb().Pose(i + 1)) - Position(teb().Pose(i));
            if (fabs(delta_rot) >
                    config_->trajectory.min_resolution_collision_check_angular ||
                Norm(delta_dist) > inscribed_radius) {
                int n_additional_samples =
                    std::max(
                        std::ceil(fabs(delta_rot) /
                                  config_->trajectory
                                      .min_resolution_collision_check_angular),
                        std::ceil(Norm(delta_dist) / inscribed_radius)) -
                    1;
                Pose2D intermediate_pose = teb().Pose(i);
                for (int step = 0; step < n_additional_samples; ++step) {
                    intermediate_pose.x +=
                        delta_dist.x / (n_additional_samples + 1.0);
                    intermediate_pose.y +=
                        delta_dist.y / (n_additional_samples + 1.0);
                    intermediate_pose.theta =
                        autonomy::common::NormalizeAngle(
                            intermediate_pose.theta +
                            delta_rot / (n_additional_samples + 1.0));
                    pose2d = intermediate_pose;

                    if (!IsPoseValid(pose2d, costmap, footprint_spec)) {
                        if (visualization_) {
                            visualization_->PublishInfeasibleRobotPose(
                                intermediate_pose, *config_->robot_footprint);
                        }
                        return false;
                    }
                }
            }
        }
    }
    return true;
}

bool TrajectoryPlanner::IsPoseValid(
    autonomy::commsgs::geometry_msgs::Pose2D pose2d,
    map::costmap_2d::Costmap2D* costmap,
    const std::vector<autonomy::commsgs::geometry_msgs::Point>&
        footprint_spec) {
    if (costmap == nullptr) {
        return true;
    }
    try {
        map::costmap_2d::FootprintCollisionChecker<map::costmap_2d::Costmap2D*>
            checker(costmap);
        const double cost = checker.footprintCostAtPose(
            pose2d.x, pose2d.y, pose2d.theta, footprint_spec);
        if (cost < 0.0 ||
            cost >= static_cast<double>(map::costmap_2d::LETHAL_OBSTACLE)) {
            return false;
        }
    } catch (...) {
        return false;
    }
    return true;
}

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
