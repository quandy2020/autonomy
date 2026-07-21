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

#include "autonomy/control/controller/teb_controller/optimizer.hpp"

#include "autolink/common/log.hpp"
#include "autonomy/control/common/controller_exceptions.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace teb_controller {

namespace {

double getYaw(const commsgs::geometry_msgs::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

constexpr double kDefaultMaxVelX = 0.5;
constexpr double kDefaultMaxVelTheta = 1.0;
constexpr double kDefaultAccLimX = 0.5;
constexpr double kDefaultAccLimTheta = 0.5;

}  // namespace

void Optimizer::initialize(
    const std::string& name,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap,
    const proto::TEBControllerOptions* options, double controller_frequency) {
  name_ = name;
  costmap_wrapper_ = std::move(costmap);
  options_ = options;
  teb_config_ = teb_local_planner::TebConfig();
  applyOptionsToConfig();

  if (controller_frequency > 0.0 && teb_config_.trajectory.dt_ref <= 0.0) {
    teb_config_.trajectory.dt_ref = 1.0 / controller_frequency;
  }

  obstacle_converter_ =
      std::make_unique<tools::CostmapObstacleConverter>(*options_);
  planner_ = std::make_unique<teb_local_planner::TebOptimalPlanner>(
      teb_config_, &obstacles_, teb_local_planner::TebVisualizationPtr(),
      &via_points_);
  has_plan_ = false;
}

void Optimizer::applyOptionsToConfig() {
  if (!options_) {
    return;
  }

  auto& robot = teb_config_.robot;
  robot.max_vel_x = options_->max_vel_x() > 0.0 ? options_->max_vel_x()
                                                : kDefaultMaxVelX;
  robot.max_vel_x_backwards =
      options_->max_vel_x_backwards() > 0.0 ? options_->max_vel_x_backwards()
                                              : 0.2;
  robot.max_vel_theta = options_->max_vel_theta() > 0.0
                            ? options_->max_vel_theta()
                            : kDefaultMaxVelTheta;
  robot.acc_lim_x =
      options_->acc_lim_x() > 0.0 ? options_->acc_lim_x() : kDefaultAccLimX;
  robot.acc_lim_theta = options_->acc_lim_theta() > 0.0
                            ? options_->acc_lim_theta()
                            : kDefaultAccLimTheta;
  robot.min_turning_radius = options_->min_turning_radius() > 0.0
                                 ? options_->min_turning_radius()
                                 : 0.0;

  auto& traj = teb_config_.trajectory;
  if (options_->dt_ref() > 0.0) {
    traj.dt_ref = options_->dt_ref();
  }
  if (options_->dt_hysteresis() > 0.0) {
    traj.dt_hysteresis = options_->dt_hysteresis();
  }
  if (options_->min_samples() > 0) {
    traj.min_samples = options_->min_samples();
  }
  if (options_->max_samples() > 0) {
    traj.max_samples = options_->max_samples();
  }
  if (options_->max_global_plan_lookahead_dist() > 0.0) {
    traj.max_global_plan_lookahead_dist =
        options_->max_global_plan_lookahead_dist();
  }
  if (options_->global_plan_prune_distance() > 0.0) {
    traj.global_plan_prune_distance = options_->global_plan_prune_distance();
  }
  if (options_->control_look_ahead_poses() > 0) {
    traj.control_look_ahead_poses = options_->control_look_ahead_poses();
  }
  traj.global_plan_overwrite_orientation =
      options_->global_plan_overwrite_orientation();

  auto& obs = teb_config_.obstacles;
  if (options_->min_obstacle_dist() > 0.0) {
    obs.min_obstacle_dist = options_->min_obstacle_dist();
  }
  if (options_->inflation_dist() > 0.0) {
    obs.inflation_dist = options_->inflation_dist();
  }
  obs.include_costmap_obstacles = options_->include_costmap_obstacles();
  if (options_->costmap_obstacles_behind_robot_dist() >= 0.0) {
    obs.costmap_obstacles_behind_robot_dist =
        options_->costmap_obstacles_behind_robot_dist();
  }
  if (options_->obstacle_poses_affected() > 0) {
    obs.obstacle_poses_affected = options_->obstacle_poses_affected();
  }

  auto& optim = teb_config_.optim;
  if (options_->no_inner_iterations() > 0) {
    optim.no_inner_iterations = options_->no_inner_iterations();
  }
  if (options_->no_outer_iterations() > 0) {
    optim.no_outer_iterations = options_->no_outer_iterations();
  }
  if (options_->weight_max_vel_x() > 0.0) {
    optim.weight_max_vel_x = options_->weight_max_vel_x();
  }
  if (options_->weight_max_vel_theta() > 0.0) {
    optim.weight_max_vel_theta = options_->weight_max_vel_theta();
  }
  if (options_->weight_acc_lim_x() > 0.0) {
    optim.weight_acc_lim_x = options_->weight_acc_lim_x();
  }
  if (options_->weight_acc_lim_theta() > 0.0) {
    optim.weight_acc_lim_theta = options_->weight_acc_lim_theta();
  }
  if (options_->weight_kinematics_nh() > 0.0) {
    optim.weight_kinematics_nh = options_->weight_kinematics_nh();
  }
  if (options_->weight_kinematics_forward_drive() > 0.0) {
    optim.weight_kinematics_forward_drive =
        options_->weight_kinematics_forward_drive();
  }
  if (options_->weight_kinematics_turning_radius() > 0.0) {
    optim.weight_kinematics_turning_radius =
        options_->weight_kinematics_turning_radius();
  }
  if (options_->weight_optimaltime() > 0.0) {
    optim.weight_optimaltime = options_->weight_optimaltime();
  }
  if (options_->weight_obstacle() > 0.0) {
    optim.weight_obstacle = options_->weight_obstacle();
  }
  if (options_->weight_viapoint() > 0.0) {
    optim.weight_viapoint = options_->weight_viapoint();
  }
  if (options_->weight_shortest_path() > 0.0) {
    optim.weight_shortest_path = options_->weight_shortest_path();
  }
  if (options_->penalty_epsilon() > 0.0) {
    optim.penalty_epsilon = options_->penalty_epsilon();
  }

  teb_config_.goal_tolerance.free_goal_vel = options_->free_goal_vel();

  const std::string& model = options_->robot_model();
  if (model == "carlike" || model == "ackermann") {
    teb_config_.robot.min_turning_radius =
        std::max(robot.min_turning_radius, 0.1);
  }
}

teb_local_planner::PoseSE2 Optimizer::ToPoseSE2(
    const commsgs::geometry_msgs::Pose& pose) {
  return teb_local_planner::PoseSE2(
      pose.position.x, pose.position.y, getYaw(pose.orientation));
}

teb_local_planner::Twist Optimizer::ToTebTwist(
    const commsgs::geometry_msgs::Twist& twist) {
  teb_local_planner::Twist out;
  out.linear.x = twist.linear.x;
  out.linear.y = twist.linear.y;
  out.angular.z = twist.angular.z;
  return out;
}

commsgs::geometry_msgs::TwistStamped Optimizer::evalControl(
    const commsgs::geometry_msgs::PoseStamped& pose,
    const commsgs::geometry_msgs::Twist& velocity,
    const std::vector<commsgs::geometry_msgs::PoseStamped>& plan,
    const commsgs::geometry_msgs::PoseStamped& goal) {
  if (!planner_ || !costmap_wrapper_ || plan.size() < 2) {
    throw common::NoValidControl("TEB optimizer missing plan or planner");
  }

  auto* costmap = costmap_wrapper_->getCostmap();
  obstacle_converter_->update(*costmap, pose.pose);
  obstacles_ = obstacle_converter_->obstacles();

  via_points_.clear();
  if (options_->weight_viapoint() > 0.0) {
    for (size_t i = 1; i + 1 < plan.size(); ++i) {
      via_points_.emplace_back(plan[i].pose.position.x,
                               plan[i].pose.position.y);
    }
  }

  const auto start = ToPoseSE2(pose.pose);
  const auto local_goal = ToPoseSE2(goal.pose);
  const teb_local_planner::Twist start_vel = ToTebTwist(velocity);

  bool success = false;
  if (!has_plan_) {
    success = planner_->plan(plan, &start_vel, false);
    has_plan_ = success;
  } else {
    success = planner_->plan(start, local_goal, &start_vel, false);
  }

  if (!success) {
    success = planner_->plan(start, local_goal, &start_vel, false);
  }
  if (!success) {
    throw common::NoValidControl("TEB optimization failed");
  }

  double vx = 0.0;
  double vy = 0.0;
  double omega = 0.0;
  if (!planner_->getVelocityCommand(vx, vy, omega,
                                  teb_config_.trajectory.control_look_ahead_poses)) {
    throw common::NoValidControl("TEB velocity extraction failed");
  }

  commsgs::geometry_msgs::TwistStamped cmd;
  cmd.header = pose.header;
  cmd.twist.linear.x = vx;
  cmd.twist.linear.y = vy;
  cmd.twist.angular.z = omega;
  return cmd;
}

void Optimizer::reset() { has_plan_ = false; }

}  // namespace teb_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
