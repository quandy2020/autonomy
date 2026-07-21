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

#include <algorithm>
#include <chrono>
#include <cmath>

#include "autolink/common/log.hpp"
#include "autonomy/control/common/controller_exceptions.hpp"
#include "autonomy/map/costmap_2d/filters/filter_values.hpp"
#include "autonomy/map/costmap_2d/footprint.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace teb_controller {

namespace {

constexpr double kDefaultMaxVelX = 0.5;
constexpr double kDefaultMaxVelTheta = 1.0;
constexpr double kDefaultAccLimX = 0.5;
constexpr double kDefaultAccLimTheta = 0.5;
constexpr double kMinInscribedRadius = 1e-3;

void SynthesizeCircularFootprint(double radius, std::vector<Point>* out) {
    out->clear();
    if (radius < kMinInscribedRadius) {
        return;
    }
    constexpr int kSamples = 16;
    out->reserve(kSamples);
    for (int i = 0; i < kSamples; ++i) {
        const double theta = 2.0 * M_PI * static_cast<double>(i) /
                             static_cast<double>(kSamples);
        Point p;
        p.x = radius * std::cos(theta);
        p.y = radius * std::sin(theta);
        out->push_back(p);
    }
}

}  // namespace

void Optimizer::initialize(
    const std::string& name,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap,
    const proto::TEBControllerOptions* options, double controller_frequency) {
    name_ = name;
    costmap_wrapper_ = std::move(costmap);
    options_ = options;
    teb_config_ = TebConfig();
    applyOptionsToConfig();

    if (controller_frequency > 0.0 && teb_config_.trajectory.dt_ref <= 0.0) {
        teb_config_.trajectory.dt_ref = 1.0 / controller_frequency;
    }

    obstacle_converter_ =
        std::make_unique<tools::CostmapObstacleConverter>(*options_);

    controller_frequency_ =
        controller_frequency > 0.0 ? controller_frequency : 5.0;
    failure_detector_.setBufferLength(static_cast<std::size_t>(
        std::round(teb_config_.recovery.oscillation_filter_duration *
                   controller_frequency_)));

    if (costmap_wrapper_) {
        costmap_model_.setCostmap(costmap_wrapper_->getCostmap());
        updateFootprintFromCostmap();
    }
    configureRobotModel();
    validateFootprints();

    if (teb_config_.hcp.enable_homotopy_class_planning) {
        planner_ = std::make_unique<HomotopyClassPlanner>(
            teb_config_, &obstacles_, TebVisualization::SharedPtr(),
            &via_points_);
        AINFO << "TEB parallel planning in distinctive topologies enabled.";
    } else {
        planner_ = std::make_unique<TebOptimalPlanner>(
            teb_config_, &obstacles_, TebVisualization::SharedPtr(),
            &via_points_);
        AINFO << "TEB parallel planning in distinctive topologies disabled.";
    }
    has_plan_ = false;
    no_infeasible_plans_ = 0;
}

void Optimizer::updateFootprintFromCostmap() {
    if (!costmap_wrapper_) {
        return;
    }
    footprint_spec_ = costmap_wrapper_->getRobotFootprint();
    if (auto* layered = costmap_wrapper_->getLayeredCostmap()) {
        robot_inscribed_radius_ = layered->getInscribedRadius();
        robot_circumscribed_radius_ = layered->getCircumscribedRadius();
    }
    if (robot_inscribed_radius_ < kMinInscribedRadius &&
        !footprint_spec_.empty()) {
        const auto radii =
            map::costmap_2d::calculateMinAndMaxDistances(footprint_spec_);
        robot_inscribed_radius_ = std::max(radii.first, kMinInscribedRadius);
        robot_circumscribed_radius_ =
            std::max(radii.second, robot_inscribed_radius_);
    } else if (robot_inscribed_radius_ < kMinInscribedRadius) {
        robot_inscribed_radius_ = kMinInscribedRadius;
    }

    if (footprint_spec_.empty()) {
        SynthesizeCircularFootprint(robot_inscribed_radius_, &footprint_spec_);
        if (!footprint_spec_.empty()) {
            AINFO << "TEB: empty costmap footprint; synthesized circular "
                     "footprint with r="
                  << robot_inscribed_radius_;
        }
    }
}

void Optimizer::configureRobotModel() {
    const std::string model =
        options_ ? options_->robot_model() : std::string("point");

    if (model == "point") {
        teb_config_.robot_model = std::make_shared<PointRobotFootprint>();
    } else if (model == "circular" || model == "circle") {
        teb_config_.robot_model = std::make_shared<CircularRobotFootprint>(
            std::max(robot_inscribed_radius_, kMinInscribedRadius));
    } else if (model == "polygon" ||
               (!footprint_spec_.empty() && model != "point")) {
        Point2dContainer vertices;
        vertices.reserve(footprint_spec_.size());
        for (const auto& p : footprint_spec_) {
            vertices.emplace_back(p.x, p.y);
        }
        if (vertices.size() >= 3) {
            teb_config_.robot_model =
                std::make_shared<PolygonRobotFootprint>(vertices);
        } else {
            teb_config_.robot_model = std::make_shared<CircularRobotFootprint>(
                std::max(robot_inscribed_radius_, kMinInscribedRadius));
        }
    } else {
        // carlike / ackermann / default: circular approximation
        teb_config_.robot_model = std::make_shared<CircularRobotFootprint>(
            std::max(robot_inscribed_radius_, kMinInscribedRadius));
    }

    if (planner_) {
        planner_->updateRobotModel(teb_config_.robot_model);
    }
}

void Optimizer::validateFootprints() const {
    if (!teb_config_.robot_model) {
        return;
    }
    const double opt_r = teb_config_.robot_model->getInscribedRadius();
    const double min_obst = teb_config_.obstacles.min_obstacle_dist;
    if (opt_r + min_obst + 1e-6 < robot_inscribed_radius_) {
        AWARN << "TEB optimize inscribed radius (" << opt_r
              << ") + min_obstacle_dist (" << min_obst
              << ") < costmap inscribed radius (" << robot_inscribed_radius_
              << "). Infeasible results may occur frequently.";
    }
}

void Optimizer::applyOptionsToConfig() {
    if (!options_) {
        return;
    }

    auto& robot = teb_config_.robot;
    robot.max_vel_x =
        options_->max_vel_x() > 0.0 ? options_->max_vel_x() : kDefaultMaxVelX;
    robot.max_vel_x_backwards = options_->max_vel_x_backwards() > 0.0
                                    ? options_->max_vel_x_backwards()
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

    base_max_vel_x_ = robot.max_vel_x;
    base_max_vel_x_backwards_ = robot.max_vel_x_backwards;
    base_max_vel_y_ = robot.max_vel_y;
    base_max_vel_theta_ = robot.max_vel_theta;

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
        traj.global_plan_prune_distance =
            options_->global_plan_prune_distance();
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

    auto& hcp = teb_config_.hcp;
    hcp.enable_homotopy_class_planning =
        options_->enable_homotopy_class_planning();
    hcp.enable_multithreading = options_->enable_multithreading();
    if (options_->max_number_classes() > 0) {
        hcp.max_number_classes = options_->max_number_classes();
    }
    teb_config_.recovery.oscillation_recovery =
        options_->oscillation_recovery();
    if (options_->oscillation_filter_duration() > 0.0) {
        teb_config_.recovery.oscillation_filter_duration =
            options_->oscillation_filter_duration();
    }

    const std::string& model = options_->robot_model();
    if (model == "carlike" || model == "ackermann") {
        teb_config_.robot.min_turning_radius =
            std::max(robot.min_turning_radius, 0.1);
    }
}

void Optimizer::setSpeedLimit(double speed_limit, bool percentage) {
    auto& robot = teb_config_.robot;
    if (speed_limit == map::costmap_2d::NO_SPEED_LIMIT) {
        robot.max_vel_x = base_max_vel_x_;
        robot.max_vel_x_backwards = base_max_vel_x_backwards_;
        robot.max_vel_y = base_max_vel_y_;
        robot.max_vel_theta = base_max_vel_theta_;
        return;
    }

    if (percentage) {
        const double ratio = speed_limit / 100.0;
        robot.max_vel_x = base_max_vel_x_ * ratio;
        robot.max_vel_x_backwards = base_max_vel_x_backwards_ * ratio;
        robot.max_vel_y = base_max_vel_y_ * ratio;
        robot.max_vel_theta = base_max_vel_theta_ * ratio;
    } else {
        const double max_speed_xy =
            std::max(std::max(base_max_vel_x_, base_max_vel_x_backwards_),
                     base_max_vel_y_);
        if (max_speed_xy > 0.0 && speed_limit < max_speed_xy) {
            const double ratio = speed_limit / max_speed_xy;
            robot.max_vel_x = base_max_vel_x_ * ratio;
            robot.max_vel_x_backwards = base_max_vel_x_backwards_ * ratio;
            robot.max_vel_y = base_max_vel_y_ * ratio;
            robot.max_vel_theta = base_max_vel_theta_ * ratio;
        }
    }
}

void Optimizer::saturateVelocity(double& vx, double& vy, double& omega) const {
    const auto& robot = teb_config_.robot;
    double ratio_x = 1.0;
    double ratio_y = 1.0;
    double ratio_omega = 1.0;

    if (vx > robot.max_vel_x && robot.max_vel_x > 0.0) {
        ratio_x = robot.max_vel_x / vx;
    }
    if (std::abs(vy) > robot.max_vel_y && robot.max_vel_y > 0.0) {
        ratio_y = std::abs(robot.max_vel_y / vy);
    } else if (robot.max_vel_y <= 0.0) {
        vy = 0.0;
    }
    if (std::abs(omega) > robot.max_vel_theta && robot.max_vel_theta > 0.0) {
        ratio_omega = std::abs(robot.max_vel_theta / omega);
    }
    if (robot.max_vel_x_backwards <= 0.0) {
        // Keep optimization weight for backwards; do not hard-block here.
    } else if (vx < -robot.max_vel_x_backwards) {
        ratio_x = -robot.max_vel_x_backwards / vx;
    }

    if (robot.use_proportional_saturation) {
        const double ratio = std::min(std::min(ratio_x, ratio_y), ratio_omega);
        vx *= ratio;
        vy *= ratio;
        omega *= ratio;
    } else {
        vx *= ratio_x;
        vy *= ratio_y;
        omega *= ratio_omega;
    }
}

void Optimizer::applyShrinkHorizon(
    std::vector<commsgs::geometry_msgs::PoseStamped>& plan) const {
    if (!teb_config_.recovery.shrink_horizon_backup || plan.size() < 3) {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    const bool recently_infeasible =
        std::chrono::duration<double>(now - time_last_infeasible_plan_)
            .count() < teb_config_.recovery.shrink_horizon_min_duration;
    if (no_infeasible_plans_ <= 0 && !recently_infeasible) {
        return;
    }

    int goal_idx = static_cast<int>(plan.size()) - 1;
    int horizon_reduction = goal_idx / 2;
    if (no_infeasible_plans_ > 9) {
        horizon_reduction /= 2;
    }
    const int new_end =
        static_cast<int>(plan.size()) - horizon_reduction - 1;
    if (new_end > 1 && goal_idx - horizon_reduction >= 0) {
        plan.erase(plan.begin() + new_end, plan.end());
        if (no_infeasible_plans_ == 1) {
            AINFO << "TEB: reduced horizon backup for at least "
                  << teb_config_.recovery.shrink_horizon_min_duration << " s";
        }
    }
}

void Optimizer::markInfeasible() {
    ++no_infeasible_plans_;
    time_last_infeasible_plan_ = std::chrono::steady_clock::now();
    has_plan_ = false;
    if (planner_) {
        planner_->clearPlanner();
    }
}

void Optimizer::clearInfeasible() {
    no_infeasible_plans_ = 0;
}

PoseSE2 Optimizer::ToPoseSE2(const commsgs::geometry_msgs::Pose& pose) {
    return PoseSE2(pose.position.x, pose.position.y, getYaw(pose.orientation));
}

Twist Optimizer::ToTebTwist(const commsgs::geometry_msgs::Twist& twist) {
    Twist out;
    out.linear.x = twist.linear.x;
    out.linear.y = twist.linear.y;
    out.angular.z = twist.angular.z;
    return out;
}

commsgs::geometry_msgs::TwistStamped Optimizer::evalControl(
    const commsgs::geometry_msgs::PoseStamped& pose,
    const commsgs::geometry_msgs::Twist& velocity,
    const std::vector<commsgs::geometry_msgs::PoseStamped>& plan_in,
    const commsgs::geometry_msgs::PoseStamped& goal) {
    if (!planner_ || !costmap_wrapper_ || plan_in.size() < 2) {
        throw common::NoValidControl("TEB optimizer missing plan or planner");
    }

    auto plan = plan_in;
    applyShrinkHorizon(plan);
    if (plan.size() < 2) {
        markInfeasible();
        throw common::NoValidControl("TEB plan too short after shrink horizon");
    }

    auto* costmap = costmap_wrapper_->getCostmap();
    obstacle_converter_->update(*costmap, pose.pose);
    obstacles_ = obstacle_converter_->obstacles();

    via_points_.clear();
    if (options_ && options_->weight_viapoint() > 0.0) {
        for (size_t i = 1; i + 1 < plan.size(); ++i) {
            via_points_.emplace_back(plan[i].pose.position.x,
                                     plan[i].pose.position.y);
        }
    }

    const auto start = ToPoseSE2(pose.pose);
    const auto local_goal = ToPoseSE2(plan.back().pose);
    (void)goal;
    const Twist start_vel = ToTebTwist(velocity);
    const bool free_goal_vel = teb_config_.goal_tolerance.free_goal_vel;

    bool success = false;
    if (!has_plan_) {
        success = planner_->plan(plan, &start_vel, free_goal_vel);
        has_plan_ = success;
    } else {
        success =
            planner_->plan(start, local_goal, &start_vel, free_goal_vel);
    }

    if (!success) {
        success =
            planner_->plan(start, local_goal, &start_vel, free_goal_vel);
    }
    if (!success) {
        markInfeasible();
        throw common::NoValidControl("TEB optimization failed");
    }

    if (planner_->hasDiverged()) {
        markInfeasible();
        throw common::NoValidControl(
            "TEB trajectory has diverged. Resetting planner...");
    }

    if (teb_config_.robot.is_footprint_dynamic) {
        updateFootprintFromCostmap();
        configureRobotModel();
    }
    costmap_model_.setCostmap(costmap);

    if (!footprint_spec_.empty()) {
        const bool feasible = planner_->isTrajectoryFeasible(
            &costmap_model_, footprint_spec_, robot_inscribed_radius_,
            robot_circumscribed_radius_,
            teb_config_.trajectory.feasibility_check_no_poses,
            teb_config_.trajectory.feasibility_check_lookahead_distance);
        if (!feasible) {
            markInfeasible();
            throw common::NoValidControl(
                "TEB trajectory is not feasible. Resetting planner...");
        }
    }

    double vx = 0.0;
    double vy = 0.0;
    double omega = 0.0;
    if (!planner_->getVelocityCommand(
            vx, vy, omega, teb_config_.trajectory.control_look_ahead_poses)) {
        markInfeasible();
        throw common::NoValidControl("TEB velocity extraction failed");
    }

    saturateVelocity(vx, vy, omega);

    commsgs::geometry_msgs::TwistStamped cmd;
    cmd.header = pose.header;
    cmd.twist.linear.x = vx;
    cmd.twist.linear.y = vy;
    cmd.twist.angular.z = omega;

    if (teb_config_.recovery.oscillation_recovery) {
        const Twist last_cmd = ToTebTwist(cmd.twist);
        failure_detector_.update(last_cmd, teb_config_.robot.max_vel_x,
                                 teb_config_.robot.max_vel_x_backwards,
                                 teb_config_.robot.max_vel_theta,
                                 teb_config_.recovery.oscillation_v_eps,
                                 teb_config_.recovery.oscillation_omega_eps);

        const auto now = std::chrono::steady_clock::now();
        const bool recently_oscillated =
            std::chrono::duration<double>(now - time_last_oscillation_)
                .count() <
            teb_config_.recovery.oscillation_recovery_min_duration;

        if (failure_detector_.isOscillating()) {
            time_last_oscillation_ = now;
            if (last_preferred_rotdir_ == RotType::none) {
                last_preferred_rotdir_ = (last_cmd.angular.z >= 0.0)
                                             ? RotType::left
                                             : RotType::right;
            }
            planner_->setPreferredTurningDir(last_preferred_rotdir_);
        } else if (recently_oscillated &&
                   last_preferred_rotdir_ != RotType::none) {
            planner_->setPreferredTurningDir(last_preferred_rotdir_);
        } else {
            last_preferred_rotdir_ = RotType::none;
            planner_->setPreferredTurningDir(RotType::none);
        }
    }

    clearInfeasible();
    planner_->visualize();
    return cmd;
}

void Optimizer::reset() {
    has_plan_ = false;
    no_infeasible_plans_ = 0;
    failure_detector_.clear();
    last_preferred_rotdir_ = RotType::none;
    if (planner_) {
        planner_->clearPlanner();
    }
}

}  // namespace teb_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
