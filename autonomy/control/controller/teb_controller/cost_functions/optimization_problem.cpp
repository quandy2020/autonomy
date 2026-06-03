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

#include "autonomy/control/controller/teb_controller/cost_functions/optimization_problem.hpp"

#include <cmath>
#include <limits>

#include "autonomy/common/math/math.hpp"
#include "autonomy/control/controller/teb_controller/cost_functions/acceleration_cost_function.hpp"
#include "autonomy/control/controller/teb_controller/cost_functions/kinematics_cost_function.hpp"
#include "autonomy/control/controller/teb_controller/cost_functions/obstacle_cost_function.hpp"
#include "autonomy/control/controller/teb_controller/cost_functions/preferred_rotation_cost_function.hpp"
#include "autonomy/control/controller/teb_controller/cost_functions/trajectory_cost_function.hpp"
#include "autonomy/control/controller/teb_controller/cost_functions/velocity_cost_function.hpp"

namespace autonomy {
namespace control {
namespace teb_controller {

namespace {

void SetFixedTebParameters(ceres::Problem& problem,
                           TimedElasticBand& timed_elastic_band) {
    for (int i = 0; i < timed_elastic_band.SizePoses(); ++i) {
        if (timed_elastic_band.IsPoseVertexFixed(i)) {
            problem.SetParameterBlockConstant(&timed_elastic_band.Pose(i).x);
            problem.SetParameterBlockConstant(&timed_elastic_band.Pose(i).y);
            problem.SetParameterBlockConstant(
                &timed_elastic_band.Pose(i).theta);
        }
    }
    for (int i = 0; i < timed_elastic_band.SizeTimeDiffs(); ++i) {
        if (timed_elastic_band.IsTimeDiffVertexFixed(i)) {
            problem.SetParameterBlockConstant(&timed_elastic_band.TimeDiff(i));
        }
    }
}

}  // namespace

void TebOptimizationProblem::SetBuildContext(
    const TimedElasticBandConfig& config, TimedElasticBand& timed_elastic_band,
    ObstContainer* obstacles, const ViaPointContainer* via_points,
    const RobotFootprint* robot_model,
    std::vector<ObstContainer>& obstacles_per_vertex,
    PreferredRotationDirection preferred_rotation_direction,
    const std::pair<bool, commsgs::geometry_msgs::Twist>& velocity_start,
    const std::pair<bool, commsgs::geometry_msgs::Twist>& velocity_goal) {
    config_ = &config;
    teb_ = &timed_elastic_band;
    obstacles_ = obstacles;
    via_points_ = via_points;
    robot_model_ = robot_model;
    obstacles_per_vertex_ = &obstacles_per_vertex;
    prefer_rotdir_ = preferred_rotation_direction;
    vel_start_ = velocity_start;
    vel_goal_ = velocity_goal;
}

void TebOptimizationProblem::RegisterResidualBlock(ceres::ResidualBlockId id,
                                                   TebResidualKind kind) {
    const ceres::CostFunction* cost =
        problem_.GetCostFunctionForResidualBlock(id);
    residual_block_ids_.push_back(id);
    residual_meta_.push_back({kind, cost->num_residuals()});
}

void TebOptimizationProblem::PrepareObstacleAssociation() {
    obstacles_per_vertex_->resize(teb_->SizePoses());
    for (auto& per_vertex : obstacles_per_vertex_) {
        per_vertex.clear();
        if (obstacles_ != nullptr) {
            per_vertex.reserve(obstacles_->size());
        }
    }
}

bool TebOptimizationProblem::Build(double weight_multiplier) {
    PrepareObstacleAssociation();

    if (config_->obstacles.legacy_obstacle_association) {
        AddObstacleResidualsLegacy(weight_multiplier);
    } else {
        AddObstacleResiduals(weight_multiplier);
    }

    if (config_->obstacles.include_dynamic_obstacles) {
        AddDynamicObstacleResiduals(weight_multiplier);
    }

    AddViaPointResiduals();
    AddVelocityResiduals();
    AddAccelerationResiduals();
    AddTimeOptimalResiduals();
    AddShortestPathResiduals();

    if (config_->robot.min_turning_radius == 0 ||
        config_->optimization.weight_kinematics_turning_radius == 0) {
        AddDiffDriveKinematicsResiduals();
    } else {
        AddCarlikeKinematicsResiduals();
    }

    AddPreferredRotationDirectionResiduals();

    if (config_->optimization.weight_velocity_obstacle_ratio > 0) {
        AddVelocityObstacleRatioResiduals();
    }

    SetFixedTebParameters(problem_, teb_);
    return true;
}

void TebOptimizationProblem::AddObstacleResiduals(double weight_multiplier) {
    if (config_->optimization.weight_obstacle == 0 || weight_multiplier == 0 ||
        obstacles_ == nullptr || robot_model_ == nullptr) {
        return;
    }

    const bool inflated =
        config_->obstacles.inflation_dist > config_->obstacles.min_obstacle_dist;
    const double weight_obstacle =
        config_->optimization.weight_obstacle * weight_multiplier;

    auto iter_obstacle = obstacles_per_vertex_->begin();
    const int first_vertex =
        config_->optimization.weight_velocity_obstacle_ratio == 0 ? 1 : 0;

    for (int i = first_vertex; i < teb_->SizePoses() - 1; ++i) {
        double left_min_dist = std::numeric_limits<double>::max();
        double right_min_dist = std::numeric_limits<double>::max();
        ObstaclePtr left_obstacle;
        ObstaclePtr right_obstacle;
        const Point pose_orient = OrientationUnitVec(teb_->Pose(i));

        for (const ObstaclePtr& obstacle : *obstacles_) {
            if (config_->obstacles.include_dynamic_obstacles &&
                obstacle->IsDynamic()) {
                continue;
            }
            double dist =
                robot_model_->CalculateDistance(teb_->Pose(i), obstacle.get());
            if (dist < config_->obstacles.min_obstacle_dist *
                           config_->obstacles
                               .obstacle_association_force_inclusion_factor) {
                iter_obstacle->push_back(obstacle);
                continue;
            }
            if (dist > config_->obstacles.min_obstacle_dist *
                           config_->obstacles.obstacle_association_cutoff_factor) {
                continue;
            }
            if (Cross2d(pose_orient, obstacle->GetCentroid()) > 0) {
                if (dist < left_min_dist) {
                    left_min_dist = dist;
                    left_obstacle = obstacle;
                }
            } else {
                if (dist < right_min_dist) {
                    right_min_dist = dist;
                    right_obstacle = obstacle;
                }
            }
        }

        if (left_obstacle) {
            iter_obstacle->push_back(left_obstacle);
        }
        if (right_obstacle) {
            iter_obstacle->push_back(right_obstacle);
        }

        if (i == 0) {
            ++iter_obstacle;
            continue;
        }

        Pose2D& pose = teb_->Pose(i);
        for (const ObstaclePtr& obstacle : *iter_obstacle) {
            ceres::CostFunction* cost = nullptr;
            if (inflated) {
                cost = CreateInflatedObstacleCostFunction(
                    config_, robot_model_, obstacle.get(), weight_obstacle,
                    config_->optimization.weight_inflation);
            } else {
                cost = CreateObstacleCostFunction(
                    config_, robot_model_, obstacle.get(), weight_obstacle);
            }
            RegisterResidualBlock(
                problem_.AddResidualBlock(cost, nullptr, &pose.x, &pose.y,
                                          &pose.theta),
                TebResidualKind::Obstacle);
        }
        ++iter_obstacle;
    }
}

void TebOptimizationProblem::AddObstacleResidualsLegacy(
    double weight_multiplier) {
    if (config_->optimization.weight_obstacle == 0 || weight_multiplier == 0 ||
        obstacles_ == nullptr || robot_model_ == nullptr) {
        return;
    }

    const double weight_obstacle =
        config_->optimization.weight_obstacle * weight_multiplier;
    const bool inflated =
        config_->obstacles.inflation_dist > config_->obstacles.min_obstacle_dist;

    for (const ObstaclePtr& obstacle : *obstacles_) {
        if (config_->obstacles.include_dynamic_obstacles &&
            obstacle->IsDynamic()) {
            continue;
        }

        int index =
            (config_->obstacles.obstacle_poses_affected >= teb_->SizePoses())
                ? teb_->SizePoses() / 2
                : teb_->FindClosestTrajectoryPose(*(obstacle.get()));

        if (index <= 1 || index > teb_->SizePoses() - 2) {
            continue;
        }

        auto AddAt = [&](int pose_index) {
            Pose2D& pose = teb_->Pose(pose_index);
            ceres::CostFunction* cost = nullptr;
            if (inflated) {
                cost = CreateInflatedObstacleCostFunction(
                    config_, robot_model_, obstacle.get(), weight_obstacle,
                    config_->optimization.weight_inflation);
            } else {
                cost = CreateObstacleCostFunction(
                    config_, robot_model_, obstacle.get(), weight_obstacle);
            }
            RegisterResidualBlock(
                problem_.AddResidualBlock(cost, nullptr, &pose.x, &pose.y,
                                          &pose.theta),
                TebResidualKind::Obstacle);
        };

        AddAt(index);
        const int half = static_cast<int>(
            std::floor(config_->obstacles.obstacle_poses_affected / 2));
        for (int neighbour = 0; neighbour < half; ++neighbour) {
            if (index + neighbour < teb_->SizePoses()) {
                AddAt(index + neighbour);
            }
            if (index - neighbour >= 0) {
                AddAt(index - neighbour);
            }
        }
    }
}

void TebOptimizationProblem::AddDynamicObstacleResiduals(
    double weight_multiplier) {
    if (config_->optimization.weight_obstacle == 0 || weight_multiplier == 0 ||
        obstacles_ == nullptr || robot_model_ == nullptr) {
        return;
    }

    const double weight_dynamic_obstacle =
        config_->optimization.weight_dynamic_obstacle * weight_multiplier;
    const double weight_dynamic_obstacle_inflation =
        config_->optimization.weight_dynamic_obstacle_inflation;

    for (const ObstaclePtr& obstacle : *obstacles_) {
        if (!obstacle->IsDynamic()) {
            continue;
        }
        double time_offset = teb_->TimeDiff(0);
        for (int i = 1; i < teb_->SizePoses() - 1; ++i) {
            Pose2D& pose = teb_->Pose(i);
            ceres::CostFunction* cost = CreateDynamicObstacleCostFunction(
                config_, robot_model_, obstacle.get(), time_offset,
                weight_dynamic_obstacle, weight_dynamic_obstacle_inflation);
            RegisterResidualBlock(
                problem_.AddResidualBlock(cost, nullptr, &pose.x, &pose.y,
                                          &pose.theta),
                TebResidualKind::Obstacle);
            time_offset += teb_->TimeDiff(i);
        }
    }
}

void TebOptimizationProblem::AddViaPointResiduals() {
    if (config_->optimization.weight_via_point == 0 || via_points_ == nullptr ||
        via_points_->empty()) {
        return;
    }

    const int n = teb_->SizePoses();
    if (n < 3) {
        return;
    }

    int start_pose_index = 0;
    for (const Point& via_point : *via_points_) {
        int index = teb_->FindClosestTrajectoryPose(via_point, nullptr,
                                                    start_pose_index);
        if (config_->trajectory.via_points_ordered) {
            start_pose_index = index + 2;
        }
        if (index > n - 2) {
            index = n - 2;
        }
        if (index < 1) {
            if (config_->trajectory.via_points_ordered) {
                index = 1;
            } else {
                continue;
            }
        }
        Pose2D& pose = teb_->Pose(index);
        ceres::CostFunction* cost = CreateViaPointCostFunction(
            via_point.x, via_point.y, config_->optimization.weight_via_point);
        RegisterResidualBlock(
            problem_.AddResidualBlock(cost, nullptr, &pose.x, &pose.y),
            TebResidualKind::ViaPoint);
    }
}

void TebOptimizationProblem::AddVelocityResiduals() {
    if (config_->robot.max_velocity_y == 0) {
        if (config_->optimization.weight_max_velocity_x == 0 &&
            config_->optimization.weight_max_angular_velocity == 0) {
            return;
        }
        for (int i = 0; i < teb_->SizePoses() - 1; ++i) {
            Pose2D& p0 = teb_->Pose(i);
            Pose2D& p1 = teb_->Pose(i + 1);
            ceres::CostFunction* cost =
                CreateVelocityCostFunction(config_, config_->optimization.weight_max_velocity_x,
                                           config_->optimization.weight_max_angular_velocity);
            RegisterResidualBlock(
                problem_.AddResidualBlock(cost, nullptr, &p0.x, &p0.y,
                                          &p0.theta, &p1.x, &p1.y,
                                          &p1.theta, &teb_->TimeDiff(i)),
                TebResidualKind::Other);
        }
    } else {
        if (config_->optimization.weight_max_velocity_x == 0 &&
            config_->optimization.weight_max_velocity_y == 0 &&
            config_->optimization.weight_max_angular_velocity == 0) {
            return;
        }
        for (int i = 0; i < teb_->SizePoses() - 1; ++i) {
            Pose2D& p0 = teb_->Pose(i);
            Pose2D& p1 = teb_->Pose(i + 1);
            ceres::CostFunction* cost = CreateVelocityHolonomicCostFunction(
                config_, config_->optimization.weight_max_velocity_x,
                config_->optimization.weight_max_velocity_y, config_->optimization.weight_max_angular_velocity);
            RegisterResidualBlock(
                problem_.AddResidualBlock(cost, nullptr, &p0.x, &p0.y,
                                          &p0.theta, &p1.x, &p1.y,
                                          &p1.theta, &teb_->TimeDiff(i)),
                TebResidualKind::Other);
        }
    }
}

void TebOptimizationProblem::AddAccelerationResiduals() {
    if (config_->optimization.weight_max_acceleration_x == 0 &&
        config_->optimization.weight_max_angular_acceleration == 0) {
        return;
    }

    const int n = teb_->SizePoses();
    if (config_->robot.max_velocity_y == 0 || config_->robot.max_acceleration_y == 0) {
        if (vel_start_.first) {
            Pose2D& p0 = teb_->Pose(0);
            Pose2D& p1 = teb_->Pose(1);
            ceres::CostFunction* cost = CreateAccelerationStartCostFunction(
                config_, vel_start_.second.linear.x, vel_start_.second.angular.z,
                config_->optimization.weight_max_acceleration_x, config_->optimization.weight_max_angular_acceleration);
            RegisterResidualBlock(
                problem_.AddResidualBlock(cost, nullptr, &p0.x, &p0.y,
                                          &p0.theta, &p1.x, &p1.y,
                                          &p1.theta, &teb_->TimeDiff(0)),
                TebResidualKind::Other);
        }
        for (int i = 0; i < n - 2; ++i) {
            Pose2D& p0 = teb_->Pose(i);
            Pose2D& p1 = teb_->Pose(i + 1);
            Pose2D& p2 = teb_->Pose(i + 2);
            ceres::CostFunction* cost = CreateAccelerationCostFunction(
                config_, config_->optimization.weight_max_acceleration_x,
                config_->optimization.weight_max_angular_acceleration);
            RegisterResidualBlock(
                problem_.AddResidualBlock(
                    cost, nullptr, &p0.x, &p0.y, &p0.theta, &p1.x,
                    &p1.y, &p1.theta, &p2.x, &p2.y, &p2.theta,
                    &teb_->TimeDiff(i), &teb_->TimeDiff(i + 1)),
                TebResidualKind::Other);
        }
        if (vel_goal_.first) {
            Pose2D& p0 = teb_->Pose(n - 2);
            Pose2D& p1 = teb_->Pose(n - 1);
            ceres::CostFunction* cost = CreateAccelerationGoalCostFunction(
                config_, vel_goal_.second.linear.x, vel_goal_.second.angular.z,
                config_->optimization.weight_max_acceleration_x, config_->optimization.weight_max_angular_acceleration);
            RegisterResidualBlock(
                problem_.AddResidualBlock(
                    cost, nullptr, &p0.x, &p0.y, &p0.theta, &p1.x,
                    &p1.y, &p1.theta,
                    &teb_->TimeDiff(teb_->SizeTimeDiffs() - 1)),
                TebResidualKind::Other);
        }
    } else {
        if (vel_start_.first) {
            Pose2D& p0 = teb_->Pose(0);
            Pose2D& p1 = teb_->Pose(1);
            ceres::CostFunction* cost =
                CreateAccelerationHolonomicStartCostFunction(
                    config_, vel_start_.second.linear.x,
                    vel_start_.second.linear.y, vel_start_.second.angular.z,
                    config_->optimization.weight_max_acceleration_x, config_->optimization.weight_max_acceleration_y,
                    config_->optimization.weight_max_angular_acceleration);
            RegisterResidualBlock(
                problem_.AddResidualBlock(cost, nullptr, &p0.x, &p0.y,
                                          &p0.theta, &p1.x, &p1.y,
                                          &p1.theta, &teb_->TimeDiff(0)),
                TebResidualKind::Other);
        }
        for (int i = 0; i < n - 2; ++i) {
            Pose2D& p0 = teb_->Pose(i);
            Pose2D& p1 = teb_->Pose(i + 1);
            Pose2D& p2 = teb_->Pose(i + 2);
            ceres::CostFunction* cost = CreateAccelerationHolonomicCostFunction(
                config_, config_->optimization.weight_max_acceleration_x,
                config_->optimization.weight_max_acceleration_y, config_->optimization.weight_max_angular_acceleration);
            RegisterResidualBlock(
                problem_.AddResidualBlock(
                    cost, nullptr, &p0.x, &p0.y, &p0.theta, &p1.x,
                    &p1.y, &p1.theta, &p2.x, &p2.y, &p2.theta,
                    &teb_->TimeDiff(i), &teb_->TimeDiff(i + 1)),
                TebResidualKind::Other);
        }
        if (vel_goal_.first) {
            Pose2D& p0 = teb_->Pose(n - 2);
            Pose2D& p1 = teb_->Pose(n - 1);
            ceres::CostFunction* cost =
                CreateAccelerationHolonomicGoalCostFunction(
                    config_, vel_goal_.second.linear.x, vel_goal_.second.linear.y,
                    vel_goal_.second.angular.z, config_->optimization.weight_max_acceleration_x,
                    config_->optimization.weight_max_acceleration_y,
                    config_->optimization.weight_max_angular_acceleration);
            RegisterResidualBlock(
                problem_.AddResidualBlock(
                    cost, nullptr, &p0.x, &p0.y, &p0.theta, &p1.x,
                    &p1.y, &p1.theta,
                    &teb_->TimeDiff(teb_->SizeTimeDiffs() - 1)),
                TebResidualKind::Other);
        }
    }
}

void TebOptimizationProblem::AddTimeOptimalResiduals() {
    if (config_->optimization.weight_time_optimal == 0) {
        return;
    }
    for (int i = 0; i < teb_->SizeTimeDiffs(); ++i) {
        ceres::CostFunction* cost =
            CreateTimeOptimalCostFunction(config_->optimization.weight_time_optimal);
        RegisterResidualBlock(
            problem_.AddResidualBlock(cost, nullptr, &teb_->TimeDiff(i)),
            TebResidualKind::TimeOptimal);
    }
}

void TebOptimizationProblem::AddShortestPathResiduals() {
    if (config_->optimization.weight_shortest_path == 0) {
        return;
    }
    for (int i = 0; i < teb_->SizePoses() - 1; ++i) {
        Pose2D& p0 = teb_->Pose(i);
        Pose2D& p1 = teb_->Pose(i + 1);
        ceres::CostFunction* cost =
            CreateShortestPathCostFunction(config_->optimization.weight_shortest_path);
        RegisterResidualBlock(
            problem_.AddResidualBlock(cost, nullptr, &p0.x, &p0.y, &p1.x,
                                      &p1.y),
            TebResidualKind::Other);
    }
}

void TebOptimizationProblem::AddDiffDriveKinematicsResiduals() {
    if (config_->optimization.weight_kinematics_non_holonomic == 0 &&
        config_->optimization.weight_kinematics_forward_drive == 0) {
        return;
    }
    for (int i = 0; i < teb_->SizePoses() - 1; ++i) {
        Pose2D& p0 = teb_->Pose(i);
        Pose2D& p1 = teb_->Pose(i + 1);
        ceres::CostFunction* cost = CreateKinematicsDiffDriveCostFunction(
            config_->optimization.weight_kinematics_non_holonomic,
            config_->optimization.weight_kinematics_forward_drive);
        RegisterResidualBlock(problem_.AddResidualBlock(
                                  cost, nullptr, &p0.x, &p0.y, &p0.theta,
                                  &p1.x, &p1.y, &p1.theta),
                              TebResidualKind::Other);
    }
}

void TebOptimizationProblem::AddCarlikeKinematicsResiduals() {
    if (config_->optimization.weight_kinematics_non_holonomic == 0 &&
        config_->optimization.weight_kinematics_turning_radius == 0) {
        return;
    }
    for (int i = 0; i < teb_->SizePoses() - 1; ++i) {
        Pose2D& p0 = teb_->Pose(i);
        Pose2D& p1 = teb_->Pose(i + 1);
        ceres::CostFunction* cost = CreateKinematicsCarlikeCostFunction(
            config_, config_->optimization.weight_kinematics_non_holonomic,
            config_->optimization.weight_kinematics_turning_radius);
        RegisterResidualBlock(problem_.AddResidualBlock(
                                  cost, nullptr, &p0.x, &p0.y, &p0.theta,
                                  &p1.x, &p1.y, &p1.theta),
                              TebResidualKind::Other);
    }
}

void TebOptimizationProblem::AddPreferredRotationDirectionResiduals() {
    if (prefer_rotdir_ == PreferredRotationDirection::none ||
        config_->optimization.weight_preferred_rotation_direction == 0) {
        return;
    }
    double rotation_sign = 1.0;
    if (prefer_rotdir_ == PreferredRotationDirection::right) {
        rotation_sign = -1.0;
    } else if (prefer_rotdir_ != PreferredRotationDirection::left) {
        return;
    }
    const int limit = std::min(teb_->SizePoses() - 1, 3);
    for (int i = 0; i < limit; ++i) {
        Pose2D& p0 = teb_->Pose(i);
        Pose2D& p1 = teb_->Pose(i + 1);
        ceres::CostFunction* cost =
            CreatePreferredRotationDirectionCostFunction(
                rotation_sign, config_->optimization.weight_preferred_rotation_direction);
        RegisterResidualBlock(
            problem_.AddResidualBlock(cost, nullptr, &p0.theta, &p1.theta),
            TebResidualKind::Other);
    }
}

void TebOptimizationProblem::AddVelocityObstacleRatioResiduals() {
    if (config_->optimization.weight_velocity_obstacle_ratio == 0 ||
        robot_model_ == nullptr) {
        return;
    }
    const double weight = config_->optimization.weight_velocity_obstacle_ratio;
    auto iter_obstacle = obstacles_per_vertex_->begin();
    for (int index = 0; index < teb_->SizePoses() - 1; ++index) {
        Pose2D& p0 = teb_->Pose(index);
        Pose2D& p1 = teb_->Pose(index + 1);
        for (const ObstaclePtr& obstacle : *iter_obstacle++) {
            ceres::CostFunction* cost = CreateVelocityObstacleRatioCostFunction(
                config_, robot_model_, obstacle.get(), weight);
            RegisterResidualBlock(
                problem_.AddResidualBlock(cost, nullptr, &p0.x, &p0.y,
                                          &p0.theta, &p1.x, &p1.y,
                                          &p1.theta, &teb_->TimeDiff(index)),
                TebResidualKind::Other);
        }
    }
}

void TebOptimizationProblem::Clear() {
    problem_ = ceres::Problem();
    summary_ = ceres::Solver::Summary();
    residual_meta_.clear();
    residual_block_ids_.clear();
}

bool TebOptimizationProblem::Solve(int max_iterations, bool verbose) {
    ceres::Solver::Options options;
    options.max_num_iterations = max_iterations;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = verbose;
    options.num_threads = 2;
    return ceres::Solve(options, &problem_, &summary_).IsSolutionUsable();
}

double TebOptimizationProblem::EvaluateTotalCost(double obstacle_cost_scale,
                                                 double via_point_cost_scale,
                                                 bool skip_time_optimal) const {
    ceres::Problem::EvaluateOptions options;
    options.apply_loss_function = true;
    std::vector<double> residuals;
    if (!problem_.Evaluate(options, nullptr, &residuals, nullptr, nullptr)) {
        return 0.0;
    }

    double cost = 0.0;
    int offset = 0;
    for (size_t i = 0; i < residual_meta_.size(); ++i) {
        const TebResidualMeta& meta = residual_meta_[i];
        double scale = 1.0;
        if (meta.kind == TebResidualKind::Obstacle) {
            scale = obst_cost_scale;
        } else if (meta.kind == TebResidualKind::ViaPoint) {
            scale = viapoint_cost_scale;
        } else if (meta.kind == TebResidualKind::TimeOptimal &&
                   skip_time_optimal) {
            offset += meta.num_residuals;
            continue;
        }
        for (int r = 0; r < meta.num_residuals; ++r) {
            const double v = residuals[offset + r];
            cost += scale * v * v;
        }
        offset += meta.num_residuals;
    }
    return cost;
}

bool TebOptimizationProblem::HasDiverged(const TimedElasticBandConfig& config) const {
    if (!config.recovery.divergence_Detection_enable) {
        return false;
    }
    return summary_.final_cost >
           config.recovery.divergence_Detection_max_chi_squared;
}

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
