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

#include <ceres/ceres.h>
#include <vector>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/control/controller/teb_controller/geometry/obstacle.hpp"
#include "autonomy/control/controller/teb_controller/footprint.hpp"
#include "autonomy/control/controller/teb_controller/config.hpp"
#include "autonomy/control/controller/teb_controller/timed_elastic_band.hpp"

namespace autonomy {
namespace control {
namespace teb_controller {

/**
 * @brief Residual category for weighted cost breakdown after optimization
 */
enum class TebResidualKind {
    Other,
    Obstacle,
    ViaPoint,
    TimeOptimal,
};

/**
 * @brief Metadata stored for one registered Ceres residual block
 */
struct TebResidualMeta {
    TebResidualKind kind = TebResidualKind::Other;
    int num_residuals = 0;
};

/**
 * @brief Assembles, solves, and evaluates the Ceres optimization problem for
 * TEB
 *
 * Holds the Ceres problem for one planner instance. Call SetBuildContext()
 * before Build(). The problem must be empty; call Clear() after a prior Solve()
 * when rebuilding.
 */
class TebOptimizationProblem
{
public:
    TebOptimizationProblem() = default;

    /**
     * @brief Binds non-owning planner state used when adding residuals
     *
     * @param config TEB configuration; must outlive this object until Build()
     * finishes
     * @param timed_elastic_band Timed elastic band to optimize
     * @param obstacles Obstacle list, or nullptr if none
     * @param via_points Optional via points, or nullptr
     * @param robot_model Footprint model for distance queries
     * @param obstacles_per_vertex Per-pose obstacle association storage
     * @param preferred_rotation_direction Preferred initial rotation direction
     * @param velocity_start Optional fixed start velocity; first field is
     * enable flag
     * @param velocity_goal Optional fixed goal velocity; first field is enable
     * flag
     */
    void SetBuildContext(
        const TimedElasticBandConfig& config, TimedElasticBand& timed_elastic_band,
        ObstContainer* obstacles, const ViaPointContainer* via_points,
        const RobotFootprint* robot_model,
        std::vector<ObstContainer>& obstacles_per_vertex,
        PreferredRotationDirection preferred_rotation_direction,
        const std::pair<bool, commsgs::geometry_msgs::Twist>& velocity_start,
        const std::pair<bool, commsgs::geometry_msgs::Twist>& velocity_goal);

    /**
     * @brief Adds all cost residuals and fixes constant parameter blocks
     *
     * @param weight_multiplier Scales obstacle-related weights for outer loops
     * @return bool False if build context was not set or the problem is
     * non-empty
     */
    bool Build(double weight_multiplier = 1.0);

    /**
     * @brief Returns the mutable Ceres problem
     *
     * Valid for the lifetime of this object.
     * @return ceres::Problem& Reference to the internal Ceres problem
     */
    ceres::Problem& Problem() {
        return problem_;
    }

    /**
     * @brief Returns the const Ceres problem
     *
     * @return const ceres::Problem& Reference to the internal Ceres problem
     */
    const ceres::Problem& Problem() const {
        return problem_;
    }

    /**
     * @brief Returns the solver summary from the last successful Solve()
     *
     * @return const ceres::Solver::Summary& Last Ceres solver summary
     */
    const ceres::Solver::Summary& Summary() const {
        return summary_;
    }

    /**
     * @brief Resets the problem, summary, and residual bookkeeping
     */
    void Clear();

    /**
     * @brief Runs Ceres minimization on the current problem
     *
     * @param max_iterations Ceres iteration limit
     * @param verbose If true, prints progress to stdout
     * @return bool True if Ceres reports a usable solution
     */
    bool Solve(int max_iterations, bool verbose);

    /**
     * @brief Sums squared residuals with optional per-category scaling
     *
     * @param obstacle_cost_scale Multiplier for obstacle residuals
     * @param via_point_cost_scale Multiplier for via-point residuals
     * @param skip_time_optimal If true, excludes time-optimal residuals
     * @return double Total cost, or 0.0 if evaluation fails
     */
    double EvaluateTotalCost(double obstacle_cost_scale = 1.0,
                             double via_point_cost_scale = 1.0,
                             bool skip_time_optimal = false) const;

    /**
     * @brief Checks whether the last solve indicates divergence
     *
     * @param config Supplies divergence detection thresholds
     * @return bool True if final chi-squared cost exceeds the configured limit
     */
    bool HasDiverged(const TimedElasticBandConfig& config) const;

private:
    void PrepareObstacleAssociation();

    void AddObstacleResiduals(double weight_multiplier);
    void AddObstacleResidualsLegacy(double weight_multiplier);
    void AddDynamicObstacleResiduals(double weight_multiplier);
    void AddViaPointResiduals();
    void AddVelocityResiduals();
    void AddAccelerationResiduals();
    void AddTimeOptimalResiduals();
    void AddShortestPathResiduals();
    void AddDiffDriveKinematicsResiduals();
    void AddCarlikeKinematicsResiduals();
    void AddPreferredRotationDirectionResiduals();
    void AddVelocityObstacleRatioResiduals();

    void RegisterResidualBlock(ceres::ResidualBlockId id, TebResidualKind kind);

    ceres::Problem problem_;
    ceres::Solver::Summary summary_;
    std::vector<TebResidualMeta> residual_meta_;
    std::vector<ceres::ResidualBlockId> residual_block_ids_;

    const TimedElasticBandConfig* config_ = nullptr;
    TimedElasticBand* teb_ = nullptr;
    ObstContainer* obstacles_ = nullptr;
    const ViaPointContainer* via_points_ = nullptr;
    const RobotFootprint* robot_model_ = nullptr;
    std::vector<ObstContainer>* obstacles_per_vertex_ = nullptr;
    PreferredRotationDirection prefer_rotdir_ = PreferredRotationDirection::none;
    std::pair<bool, commsgs::geometry_msgs::Twist> vel_start_;
    std::pair<bool, commsgs::geometry_msgs::Twist> vel_goal_;
};

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
