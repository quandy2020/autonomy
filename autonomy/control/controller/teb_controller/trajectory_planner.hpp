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

#include <limits.h>
#include <math.h>

#include "autonomy/control/common/controller_interface.hpp"
#include "autonomy/control/controller/teb_controller/footprint.hpp"
#include "autonomy/control/controller/teb_controller/config.hpp"
#include "autonomy/control/controller/teb_controller/cost_functions/optimization_problem.hpp"
#include "autonomy/control/controller/teb_controller/timed_elastic_band.hpp"

namespace autonomy {
namespace map {
namespace costmap_2d {
class Costmap2D;
}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy

namespace autonomy {
namespace control {
namespace teb_controller {

using ViaPointContainer = PointContainer;

struct TrajectoryOptionsPoint {
    autonomy::commsgs::geometry_msgs::Pose pose;
    autonomy::commsgs::geometry_msgs::Twist velocity;
    autonomy::commsgs::builtin_interfaces::Duration time_from_start;
};

/**
 * @brief Single-trajectory TEB optimizer
 */
class TrajectoryPlanner : public autonomy::control::common::ControllerInterface
{
public:
    /**
     * Define TrajectoryPlanner::SharedPtr type
     */
    AUTONOMY_SHARED_PTR_DEFINITIONS(TrajectoryPlanner);

    /**
     * @brief Default constructor for TrajectoryPlanner
     */
    TrajectoryPlanner();

    /**
     * @brief Constructor for TrajectoryPlanner
     * @param cfg Planner configuration
     * @param obstacles Obstacle container (optional)
     * @param visual Visualization helper (optional)
     * @param via_points Via-point container (optional)
     */
    TrajectoryPlanner(const TimedElasticBandConfig& cfg, ObstContainer* obstacles = nullptr,
                      const ViaPointContainer* via_points = nullptr);

    /**
     * @brief Destructor for TrajectoryPlanner
     */
    virtual ~TrajectoryPlanner();

    /**
     * @brief Compute velocity commands for the current control cycle
     */
    uint32 ComputeVelocityCommands(
        const commsgs::geometry_msgs::PoseStamped& pose,
        const commsgs::geometry_msgs::TwistStamped& velocity,
        commsgs::geometry_msgs::TwistStamped& cmd_vel,
        common::GoalChecker* goal_checker, std::string& message) override;

    /**
     * @brief Check if the goal is reached
     */
    bool IsGoalReached(double dist_tolerance, double angle_tolerance) override;

    /**
     * @brief Set the global plan to follow
     */
    void SetPlan(const commsgs::planning_msgs::Path& plan) override;

    /**
     * @brief Apply speed limit to the planner
     */
    void SetSpeedLimit(const double& speed_limit,
                       const bool& percentage) override;

    void Reset() override {
        ClearPlanner();
    }

    /**
     * @brief Initialize planner with configuration and dependencies
     */
    void Initialize(const TimedElasticBandConfig& cfg,
                    ObstContainer* obstacles = nullptr,
                    PlannerVisualizationPtr visual = PlannerVisualizationPtr(),
                    const ViaPointContainer* via_points = nullptr);

    /**
     * @brief Plan and optimize a trajectory from a pose sequence
     */
    virtual bool Plan(
        const std::vector<autonomy::commsgs::geometry_msgs::PoseStamped>&
            initial_plan,
        const autonomy::commsgs::geometry_msgs::Twist* start_vel = nullptr,
        bool free_goal_velocity = false);
    /**
     * @brief Plan and optimize a trajectory between start and goal
     */
    virtual bool Plan(
        const Pose2D& start, const Pose2D& goal,
        const autonomy::commsgs::geometry_msgs::Twist* start_vel = nullptr,
        bool free_goal_velocity = false);

    /**
     * @brief Get velocity command from the optimized trajectory
     */
    virtual bool GetVelocityCommand(double& vx, double& vy, double& omega,
                                    int look_ahead_poses) const;

    /**
     * @brief Run the main TEB optimization loop
     */
    bool OptimizeTimedElasticBand(int iterations_innerloop, int iterations_outerloop,
                     bool compute_cost_afterwards = false,
                     double obst_cost_Scale = 1.0,
                     double viapoint_cost_Scale = 1.0,
                     bool alternative_time_cost = false);
    void SetVelocityStart(
        const autonomy::commsgs::geometry_msgs::Twist& vel_start);

    void SetVelocityGoal(
        const autonomy::commsgs::geometry_msgs::Twist& vel_goal);

    void SetVelocityGoalFree() {
        vel_goal_.first = false;
    }
    void SetObstVector(ObstContainer* obst_vector) {
        obstacles_ = obst_vector;
    }

    const ObstContainer& GetObstVector() const {
        return *obstacles_;
    }
    void SetViaPoints(const ViaPointContainer* via_points) {
        via_points_ = via_points;
    }

    const ViaPointContainer& GetViaPoints() const {
        return *via_points_;
    }

    virtual void ClearPlanner() {
        ResetOptimization();
        teb_.ClearTimedElasticBand();
    }

    virtual void SetPreferredTurningDir(PreferredRotationDirection dir) {
        prefer_rotdir_ = dir;
    }


    bool IsOptimized() const {
        return optimized_;
    };

    bool HasDiverged() const;

    void ComputeCurrentCost(double obst_cost_Scale = 1.0,
                            double viapoint_cost_Scale = 1.0,
                            bool alternative_time_cost = false);

    virtual void ComputeCurrentCost(std::vector<double>& cost,
                                    double obst_cost_Scale = 1.0,
                                    double viapoint_cost_Scale = 1.0,
                                    bool alternative_time_cost = false) {
        ComputeCurrentCost(obst_cost_Scale, viapoint_cost_Scale,
                           alternative_time_cost);
        cost.push_back(GetCurrentCost());
    }

    double GetCurrentCost() const {
        return cost_;
    }

    inline void ExtractVelocity(const Pose2D& pose1, const Pose2D& pose2,
                                double dt, double& vx, double& vy,
                                double& omega) const;

    void GetVelocityProfile(
        std::vector<autonomy::commsgs::geometry_msgs::Twist>& velocity_profile)
        const;

    void GetFullTrajectory(std::vector<TrajectoryPoint>& trajectory) const;

    virtual bool IsTrajectoryFeasible(
        map::costmap_2d::Costmap2D* costmap,
        const std::vector<autonomy::commsgs::geometry_msgs::Point>&
            footprint_spec,
        double inscribed_radius = 0.0, double circumscribed_radius = 0.0,
        int look_ahead_idx = -1,
        double feasibility_check_lookahead_distance = -1);

    virtual bool IsPoseValid(
        autonomy::commsgs::geometry_msgs::Pose2D pose2d,
        map::costmap_2d::Costmap2D* costmap,
        const std::vector<autonomy::commsgs::geometry_msgs::Point>&
            footprint_spec);

protected:
    bool BuildOptimizationProblem(double weight_multiplier = 1.0);

    bool SolveOptimization(int no_iterations, bool clear_after = true);

    void ResetOptimization();

    void RegisterTrajectoryParameters();

    // External configuration and inputs (not owned)
    const TimedElasticBandConfig* config_;
    ObstContainer* obstacles_;
    const ViaPointContainer* via_points_;
    std::vector<ObstContainer> obstacles_per_vertex_;

    double cost_;
    PreferredRotationDirection prefer_rotdir_;

    // Owned planner state
    TimedElasticBand teb_;
    RobotFootprintPtr robot_footprint_;
    TebOptimizationProblem optimization_problem_;
    std::pair<bool, autonomy::commsgs::geometry_msgs::Twist> vel_start_;
    std::pair<bool, autonomy::commsgs::geometry_msgs::Twist> vel_goal_;

    bool initialized_;
    bool optimized_;
    commsgs::planning_msgs::Path current_plan_;
};

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
