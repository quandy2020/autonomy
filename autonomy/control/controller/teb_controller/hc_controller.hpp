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

#include <chrono>
#include <math.h>

#include <algorithm>
#include <functional>
#include <iterator>
#include <memory>
#include <random>
#include <vector>

#include "autonomy/control/common/controller_interface.hpp"
#include "autonomy/control/controller/teb_controller/grpah/equivalence.hpp"
#include "autonomy/control/controller/teb_controller/grpah/graph_lr_keypoint.hpp"
#include "autonomy/control/controller/teb_controller/grpah/graph_prob_roadmap.hpp"
#include "autonomy/control/controller/teb_controller/grpah/signature_2d.hpp"
#include "autonomy/control/controller/teb_controller/grpah/signature_3d.hpp"
#include "autonomy/control/controller/teb_controller/geometry/obstacle.hpp"
#include "autonomy/control/controller/teb_controller/trajectory_planner.hpp"
#include "autonomy/control/controller/teb_controller/footprint.hpp"
#include "autonomy/control/controller/teb_controller/config.hpp"
#include "autonomy/control/controller/teb_controller/planner_visualization.hpp"

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

// Inline helper for calculateHomotopySignature() over TEB pose samples.
inline std::complex<long double> GetCplxFromPose2D(const Pose2D& pose) {
    return std::complex<long double>(pose.x, pose.y);
}

// Inline function used for calculateHomotopySignature() in combination with
// autonomy::commsgs::geometry_msgs::PoseStamped
inline std::complex<long double> GetCplxFromMsgPoseStamped(
    const autonomy::commsgs::geometry_msgs::PoseStamped& pose) {
    return std::complex<long double>(pose.pose.position.x,
                                     pose.pose.position.y);
};

/**
 * @brief Multi-trajectory homotopy class TEB controller
 */
class HcController
    : public autonomy::control::common::ControllerInterface
{
public:
    using EquivalenceContainer =
        std::vector<std::pair<EquivalencePtr, bool> >;

    /**
     * @brief Default constructor for HcController
     */
    HcController();

    /**
     * @brief Constructor for HcController
     * @param cfg Planner configuration
     * @param obstacles Obstacle container (optional)
     * @param visualization Visualization helper (optional)
     * @param via_points Via-point container (optional)
     */
    HcController(
        const TimedElasticBandConfig& cfg, ObstContainer* obstacles = nullptr,
        PlannerVisualizationPtr visualization = PlannerVisualizationPtr(),
        const ViaPointContainer* via_points = nullptr);

    /**
     * @brief Destructor for HcController
     */
    virtual ~HcController();

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

    /**
     * @brief Initialize planner with configuration and dependencies
     */
    void Initialize(const TimedElasticBandConfig& cfg,
                    ObstContainer* obstacles = nullptr,
                    PlannerVisualizationPtr visualization = PlannerVisualizationPtr(),
                    const ViaPointContainer* via_points = nullptr);

    /**
     * @brief Plan and optimize trajectories from a pose sequence
     */
    virtual bool Plan(
        const std::vector<autonomy::commsgs::geometry_msgs::PoseStamped>&
            initial_plan,
        const autonomy::commsgs::geometry_msgs::Twist* start_vel = nullptr,
        bool free_goal_velocity = false);

    // tf2 doesn't have tf::Pose
    //  virtual bool Plan(const tf::Pose& start, const tf::Pose& goal, const
    //  autonomy::commsgs::geometry_msgs::Twist* start_vel = NULL, bool
    //  free_goal_velocity=false);

    /**
     * @brief Plan and optimize trajectories between start and goal
     */
    virtual bool Plan(
        const Pose2D& start, const Pose2D& goal,
        const autonomy::commsgs::geometry_msgs::Twist* start_vel = nullptr,
        bool free_goal_velocity = false);

    virtual bool GetVelocityCommand(double& vx, double& vy, double& omega,
                                    int look_ahead_poses) const;

    TrajectoryPlannerPtr BestTeb() const {
        return tebs_.empty()       ? TrajectoryPlannerPtr()
               : tebs_.size() == 1 ? tebs_.front()
                                   : best_teb_;
    }

    virtual bool IsTrajectoryFeasible(
        map::costmap_2d::Costmap2D* costmap,
        const std::vector<autonomy::commsgs::geometry_msgs::Point>&
            footprint_spec,
        double inscribed_radius = 0.0, double circumscribed_radius = 0.0,
        int look_ahead_idx = -1,
        double feasibility_check_lookahead_distance = -1.0);

    /**
     * @brief Find the best TEB among homotopy candidates
     */
    TrajectoryPlannerPtr FindBestTeb();

    /**
     * @brief Remove a TEB from the candidate container
     */
    TrajectoryPlannerContainer::iterator RemoveTeb(TrajectoryPlannerPtr& teb);
    void SetVisualization(const PlannerVisualizationPtr& visualization);

    virtual void Visualize();

    void ExploreEquivalenceClassesAndInitTebs(
        const Pose2D& start, const Pose2D& goal, double dist_to_obst,
        const autonomy::commsgs::geometry_msgs::Twist* start_vel,
        bool free_goal_velocity = false);

    template <typename BidirIter, typename Fun>
    TrajectoryPlannerPtr AddAndInitNewTeb(
        BidirIter path_start, BidirIter path_end, Fun fun_position,
        double start_orientation, double goal_orientation,
        const autonomy::commsgs::geometry_msgs::Twist* start_velocity,
        bool free_goal_velocity = false);

    TrajectoryPlannerPtr AddAndInitNewTeb(
        const Pose2D& start, const Pose2D& goal,
        const autonomy::commsgs::geometry_msgs::Twist* start_velocity,
        bool free_goal_velocity = false);

    TrajectoryPlannerPtr AddAndInitNewTeb(
        const std::vector<autonomy::commsgs::geometry_msgs::PoseStamped>&
            initial_plan,
        const autonomy::commsgs::geometry_msgs::Twist* start_velocity,
        bool free_goal_velocity = false);

    void UpdateAllTEBs(
        const Pose2D* start, const Pose2D* goal,
        const autonomy::commsgs::geometry_msgs::Twist* start_velocity);

    void OptimizeAllTEBs(int iter_innerloop, int iter_outerloop);

    // Otherwise the shared ptr is empty.
    TrajectoryPlannerPtr GetInitialPlanTEB();

    TrajectoryPlannerPtr SelectBestTeb();
    // Clear all previously found H-signatures, paths, tebs and the hcgraph.
    virtual void ClearPlanner() {
        ClearExplorationGraph();
        homotopy_classes_.clear();
        tebs_.clear();
        initial_plan_ = nullptr;
    }

    virtual void SetPreferredTurningDir(PreferredRotationDirection dir);

    template <typename BidirIter, typename Fun>
    EquivalencePtr CalculateEquivalence(
        BidirIter path_start, BidirIter path_end, Fun fun_cplx_point,
        const ObstContainer* obstacles = nullptr,
        std::optional<TimeDiffSequence::iterator> timediff_start = std::nullopt,
        std::optional<TimeDiffSequence::iterator> timediff_end = std::nullopt);

    const TrajectoryPlannerContainer& GetTrajectoryContainer() const {
        return tebs_;
    }

    // Returns true if current best planner indicates optimization divergence.
    bool HasDiverged() const;

    virtual void ComputeCurrentCost(std::vector<double>& cost,
                                    double obst_cost_Scale = 1.0,
                                    double viapoint_cost_Scale = 1.0,
                                    bool alternative_time_cost = false);

    inline static bool IsHomotopySignatureSimilar(const std::complex<long double>& h1,
                                           const std::complex<long double>& h2,
                                           double threshold) {
        double diff_real = std::abs(h2.real() - h1.real());
        double diff_imag = std::abs(h2.imag() - h1.imag());
        if (diff_real <= threshold && diff_imag <= threshold)
            return true;  // Found! Homotopy class already exists, therefore
                          // nothing added
        return false;
    }
    void DeletePlansDetouringBackwards(const double orient_threshold,
                                       const double len_orientation_vector);
    bool ComputeStartOrientation(const TrajectoryPlannerPtr Plan,
                                 const double len_orientation_vector,
                                 double& orientation);

    const TimedElasticBandConfig* config() const {
        return config_;
    }

    const ObstContainer* obstacles() const {
        return obstacles_;
    }

    bool isInitialized() const {
        return initialized_;
    }

    void ClearExplorationGraph() {
        if (graph_search_) {
            graph_search_->ClearExplorationGraph();
        }
    }

    /**
     * @brief Index of the best TEB in tebs_, or -1 if none
     */
    int BestTebIdx() const;

    bool AddEquivalenceIfNew(const EquivalencePtr& homotopy_class,
                                  bool lock = false);

    const EquivalenceContainer& GetEquivalenceClasses() const {
        return homotopy_classes_;
    }

    /**
     * @brief Check if homotopy_class matches the best TEB homotopy class
     */
    bool IsInBestTebClass(const EquivalencePtr& homotopy_class) const;

    int NumTebsInClass(const EquivalencePtr& homotopy_class) const;

    int NumTebsInBestTebClass() const;

    void RandomlyDropTebs();

protected:
    bool HasEquivalence(const EquivalencePtr& homotopy_class) const;

    void RenewAndAnalyzeOldTebs(bool delete_detours);

    void UpdateReferenceTrajectoryViaPoints(bool all_trajectories);
    // External configuration and inputs (not owned)
    const TimedElasticBandConfig*
        config_;  // Config class that stores and manages all related parameters
    ObstContainer*
        obstacles_;  // Store obstacles that are relevant for planning
    const ViaPointContainer*
        via_points_;  // Store the current list of via-points

    // Owned planner state
    PlannerVisualizationPtr visualization_;  // Instance of the visualization class
    TrajectoryPlannerPtr best_teb_;      // Store the current best teb.
    EquivalencePtr best_teb_homotopy_class_;  // Homotopy class of the current best teb
    RobotFootprintPtr robot_footprint_;

    const std::vector<autonomy::commsgs::geometry_msgs::PoseStamped>*
        initial_plan_;  // Store the initial Plan if available for a better
                        // trajectory initialization
    EquivalencePtr initial_plan_homotopy_class_;  // Homotopy class of the initial plan
    TrajectoryPlannerPtr
        initial_plan_teb_;  // Store pointer to the TEB related to the initial
                            // Plan (use method GetInitialPlanTEB() since it
                            // checks if initial_plan_teb_ is still included in

    TrajectoryPlannerContainer tebs_;  // Alternative local TEB planners per homotopy class

    EquivalenceContainer
        homotopy_classes_;  // Known homotopy classes (e.g. h-signatures) for duplicate detection.
                               // Second field: exclude from detour deletion when true.

    std::shared_ptr<GraphSearchInterface> graph_search_;

    std::default_random_engine random_;
    bool initialized_;  // Keeps track about the correct initialization of this
                        // class

    TrajectoryPlannerPtr last_best_teb_;  // Points to the Plan used in the
                                          // previous control cycle
    commsgs::planning_msgs::Path current_plan_;
    std::chrono::steady_clock::time_point last_homotopy_class_switching_time_;
};

template <typename BidirIter, typename Fun>
TrajectoryPlannerPtr HcController::AddAndInitNewTeb(
    BidirIter path_start, BidirIter path_end, Fun fun_position,
    double start_orientation, double goal_orientation,
    const autonomy::commsgs::geometry_msgs::Twist* start_velocity,
    bool free_goal_velocity) {
    if (tebs_.size() >= config_->homotopy.max_number_classes) {
        return TrajectoryPlannerPtr();
    }
    TrajectoryPlannerPtr candidate = TrajectoryPlannerPtr(
        new TrajectoryPlanner(*config_, obstacles_, visualization_));

    candidate->teb().InitTrajectoryToGoal(
        path_start, path_end, fun_position, config_->robot.max_velocity_x,
        config_->robot.max_angular_velocity, std::nullopt, std::nullopt,
        std::optional<double>(start_orientation),
        std::optional<double>(goal_orientation),
        config_->trajectory.min_samples,
        config_->trajectory.allow_init_with_backwards_motion);

    if (start_velocity) {
        candidate->SetVelocityStart(*start_velocity);
    }

    EquivalencePtr H = CalculateEquivalence(
        candidate->teb().Poses().begin(), candidate->teb().Poses().end(),
        GetCplxFromPose2D, obstacles_, candidate->teb().Timediffs().begin(),
        candidate->teb().Timediffs().end());

    if (free_goal_velocity) {
        candidate->SetVelocityGoalFree();
    }

    if (AddEquivalenceIfNew(H)) {
        tebs_.push_back(candidate);
        return tebs_.back();
    }
    return TrajectoryPlannerPtr();
}

template <typename BidirIter, typename Fun>
EquivalencePtr HcController::CalculateEquivalence(
    BidirIter path_start, BidirIter path_end, Fun fun_cplx_point,
    const ObstContainer* obstacles,
    std::optional<TimeDiffSequence::iterator> timediff_start,
    std::optional<TimeDiffSequence::iterator> timediff_end) {
    if (obstacles == nullptr || obstacles->empty()) {
        return EquivalencePtr();
    }

    if (config_->obstacles.include_dynamic_obstacles && timediff_start &&
        timediff_end) {
        auto signature = std::make_shared<HomotopySignature3d>(*config_);
        signature->CalculateHomotopySignature(path_start, path_end,
                                              fun_cplx_point, obstacles,
                                              timediff_start, timediff_end);
        return signature;
    }

    auto signature = std::make_shared<HomotopySignature>(*config_);
    signature->CalculateHomotopySignature(path_start, path_end, fun_cplx_point,
                                          obstacles);
    return signature;
}

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
