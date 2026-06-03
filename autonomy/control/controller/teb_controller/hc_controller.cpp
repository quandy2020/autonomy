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

#include "autonomy/common/logging.hpp"
#include "autonomy/control/controller/teb_controller/hc_controller.hpp"

#include <limits>

namespace autonomy {
namespace control {
namespace teb_controller {

HcController::HcController()
    : config_(nullptr),
      obstacles_(nullptr),
      via_points_(nullptr),
      robot_footprint_(new PointRobotFootprint()),
      initial_plan_(nullptr),
      initialized_(false) {}

HcController::HcController(const TimedElasticBandConfig& cfg,
                                           ObstContainer* obstacles,
                                           PlannerVisualizationPtr visual,
                                           const ViaPointContainer* via_points)
    : initial_plan_(nullptr) {
    Initialize(cfg, obstacles, visual, via_points);
}

HcController::~HcController() {}

uint32 HcController::ComputeVelocityCommands(
    const commsgs::geometry_msgs::PoseStamped& pose,
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

bool HcController::IsGoalReached(double dist_tolerance,
                                         double angle_tolerance) {
    if (current_plan_.poses.empty()) {
        return false;
    }
    TrajectoryPlannerPtr best_teb = BestTeb();
    if (!best_teb || best_teb->teb().SizePoses() == 0) {
        return false;
    }

    const Pose2D& current_pose = best_teb->teb().BackPose();
    const Pose2D goal_pose = Pose2DFromPose(current_plan_.poses.back().pose);
    const double pos_error =
        Norm(Position(current_pose) - Position(goal_pose));
    const double yaw_error = std::fabs(
        autonomy::common::AngleDiff(current_pose.theta, goal_pose.theta));
    return pos_error <= dist_tolerance && yaw_error <= angle_tolerance;
}

void HcController::SetPlan(const commsgs::planning_msgs::Path& plan) {
    current_plan_ = plan;
}

void HcController::SetSpeedLimit(const double& speed_limit,
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

void HcController::Initialize(const TimedElasticBandConfig& cfg,
                                      ObstContainer* obstacles,
                                      PlannerVisualizationPtr visual,
                                      const ViaPointContainer* via_points) {
    config_ = &cfg;
    obstacles_ = obstacles;
    via_points_ = via_points;

    if (config_->homotopy.simple_exploration)
        graph_search_ = std::shared_ptr<GraphSearchInterface>(
            new LRKeyPointGraph(*config_, this));
    else
        graph_search_ = std::shared_ptr<GraphSearchInterface>(
            new ProbRoadmapGraph(*config_, this));

    std::random_device rd;
    random_.seed(rd());

    last_homotopy_class_switching_time_ = std::chrono::steady_clock::now();

    initialized_ = true;

    SetVisualization(visual);
}

void HcController::SetVisualization(
    const PlannerVisualizationPtr& visualization) {
    visualization_ = visualization;
}

bool HcController::Plan(
    const std::vector<autonomy::commsgs::geometry_msgs::PoseStamped>&
        initial_plan,
    const autonomy::commsgs::geometry_msgs::Twist* start_vel,
    bool free_goal_velocity) {
    TEB_ASSERT_MSG(initialized_, "Call Initialize() first.");

    // store initial Plan for further initializations (must be valid for the
    // lifetime of this object or ClearPlanner() is called!)
    initial_plan_ = &initial_plan;

    Pose2D start = Pose2DFromPose(initial_plan.front().pose);
    Pose2D goal = Pose2DFromPose(initial_plan.back().pose);

    return Plan(start, goal, start_vel, free_goal_velocity);
}

// tf2 doesn't have tf::Pose
// bool HcController::Plan(const tf::Pose& start, const tf::Pose& goal,
// const autonomy::commsgs::geometry_msgs::Twist* start_vel, bool free_goal_velocity)
//{
//  TEB_ASSERT_MSG(initialized_, "Call Initialize() first.");
//  Pose2D start_pose(start);
//  Pose2D goal_pose(goal);
//  return Plan(start_pose, goal_pose, start_vel, free_goal_velocity);
//}

bool HcController::Plan(
    const Pose2D& start, const Pose2D& goal,
    const autonomy::commsgs::geometry_msgs::Twist* start_vel,
    bool free_goal_velocity) {
    TEB_ASSERT_MSG(initialized_, "Call Initialize() first.");

    // Update old TEBs with new start, goal and velocity
    UpdateAllTEBs(&start, &goal, start_vel);

    // Init new TEBs based on newly explored homotopy classes
    ExploreEquivalenceClassesAndInitTebs(start, goal,
                                         config_->obstacles.min_obstacle_dist,
                                         start_vel, free_goal_velocity);
    // update via-points if activated
    UpdateReferenceTrajectoryViaPoints(config_->homotopy.via_points_all_candidates);
    // Optimize all trajectories in alternative homotopy classes
    OptimizeAllTEBs(config_->optimization.inner_iteration_count,
                    config_->optimization.outer_iteration_count);
    // Select which candidate (based on alternative homotopy classes) should be
    // used
    SelectBestTeb();

    initial_plan_ =
        nullptr;  // clear pointer to any previous initial Plan (any previous
                  // Plan is useless regarding the h-signature);
    return true;
}

bool HcController::GetVelocityCommand(double& vx, double& vy,
                                              double& omega,
                                              int look_ahead_poses) const {
    TrajectoryPlannerConstPtr best_teb = BestTeb();
    if (!best_teb) {
        vx = 0;
        vy = 0;
        omega = 0;
        return false;
    }

    return best_teb->GetVelocityCommand(vx, vy, omega, look_ahead_poses);
}

void HcController::Visualize() {
    if (visualization_) {
        // Visualize graph
        if (config_->homotopy.visualize_homotopy_graph && graph_search_)
            visualization_->PublishGraph(graph_search_->graph_);

        // Visualize active tebs as marker
        visualization_->PublishTrajectoryContainer(tebs_);

        // Visualize best teb and feedback message if desired
        TrajectoryPlannerConstPtr best_teb = BestTeb();
        if (best_teb) {
            visualization_->PublishLocalPlanAndPoses(best_teb->teb());

            if (best_teb->teb().SizePoses() >
                0)  // TODO maybe store current pose (start) within Plan method
                    // as class field.
                visualization_->PublishRobotFootprint(
                    best_teb->teb().Pose(0), *config_->robot_footprint);

            // feedback message
            if (config_->trajectory.publish_feedback) {
                int best_idx = BestTebIdx();
                if (best_idx >= 0)
                    visualization_->PublishFeedbackMessage(
                        tebs_, (unsigned int)best_idx, *obstacles_);
            }
        }
    } else
        ADEBUG << "Ignoring HcController::Visualize() call, since no "
                  "visualization class was instantiated before.";
}

bool HcController::HasEquivalence(
    const EquivalencePtr& homotopy_class) const {
    // iterate existing h-signatures and check if there is an existing
    // H-Signature similar the candidate
    for (const std::pair<EquivalencePtr, bool>& homotopy_class_entry :
         homotopy_classes_) {
        if (homotopy_class->IsEqual(*homotopy_class_entry.first))
            return true;  // Found! Homotopy class already exists, therefore
                          // nothing added
    }
    return false;
}

bool HcController::AddEquivalenceIfNew(
    const EquivalencePtr& homotopy_class, bool lock) {
    if (!homotopy_class)
        return false;

    if (!homotopy_class->IsValid()) {
        AWARN << "HcController: Ignoring invalid H-signature";
        return false;
    }

    if (HasEquivalence(homotopy_class)) {
        // Allow up to configured number of Tebs that are in the same homotopy
        // class as the current (best) Teb to avoid being stuck in a local
        // minimum
        if (!IsInBestTebClass(homotopy_class) ||
            NumTebsInBestTebClass() >=
                config_->homotopy.max_plans_per_homotopy_class)
            return false;
    }

    // Homotopy class not found -> Add to class-list, return that the
    // h-signature is new
    homotopy_classes_.emplace_back(homotopy_class, lock);
    return true;
}

void HcController::RenewAndAnalyzeOldTebs(bool delete_detours) {
    // clear old h-signatures (since they could be changed due to new obstacle
    // positions.
    homotopy_classes_.clear();

    // Adding the equivalence class of the latest best_teb_ first
    TrajectoryPlannerContainer::iterator it_best_teb =
        best_teb_ ? std::find(tebs_.begin(), tebs_.end(), best_teb_)
                  : tebs_.end();
    bool has_best_teb = it_best_teb != tebs_.end();
    if (has_best_teb) {
        std::iter_swap(tebs_.begin(),
                       it_best_teb);  // Putting the last best teb at the
                                      // beginning of the container
        best_teb_homotopy_class_ = CalculateEquivalence(
            best_teb_->teb().Poses().begin(), best_teb_->teb().Poses().end(),
            GetCplxFromPose2D, obstacles_,
            best_teb_->teb().Timediffs().begin(),
            best_teb_->teb().Timediffs().end());
        AddEquivalenceIfNew(best_teb_homotopy_class_);
    }
    // Collect h-signatures for all existing TEBs and store them together with
    // the corresponding iterator / pointer:
    //   using TebCandidateType = std::list<
    //   std::pair<TrajectoryPlannerContainer::iterator, std::complex<long double> >
    //   >; TebCandidateType teb_candidates;

    // get new homotopy classes and delete multiple TEBs per homotopy class.
    // Skips the best teb if available (added before).
    TrajectoryPlannerContainer::iterator it_teb =
        has_best_teb ? std::next(tebs_.begin(), 1) : tebs_.begin();
    while (it_teb != tebs_.end()) {
        // calculate equivalence class for the current candidate
        EquivalencePtr homotopy_class = CalculateEquivalence(
            it_teb->get()->teb().Poses().begin(),
            it_teb->get()->teb().Poses().end(), GetCplxFromPose2D,
            obstacles_, it_teb->get()->teb().Timediffs().begin(),
            it_teb->get()->teb().Timediffs().end());

        //     teb_candidates.push_back(std::make_pair(it_teb,H));

        // WORKAROUND until the commented code below works
        // Here we do not compare cost values. Just first come first serve...
        bool new_flag = AddEquivalenceIfNew(homotopy_class);
        if (!new_flag) {
            it_teb = tebs_.erase(it_teb);
            continue;
        }

        ++it_teb;
    }
    if (delete_detours)
        DeletePlansDetouringBackwards(
            config_->homotopy.detours_orientation_tolerance,
            config_->homotopy.length_start_orientation_vector);

    // Find multiple candidates and delete the one with higher cost
    // TODO: this code needs to be adpated. Erasing tebs from the teb container_
    // could make iteratores stored in the candidate list invalid!
    //   TebCandidateType::reverse_iterator cand_i = teb_candidates.rbegin();
    //   int test_idx = 0;
    //   while (cand_i != teb_candidates.rend())
    //   {
    //
    //     TebCandidateType::reverse_iterator cand_j =
    //     std::find_if(boost::next(cand_i),teb_candidates.rend(),
    //     boost::bind(compareH,_1,cand_i->second)); if (cand_j !=
    //     teb_candidates.rend() && cand_j != cand_i)
    //     {
    //         TrajectoryPlannerPtr pt1 = *(cand_j->first);
    //         TrajectoryPlannerPtr pt2 = *(cand_i->first);
    //         assert(pt1);
    //         assert(pt2);
    //       if ( cand_j->first->get()->GetCurrentCost().sum() >
    //       cand_i->first->get()->GetCurrentCost().sum() )
    //       {
    // 	// found one that has higher cost, therefore erase cand_j
    // 	tebs_.erase(cand_j->first);
    // 	teb_candidates.erase(cand_j);
    //       }
    //       else   // otherwise erase cand_i
    //       {
    // 	tebs_.erase(cand_i->first);
    // 	cand_i = teb_candidates.erase(cand_i);
    //       }
    //     }
    //     else
    //     {
    //         ROS_WARN_STREAM("increase cand_i");
    //         ++cand_i;
    //     }
    //   }

    // now add the h-signatures to the internal lookup-table (but only if there
    // is no existing duplicate)
    //   for (TebCandidateType::iterator cand=teb_candidates.begin();
    //   cand!=teb_candidates.end(); ++cand)
    //   {
    //     bool new_flag = addNewHomotopySignatureIfNew(cand->second,
    //     config_->homotopy.homotopy_signature_threshold); if (!new_flag)
    //     {
    // //       ROS_ERROR_STREAM("getAndFilterHomotopyOptionsTEB() - This
    // schould not be happen.");
    //       tebs_.erase(cand->first);
    //     }
    //   }
}

void HcController::UpdateReferenceTrajectoryViaPoints(
    bool all_trajectories) {
    if ((!all_trajectories && !initial_plan_) || !via_points_ ||
        via_points_->empty() || config_->optimization.weight_via_point <= 0)
        return;

    if (homotopy_classes_.size() < tebs_.size()) {
        AERROR << "HcController::updateReferenceTrajectoryWithViaPoints(): "
                  "Number of h-signatures does not match number of trajectories.";
        return;
    }

    if (all_trajectories) {
        // enable via-points for all tebs
        for (std::size_t i = 0; i < homotopy_classes_.size(); ++i) {
            tebs_[i]->SetViaPoints(via_points_);
        }
    } else {
        // enable via-points for teb in the same hommotopy class as the
        // initial_plan and deactivate it for all other ones
        for (std::size_t i = 0; i < homotopy_classes_.size(); ++i) {
            if (initial_plan_homotopy_class_->IsEqual(*homotopy_classes_[i].first))
                tebs_[i]->SetViaPoints(via_points_);
            else
                tebs_[i]->SetViaPoints(nullptr);
        }
    }
}

void HcController::ExploreEquivalenceClassesAndInitTebs(
    const Pose2D& start, const Pose2D& goal, double dist_to_obst,
    const autonomy::commsgs::geometry_msgs::Twist* start_vel,
    bool free_goal_velocity) {
    // first process old trajectories
    RenewAndAnalyzeOldTebs(config_->homotopy.delete_detours_backwards);
    RandomlyDropTebs();

    // inject initial Plan if available and not yet captured
    if (initial_plan_) {
        initial_plan_teb_ =
            AddAndInitNewTeb(*initial_plan_, start_vel, free_goal_velocity);
    } else {
        initial_plan_teb_.reset();
        initial_plan_teb_ =
            GetInitialPlanTEB();  // this method searches for
                                  // initial_plan_homotopy_class_ in the teb container
                                  // (-> if !initial_plan_teb_)
    }

    // now explore new homotopy classes and Initialize tebs if new ones are
    // found. The appropriate createGraph method is chosen via polymorphism.
    graph_search_->CreateGraph(start, goal, dist_to_obst,
                               config_->homotopy.obstacle_heading_threshold, start_vel,
                               free_goal_velocity);
}

TrajectoryPlannerPtr HcController::AddAndInitNewTeb(
    const Pose2D& start, const Pose2D& goal,
    const autonomy::commsgs::geometry_msgs::Twist* start_velocity,
    bool free_goal_velocity) {
    if (tebs_.size() >= config_->homotopy.max_number_classes)
        return TrajectoryPlannerPtr();
    TrajectoryPlannerPtr candidate = TrajectoryPlannerPtr(
        new TrajectoryPlanner(*config_, obstacles_, visualization_));

    candidate->teb().InitTrajectoryToGoal(
        start, goal, 0, config_->robot.max_velocity_x, config_->trajectory.min_samples,
        config_->trajectory.allow_init_with_backwards_motion);

    if (start_velocity)
        candidate->SetVelocityStart(*start_velocity);

    EquivalencePtr H = CalculateEquivalence(
        candidate->teb().Poses().begin(), candidate->teb().Poses().end(),
        GetCplxFromPose2D, obstacles_,
        candidate->teb().Timediffs().begin(),
        candidate->teb().Timediffs().end());

    if (free_goal_velocity)
        candidate->SetVelocityGoalFree();

    if (AddEquivalenceIfNew(H)) {
        tebs_.push_back(candidate);
        return tebs_.back();
    }

    // If the candidate constitutes no new equivalence class, return a null
    // pointer
    return TrajectoryPlannerPtr();
}

bool HcController::IsInBestTebClass(
    const EquivalencePtr& homotopy_class) const {
    bool answer = false;
    if (best_teb_homotopy_class_)
        answer = best_teb_homotopy_class_->IsEqual(*homotopy_class);
    return answer;
}

int HcController::NumTebsInClass(
    const EquivalencePtr& homotopy_class) const {
    int count = 0;
    for (const std::pair<EquivalencePtr, bool>& homotopy_class_entry :
         homotopy_classes_) {
        if (homotopy_class->IsEqual(*homotopy_class_entry.first))
            ++count;
    }
    return count;
}

int HcController::NumTebsInBestTebClass() const {
    int count = 0;
    if (best_teb_homotopy_class_)
        count = NumTebsInClass(best_teb_homotopy_class_);
    return count;
}

TrajectoryPlannerPtr HcController::AddAndInitNewTeb(
    const std::vector<autonomy::commsgs::geometry_msgs::PoseStamped>&
        initial_plan,
    const autonomy::commsgs::geometry_msgs::Twist* start_velocity,
    bool free_goal_velocity) {
    if (tebs_.size() >= config_->homotopy.max_number_classes)
        return TrajectoryPlannerPtr();
    TrajectoryPlannerPtr candidate = TrajectoryPlannerPtr(
        new TrajectoryPlanner(*config_, obstacles_, visualization_));

    candidate->teb().InitTrajectoryToGoal(
        initial_plan, config_->robot.max_velocity_x, config_->robot.max_angular_velocity,
        config_->trajectory.global_plan_overwrite_orientation,
        config_->trajectory.min_samples,
        config_->trajectory.allow_init_with_backwards_motion);

    if (start_velocity)
        candidate->SetVelocityStart(*start_velocity);

    if (free_goal_velocity)
        candidate->SetVelocityGoalFree();

    // store the h signature of the initial Plan to enable searching a matching
    // teb later.
    initial_plan_homotopy_class_ = CalculateEquivalence(
        candidate->teb().Poses().begin(), candidate->teb().Poses().end(),
        GetCplxFromPose2D, obstacles_,
        candidate->teb().Timediffs().begin(),
        candidate->teb().Timediffs().end());

    if (AddEquivalenceIfNew(initial_plan_homotopy_class_,
                                 true))  // also prevent candidate from deletion
    {
        tebs_.push_back(candidate);
        return tebs_.back();
    }

    // If the candidate constitutes no new equivalence class, return a null
    // pointer
    return TrajectoryPlannerPtr();
}

void HcController::UpdateAllTEBs(
    const Pose2D* start, const Pose2D* goal,
    const autonomy::commsgs::geometry_msgs::Twist* start_velocity) {
    // If new goal is too far away, clear all existing trajectories to let them
    // reinitialize later. Since all Tebs are sharing the same fixed goal pose,
    // just take the first candidate:
    if (!tebs_.empty() &&
        (Norm(Position(*goal) - Position(tebs_.front()->teb().BackPose())) >=
             config_->trajectory.force_reinit_new_goal_dist ||
         fabs(autonomy::common::AngleDiff(
                  tebs_.front()->teb().BackPose().theta, goal->theta)) >=
             config_->trajectory.force_reinit_new_goal_angular)) {
        ADEBUG << "New goal: distance to existing goal is higher than the "
                  "specified threshold. Reinitalizing trajectories.";
        tebs_.clear();
        homotopy_classes_.clear();
    }

    // hot-start from previous solutions
    for (TrajectoryPlannerContainer::iterator it_teb = tebs_.begin();
         it_teb != tebs_.end(); ++it_teb) {
        it_teb->get()->teb().UpdateAndPruneTEB(*start, *goal);
        if (start_velocity)
            it_teb->get()->SetVelocityStart(*start_velocity);
    }
}

void HcController::OptimizeAllTEBs(int iter_innerloop,
                                           int iter_outerloop) {
    // optimize TEBs in parallel since they are independend of each other
    if (config_->homotopy.enable_multithreading) {
        std::vector<std::thread> teb_threads;

        for (TrajectoryPlannerContainer::iterator it_teb = tebs_.begin();
             it_teb != tebs_.end(); ++it_teb) {
            auto functor = [&, it_teb]() {
                it_teb->get()->OptimizeTimedElasticBand(
                    iter_innerloop, iter_outerloop, true,
                    config_->homotopy.selection_obstacle_cost_scale,
                    config_->homotopy.selection_via_point_cost_scale,
                    config_->homotopy.selection_alternative_time_cost);
            };

            teb_threads.emplace_back(functor);
        }

        for (auto& thread : teb_threads) {
            if (thread.joinable()) {
                thread.join();
            }
        }
    } else {
        for (TrajectoryPlannerContainer::iterator it_teb = tebs_.begin();
             it_teb != tebs_.end(); ++it_teb) {
            it_teb->get()->OptimizeTimedElasticBand(
                iter_innerloop, iter_outerloop, true,
                config_->homotopy.selection_obstacle_cost_scale,
                config_->homotopy.selection_via_point_cost_scale,
                config_->hcp
                    .selection_alternative_time_cost);  // compute cost as well
                                                        // inside OptimizeTimedElasticBand
                                                        // (last argument =
                                                        // true)
        }
    }
}

TrajectoryPlannerPtr HcController::GetInitialPlanTEB() {
    // first check stored teb object
    if (initial_plan_teb_) {
        // check if the teb is still part of the teb container
        if (std::find(tebs_.begin(), tebs_.end(), initial_plan_teb_) !=
            tebs_.end())
            return initial_plan_teb_;
        else {
            initial_plan_teb_.reset();  // reset pointer for next call
            ADEBUG << "initial teb not found, trying to find a match according "
                      "to the cached equivalence class";
        }
    }

    // reset the locked state for equivalence classes // TODO: this might be
    // adapted if not only the Plan containing the initial Plan is locked!
    for (int i = 0; i < homotopy_classes_.size(); ++i) {
        homotopy_classes_[i].second = false;
    }

    // otherwise check if the stored reference equivalence class exist in the
    // list of known classes
    if (initial_plan_homotopy_class_ && initial_plan_homotopy_class_->IsValid()) {
        if (homotopy_classes_.size() == tebs_.size()) {
            for (int i = 0; i < homotopy_classes_.size(); ++i) {
                if (homotopy_classes_[i].first->IsEqual(
                        *initial_plan_homotopy_class_)) {
                    homotopy_classes_[i].second = true;
                    return tebs_[i];
                }
            }
        } else
            AERROR << "HcController::GetInitialPlanTEB(): number of "
                      "equivalence classes (" << homotopy_classes_.size()
                   << ") and number of trajectories (" << tebs_.size()
                   << ") does not match.";
    } else
        ADEBUG << "HcController::GetInitialPlanTEB(): initial TEB not "
                  "found in the set of available trajectories.";

    return TrajectoryPlannerPtr();
}

void HcController::RandomlyDropTebs() {
    if (config_->homotopy.selection_dropping_probability == 0.0) {
        return;
    }
    // interate both vectors in parallel
    auto it_homotopy_class = homotopy_classes_.begin();
    auto it_teb = tebs_.begin();
    while (it_teb != tebs_.end() && it_homotopy_class != homotopy_classes_.end()) {
        if (it_teb->get() != best_teb_.get()  // Always preserve the "best" teb
            && (random_() <=
                config_->homotopy.selection_dropping_probability * random_.max())) {
            it_teb = tebs_.erase(it_teb);
            it_homotopy_class = homotopy_classes_.erase(it_homotopy_class);
        } else {
            ++it_teb;
            ++it_homotopy_class;
        }
    }
}

TrajectoryPlannerPtr HcController::SelectBestTeb() {
    double min_cost = std::numeric_limits<double>::max();  // maximum cost
    double min_cost_last_best = std::numeric_limits<double>::max();
    double min_cost_initial_plan_teb = std::numeric_limits<double>::max();
    TrajectoryPlannerPtr initial_plan_teb = GetInitialPlanTEB();

    // check if last best_teb is still a valid candidate
    if (best_teb_ &&
        std::find(tebs_.begin(), tebs_.end(), best_teb_) != tebs_.end()) {
        // get cost of this candidate
        min_cost_last_best =
            best_teb_->GetCurrentCost() *
            config_->homotopy.selection_cost_hysteresis;  // small hysteresis
        last_best_teb_ = best_teb_;
    } else {
        last_best_teb_.reset();
    }

    if (initial_plan_teb)  // the validity was already checked in
                           // GetInitialPlanTEB()
    {
        // get cost of this candidate
        min_cost_initial_plan_teb =
            initial_plan_teb->GetCurrentCost() *
            config_->homotopy.selection_prefer_initial_plan;  // small hysteresis
    }

    best_teb_.reset();  // reset pointer

    for (TrajectoryPlannerContainer::iterator it_teb = tebs_.begin();
         it_teb != tebs_.end(); ++it_teb) {
        // check if the related TEB leaves the local costmap region
        //      if (tebs_.size()>1 &&
        //      !(*it_teb)->teb().IsTrajectoryInsideRegion(20, -1, 1))
        //      {
        //          ROS_INFO("HcController::SelectBestTeb(): skipping
        //          trajectories that are not inside the local costmap");
        //          continue;
        //      }

        double teb_cost;

        if (*it_teb == last_best_teb_)
            teb_cost = min_cost_last_best;  // skip already known cost value of
                                            // the last best_teb
        else if (*it_teb == initial_plan_teb)
            teb_cost = min_cost_initial_plan_teb;
        else
            teb_cost = it_teb->get()->GetCurrentCost();

        if (teb_cost < min_cost) {
            // check if this candidate is currently not selected
            best_teb_ = *it_teb;
            min_cost = teb_cost;
        }
    }

    // in case we haven't found any teb due to some previous checks, investigate
    // list again
    //   if (!best_teb_ && !tebs_.empty())
    //   {
    //       LOG_DEBUG(runtime::get_logger("teb_local_planner"), "all " <<
    //       tebs_.size() << " tebs rejected previously"); if (tebs_.size()==1)
    //         best_teb_ = tebs_.front();
    //       else // if multiple TEBs are available:
    //       {
    //           // try to use the one that relates to the initial Plan
    //           TrajectoryPlannerPtr initial_plan_teb = GetInitialPlanTEB();
    //           if (initial_plan_teb)
    //               best_teb_ = initial_plan_teb;
    //           else
    //           {
    //              // now compute the cost for the rest (we haven't computed it
    //              before) for (TrajectoryPlannerContainer::iterator it_teb =
    //              tebs_.begin(); it_teb != tebs_.end(); ++it_teb)
    //              {
    //                 double teb_cost = it_teb->get()->GetCurrentCost();
    //                 if (teb_cost < min_cost)
    //                 {
    //                     // check if this candidate is currently not selected
    //                     best_teb_ = *it_teb;
    //                     min_cost = teb_cost;
    //                 }
    //              }
    //           }
    //       }
    //   }

    // check if we are allowed to change
    if (last_best_teb_ && best_teb_ != last_best_teb_) {
        const auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now - last_homotopy_class_switching_time_)
                .count() > config_->homotopy.switching_blocking_period) {
            last_homotopy_class_switching_time_ = now;
        } else {
            ADEBUG << "HcController::SelectBestTeb(): Switching equivalence "
                      "classes blocked (check parameter switching_blocking_period).";
            // block switching, so revert best_teb_
            best_teb_ = last_best_teb_;
        }
    }

    return best_teb_;
}

int HcController::BestTebIdx() const {
    if (tebs_.size() == 1)
        return 0;

    if (!best_teb_)
        return -1;

    int idx = 0;
    for (TrajectoryPlannerContainer::const_iterator it_teb = tebs_.begin();
         it_teb != tebs_.end(); ++it_teb, ++idx) {
        if (*it_teb == best_teb_)
            return idx;
    }
    return -1;
}

bool HcController::IsTrajectoryFeasible(
    map::costmap_2d::Costmap2D* costmap,
    const std::vector<autonomy::commsgs::geometry_msgs::Point>& footprint_spec,
    double inscribed_radius, double circumscribed_radius, int look_ahead_idx,
    double feasibility_check_lookahead_distance) {
    bool feasible = false;
    while (!feasible && tebs_.size() > 0) {
        TrajectoryPlannerPtr best = FindBestTeb();
        if (!best) {
            AERROR << "Couldn't retrieve the best Plan";
            return false;
        }
        feasible = best->IsTrajectoryFeasible(
            costmap, footprint_spec, inscribed_radius,
            circumscribed_radius, look_ahead_idx,
            feasibility_check_lookahead_distance);
        if (!feasible) {
            RemoveTeb(best);
            if (last_best_teb_ &&
                (last_best_teb_ == best))  // Same Plan as before.
                return feasible;  // Not failing could result in oscillations
                                  // between trajectories.
        }
    }
    return feasible;
}

TrajectoryPlannerPtr HcController::FindBestTeb() {
    if (tebs_.empty())
        return TrajectoryPlannerPtr();
    if (!best_teb_ ||
        std::find(tebs_.begin(), tebs_.end(), best_teb_) == tebs_.end())
        best_teb_ = SelectBestTeb();
    return best_teb_;
}

TrajectoryPlannerContainer::iterator HcController::RemoveTeb(
    TrajectoryPlannerPtr& teb) {
    TrajectoryPlannerContainer::iterator return_iterator = tebs_.end();
    if (homotopy_classes_.size() != tebs_.size()) {
        AERROR << "RemoveTeb: size of eq classes != size of tebs";
        return return_iterator;
    }
    auto it_homotopy_classes = homotopy_classes_.begin();
    for (auto it = tebs_.begin(); it != tebs_.end(); ++it) {
        if (*it == teb) {
            return_iterator = tebs_.erase(it);
            homotopy_classes_.erase(it_homotopy_classes);
            break;
        }
        ++it_homotopy_classes;
    }
    return return_iterator;
}

void HcController::SetPreferredTurningDir(PreferredRotationDirection dir) {
    // set preferred turning dir for all TEBs
    for (TrajectoryPlannerContainer::const_iterator it_teb = tebs_.begin();
         it_teb != tebs_.end(); ++it_teb) {
        (*it_teb)->SetPreferredTurningDir(dir);
    }
}

bool HcController::HasDiverged() const {
    // Early return if there is no best trajectory initialized
    if (!best_teb_)
        return false;

    return best_teb_->HasDiverged();
}

void HcController::ComputeCurrentCost(std::vector<double>& cost,
                                              double obst_cost_Scale,
                                              double viapoint_cost_Scale,
                                              bool alternative_time_cost) {
    for (TrajectoryPlannerContainer::iterator it_teb = tebs_.begin();
         it_teb != tebs_.end(); ++it_teb) {
        it_teb->get()->ComputeCurrentCost(
            cost, obst_cost_Scale, viapoint_cost_Scale, alternative_time_cost);
    }
}

void HcController::DeletePlansDetouringBackwards(
    const double orient_threshold, const double len_orientation_vector) {
    if (tebs_.size() < 2 || !best_teb_ ||
        std::find(tebs_.begin(), tebs_.end(), best_teb_) == tebs_.end() ||
        best_teb_->teb().SizePoses() < 2) {
        return;  // A moving direction wasn't chosen yet
    }
    double current_movement_orientation;
    double best_plan_duration =
        std::max(best_teb_->teb().GetSumOfAllTimeDiffs(), 1.0);
    if (!ComputeStartOrientation(best_teb_, len_orientation_vector,
                                 current_movement_orientation))
        return;  // The Plan is shorter than len_orientation_vector
    for (auto it_teb = tebs_.begin(); it_teb != tebs_.end();) {
        if (*it_teb ==
            best_teb_)  // The current Plan should not be considered a detour
        {
            ++it_teb;
            continue;
        }
        if ((*it_teb)->teb().SizePoses() < 2) {
            ADEBUG << "Discarding a Plan with less than 2 Poses";
            it_teb = RemoveTeb(*it_teb);
            continue;
        }
        double plan_orientation;
        if (!ComputeStartOrientation(*it_teb, len_orientation_vector,
                                     plan_orientation)) {
            ADEBUG << "Failed to compute the start orientation for one of the "
                      "tebs, likely close to the target";
            it_teb = RemoveTeb(*it_teb);
            continue;
        }
        if (fabs(autonomy::common::NormalizeAngle(plan_orientation -
                                      current_movement_orientation)) >
            orient_threshold) {
            it_teb = RemoveTeb(*it_teb);  // Plan detouring backwards
            continue;
        }
        if (!it_teb->get()->isOptimized()) {
            ADEBUG << "Removing a teb because it's not optimized";
            it_teb = RemoveTeb(*it_teb);  // Deletes tebs that cannot be
                                          // optimized (last optim call failed)
            continue;
        }
        if (it_teb->get()->teb().GetSumOfAllTimeDiffs() / best_plan_duration >
            config_->homotopy.max_ratio_detours_duration_best_duration) {
            ADEBUG << "Removing a teb because it's duration is much longer "
                      "than that of the best teb";
            it_teb = RemoveTeb(*it_teb);
            continue;
        }
        ++it_teb;
    }
}

bool HcController::ComputeStartOrientation(
    const TrajectoryPlannerPtr Plan, const double len_orientation_vector,
    double& orientation) {
    const Pose2D& start_pose = Plan->teb().Pose(0);
    bool second_pose_found = false;
    Point start_vector;
    for (const Pose2D& pose : Plan->teb().Poses()) {
        start_vector = Position(start_pose) - Position(pose);
        if (Norm(start_vector) > len_orientation_vector) {
            second_pose_found = true;
            break;
        }
    }
    if (!second_pose_found)  // The current Plan is too short to make
                             // assumptions on the start orientation
        return false;
    orientation = std::atan2(start_vector.y, start_vector.x);
    return true;
}

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
