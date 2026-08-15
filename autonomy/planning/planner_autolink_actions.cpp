/*
 * Copyright 2026 The Openbot Authors
 *
 * Autolink action/service endpoints for PlannerServer.
 */

#include "autonomy/planning/planner_server.hpp"

#include <chrono>
#include <memory>

#include "autonomy/common/logging.hpp"
#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/builtin_interfaces/duration.pb.h>
#include <automsgs/msgs/time_utils.hpp>
#include <automsgs/msgs/status_msgs/status_msgs.pb.h>
#include "autonomy/planning/common/planner_exceptions.hpp"
#include "autonomy/planning/common/smoother_exceptions.hpp"
#include "autonomy/planning/constants.hpp"

namespace autonomy {
namespace planning {
namespace {

using Time = automsgs::msgs::builtin_interfaces::Time;
namespace err_proto = automsgs::msgs::status_msgs;

err_proto::StatusCode MapComputePathToPoseError(const std::exception& ex)
{
    if (dynamic_cast<const common::InvalidPlanner*>(&ex)) {
        return err_proto::PLANNING_INVALID_PLUGIN;
    }
    if (dynamic_cast<const common::StartOccupied*>(&ex)) {
        return err_proto::PLANNING_START_OCCUPIED;
    }
    if (dynamic_cast<const common::GoalOccupied*>(&ex)) {
        return err_proto::PLANNING_GOAL_OCCUPIED;
    }
    if (dynamic_cast<const common::NoValidPathCouldBeFound*>(&ex)) {
        return err_proto::PLANNING_NO_PATH_FOUND;
    }
    if (dynamic_cast<const common::PlannerTimedOut*>(&ex)) {
        return err_proto::PLANNING_TIMEOUT;
    }
    if (dynamic_cast<const common::StartOutsideMapBounds*>(&ex)) {
        return err_proto::PLANNING_START_OUTSIDE_MAP;
    }
    if (dynamic_cast<const common::GoalOutsideMapBounds*>(&ex)) {
        return err_proto::PLANNING_GOAL_OUTSIDE_MAP;
    }
    if (dynamic_cast<const common::PlannerTFError*>(&ex)) {
        return err_proto::PLANNING_TF_ERROR;
    }
    return err_proto::PLANNING_UNKNOWN;
}

err_proto::StatusCode MapComputePathThroughPosesError(const std::exception& ex)
{
    if (dynamic_cast<const common::NoViapointsGiven*>(&ex)) {
        return err_proto::PLANNING_NO_WAYPOINTS;
    }
    if (dynamic_cast<const common::InvalidPlanner*>(&ex)) {
        return err_proto::PLANNING_INVALID_PLUGIN;
    }
    if (dynamic_cast<const common::NoValidPathCouldBeFound*>(&ex)) {
        return err_proto::PLANNING_NO_PATH_FOUND;
    }
    if (dynamic_cast<const common::PlannerTFError*>(&ex)) {
        return err_proto::PLANNING_TF_ERROR;
    }
    if (dynamic_cast<const common::PlannerTimedOut*>(&ex)) {
        return err_proto::PLANNING_TIMEOUT;
    }
    return err_proto::PLANNING_UNKNOWN;
}

err_proto::StatusCode MapSmoothPathError(const std::exception& ex)
{
    if (dynamic_cast<const common::InvalidSmoother*>(&ex)) {
        return err_proto::SMOOTHER_INVALID_PLUGIN;
    }
    if (dynamic_cast<const common::SmootherTimedOut*>(&ex)) {
        return err_proto::SMOOTHER_TIMEOUT;
    }
    if (dynamic_cast<const common::SmoothedPathInCollision*>(&ex)) {
        return err_proto::SMOOTHER_PATH_IN_COLLISION;
    }
    if (dynamic_cast<const common::FailedToSmoothPath*>(&ex)) {
        return err_proto::SMOOTHER_FAILED;
    }
    if (dynamic_cast<const common::InvalidPath*>(&ex)) {
        return err_proto::SMOOTHER_INVALID_PATH;
    }
    return err_proto::SMOOTHER_UNKNOWN;
}

std::chrono::milliseconds MaxSmoothingDuration(
    const nav_proto::SmoothPathAction::Goal& goal)
{
    if (!goal.has_max_smoothing_duration()) {
        return std::chrono::milliseconds(1000);
    }
    const double max_sec =
        automsgs::msgs::builtin_interfaces::DurationToSeconds(
            goal.max_smoothing_duration());
    if (max_sec <= 0.0) {
        return std::chrono::milliseconds(1000);
    }
    return std::chrono::milliseconds(static_cast<int>(max_sec * 1000.0));
}

}  // namespace

void PlannerServer::RegisterAutolinkEndpoints()
{
    if (!node_) {
        return;
    }

    PlannerServer* self = this;

    compute_path_to_pose_server_ = std::make_shared<ComputePathToPoseServer>(
        node_, kComputePathToPoseActionName,
        [self]() { self->ComputePlan(); });

    compute_path_through_poses_server_ =
        std::make_shared<ComputePathThroughPosesServer>(
            node_, kComputePathThroughPosesActionName,
            [self]() { self->ComputePlanThroughPoses(); });

    smooth_path_server_ = std::make_shared<SmoothPathServer>(
        node_, kSmoothPathActionName, [self]() { self->SmoothPathAction(); });

    path_valid_service_ =
        node_->CreateService<PathValidRequest, PathValidResponse>(
            kIsPathValidServiceName,
            [self](const std::shared_ptr<PathValidRequest>& request,
                   std::shared_ptr<PathValidResponse>& response) {
                const auto path =
                    request->path();
                const uint8_t max_cost = request->max_cost() > 0
                                             ? static_cast<uint8_t>(request->max_cost())
                                             : 253;
                response->set_is_valid(self->IsPathValid(
                    path, max_cost, request->consider_unknown_as_obstacle()));
            });

    clear_costmap_service_ =
        node_->CreateService<ClearCostmapRequest, ClearCostmapResponse>(
            kClearGlobalCostmapServiceName,
            [self](const std::shared_ptr<ClearCostmapRequest>& /*request*/,
                   std::shared_ptr<ClearCostmapResponse>& /*response*/) {
                self->ClearEntireCostmap();
            });

    AINFO << "PlannerServer autolink action/service endpoints started.";
}

void PlannerServer::ComputePlan()
{
    std::lock_guard<std::mutex> lock(dynamic_params_mutex_);

    auto& server = compute_path_to_pose_server_;
    if (!server || !server->IsServerActive() || server->IsCancelRequested()) {
        return;
    }

    auto goal = server->GetCurrentGoal();
    if (!goal) {
        return;
    }
    if (server->IsPreemptRequested()) {
        goal = server->AcceptPendingGoal();
    }
    if (!goal || !server->IsServerActive()) {
        return;
    }

    auto result = std::make_shared<nav_proto::ComputePathToPoseAction::Result>();
    const auto start_time = automsgs::msgs::builtin_interfaces::TimeNow();
    metrics_.plans_requested.fetch_add(1, std::memory_order_relaxed);

    try {
        if (server->IsCancelRequested()) {
            return;
        }

        WaitForCostmap();

        automsgs::msgs::geometry_msgs::PoseStamped start;
        automsgs::msgs::geometry_msgs::PoseStamped goal_pose;

        if (goal->use_start() && goal->has_start()) {
            start = goal->start();
        } else if (!costmap_wrapper_ ||
                   !costmap_wrapper_->getRobotPose(start)) {
            throw common::PlannerTFError("Unable to get start pose");
        }

        goal_pose = goal->goal();
        if (!TransformPosesToGlobalFrame(start, goal_pose)) {
            throw common::PlannerTFError(
                "Unable to transform poses to global frame");
        }

        auto cancel_checker = [&]() { return server->IsCancelRequested(); };
        const auto path =
            GetPlan(start, goal_pose, goal->planner_id(), cancel_checker);
        if (!ValidatePath(goal_pose, path, goal->planner_id())) {
            throw common::NoValidPathCouldBeFound(goal->planner_id() + " generated an empty path");
        }

        PublishPlan(path);
        metrics_.plans_succeeded.fetch_add(1, std::memory_order_relaxed);

        *result->mutable_path() = path;
        *result->mutable_planning_time() =
            (automsgs::msgs::builtin_interfaces::TimeNow() - start_time);
        result->set_error_code(err_proto::OK);
        server->SucceededCurrent(result);
    } catch (const common::PlannerCancelled&) {
        metrics_.plans_failed.fetch_add(1, std::memory_order_relaxed);
        result->set_error_msg("Goal was canceled. Canceling planning action.");
        server->TerminateAll(result);
    } catch (const std::exception& ex) {
        metrics_.plans_failed.fetch_add(1, std::memory_order_relaxed);
        result->set_error_code(MapComputePathToPoseError(ex));
        result->set_error_msg(ex.what());
        server->TerminateCurrent(result);
    }
}

void PlannerServer::ComputePlanThroughPoses()
{
    std::lock_guard<std::mutex> lock(dynamic_params_mutex_);

    auto& server = compute_path_through_poses_server_;
    if (!server || !server->IsServerActive() || server->IsCancelRequested()) {
        return;
    }

    auto goal = server->GetCurrentGoal();
    if (!goal) {
        return;
    }
    if (server->IsPreemptRequested()) {
        goal = server->AcceptPendingGoal();
    }
    if (!goal || !server->IsServerActive()) {
        return;
    }

    auto result =
        std::make_shared<nav_proto::ComputePathThroughPosesAction::Result>();
    const auto start_time = automsgs::msgs::builtin_interfaces::TimeNow();
    metrics_.plans_requested.fetch_add(1, std::memory_order_relaxed);

    try {
        if (server->IsCancelRequested()) {
            return;
        }

        WaitForCostmap();

        std::vector<automsgs::msgs::geometry_msgs::PoseStamped> goals;
        if (goal->has_goals()) {
            goals.reserve(static_cast<size_t>(goal->goals().goals_size()));
            for (const auto& pose_proto : goal->goals().goals()) {
                goals.push_back(pose_proto);
            }
        }
        if (goals.empty()) {
            throw common::NoViapointsGiven("No viapoints given");
        }

        automsgs::msgs::geometry_msgs::PoseStamped curr_start;
        if (goal->use_start() && goal->has_start()) {
            curr_start = goal->start();
        } else if (!costmap_wrapper_ ||
                   !costmap_wrapper_->getRobotPose(curr_start)) {
            throw common::PlannerTFError("Unable to get start pose");
        }

        auto cancel_checker = [&]() { return server->IsCancelRequested(); };
        automsgs::msgs::nav_msgs::Path merged_path;

        for (size_t i = 0; i < goals.size(); ++i) {
            if (server->IsCancelRequested()) {
                throw common::PlannerCancelled(
                    "ComputePathThroughPoses cancelled");
            }

            automsgs::msgs::geometry_msgs::PoseStamped segment_start =
                (i == 0) ? curr_start : merged_path.poses(merged_path.poses_size() - 1);
            if (i > 0) {
                *segment_start.mutable_header() = merged_path.header();
            }

            auto curr_goal = goals[i];
            if (!TransformPosesToGlobalFrame(segment_start, curr_goal)) {
                throw common::PlannerTFError(
                    "Unable to transform poses to global frame");
            }

            auto segment = GetPlan(segment_start, curr_goal, goal->planner_id(),
                                   cancel_checker);
            if (!ValidatePath(curr_goal, segment, goal->planner_id())) {
                throw common::NoValidPathCouldBeFound(goal->planner_id() + " generated an empty path");
            }

            for (const auto& pose : segment.poses()) {
                *merged_path.mutable_poses()->Add() = pose;
            }
            *merged_path.mutable_header() = segment.header();
        }

        PublishPlan(merged_path);
        metrics_.plans_succeeded.fetch_add(1, std::memory_order_relaxed);

        *result->mutable_path() = merged_path;
        *result->mutable_planning_time() =
            (automsgs::msgs::builtin_interfaces::TimeNow() - start_time);
        result->set_error_code(err_proto::OK);
        server->SucceededCurrent(result);
    } catch (const common::PlannerCancelled&) {
        metrics_.plans_failed.fetch_add(1, std::memory_order_relaxed);
        result->set_error_msg("Goal was canceled. Canceling planning action.");
        server->TerminateAll(result);
    } catch (const std::exception& ex) {
        metrics_.plans_failed.fetch_add(1, std::memory_order_relaxed);
        result->set_error_code(MapComputePathThroughPosesError(ex));
        result->set_error_msg(ex.what());
        server->TerminateCurrent(result);
    }
}

void PlannerServer::SmoothPathAction()
{
    std::lock_guard<std::mutex> lock(dynamic_params_mutex_);

    auto& server = smooth_path_server_;
    if (!server || !server->IsServerActive() || server->IsCancelRequested()) {
        return;
    }

    auto goal = server->GetCurrentGoal();
    if (!goal) {
        return;
    }
    if (server->IsPreemptRequested()) {
        goal = server->AcceptPendingGoal();
    }
    if (!goal || !server->IsServerActive()) {
        return;
    }

    auto result = std::make_shared<nav_proto::SmoothPathAction::Result>();
    const auto start_time = std::chrono::steady_clock::now();

    try {
        if (!default_smoother_) {
            throw common::InvalidSmoother("No smoother configured");
        }

        automsgs::msgs::nav_msgs::Path path =
            goal->path();
        if (path.poses_size() < 2) {
            throw common::InvalidPath("Path must contain at least 2 poses");
        }

        const auto max_time = MaxSmoothingDuration(*goal);
        auto cancel_checker = [&]() { return server->IsCancelRequested(); };
        if (cancel_checker()) {
            throw common::SmootherTimedOut("SmoothPath cancelled");
        }

        const bool was_completed = default_smoother_->Smooth(path, max_time);
        if (!was_completed) {
            throw common::SmootherTimedOut("Smoother exceeded max duration");
        }

        PublishPlan(path);

        const auto elapsed = std::chrono::steady_clock::now() - start_time;
        const double duration_sec =
            std::chrono::duration<double>(elapsed).count();

        *result->mutable_path() = path;
        *result->mutable_smoothing_duration() =
            (
                automsgs::msgs::builtin_interfaces::DurationFromSeconds(duration_sec));
        result->set_was_completed(was_completed);
        result->set_error_code(err_proto::OK);
        server->SucceededCurrent(result);
    } catch (const std::exception& ex) {
        result->set_error_msg(ex.what());
        result->set_error_code(MapSmoothPathError(ex));
        server->TerminateCurrent(result);
    }
}

}  // namespace planning
}  // namespace autonomy
