/*
 * Copyright 2026 The Openbot Authors
 */

#include <algorithm>
#include <string>

#include "autolink/autolink.hpp"
#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/vector3.pb.h>
#include "autonomy/task/apps/exploration/exploration.hpp"
#include "autonomy/task/apps/navigation/navigation_client.hpp"

namespace autonomy {
namespace task {
namespace {

constexpr char kGlobalFrame[] = "map";
constexpr char kRobotBaseFrame[] = "base_link";
constexpr char kDefaultPlannerId[] = "navfn_planner";
constexpr char kDefaultControllerId[] = "FollowPath";
constexpr char kDefaultSmootherId[] = "simple_smoother";
constexpr double kDefaultGoalReachedTol = 0.25;
constexpr char kDefaultMapName[] = "exploration_map";

automsgs::msgs::geometry_msgs::Polygon ToPolygon(
    const ::automsgs::msgs::geometry_msgs::Polygon& proto)
{
    return proto;
}

}  // namespace

using RobotTaskType = ::automsgs::msgs::vehicle_msgs::RobotTaskType;
namespace exp_proto = ::autonomy::task::proto;

RobotTaskType ExplorationTask::GetTaskType() const
{
    return RobotTaskType::ROBOT_TASK_EXPLORATION;
}

void ExplorationTask::SetExplorationClient(
    exploration::ExplorationClient::Ptr client)
{
    exploration_client_ = std::move(client);
    exploration::ExplorationClient::SetShared(exploration_client_);
}

bool ExplorationTask::EnsureExplorationClient()
{
    if (exploration_client_) {
        return true;
    }
    if (!shared_navigation()) {
        return false;
    }
    exploration_client_ =
        exploration::ExplorationClient::Create(shared_navigation());
    exploration::ExplorationClient::SetShared(exploration_client_);
    return static_cast<bool>(exploration_client_);
}

bool ExplorationTask::OnTreeInitialize(
    const exp_proto::TaskServerOptions& /*options*/)
{
    return EnsureExplorationClient();
}

void ExplorationTask::ApplyGoalParams(const exp_proto::ExplorationGoal& goal)
{
    if (!exploration_client_) {
        return;
    }

    switch (goal.params_case()) {
    case exp_proto::ExplorationGoal::kMapName:
        exploration_client_->SetMapName(goal.map_name());
        break;
    case exp_proto::ExplorationGoal::kArea:
        exploration_client_->SetExplorationArea(ToPolygon(goal.area()));
        break;
    default:
        break;
    }
}

void ExplorationTask::PopulateBlackboard(const BT::Blackboard::Ptr& blackboard)
{
    if (!blackboard || !exploration_client_) {
        return;
    }

    blackboard->set(exploration::kExplorationClientBlackboardKey,
                    exploration_client_);
    blackboard->set(navigation::kNavigationClientBlackboardKey,
                    exploration_client_->navigation_ptr());

    blackboard->set("global_frame", std::string(kGlobalFrame));
    blackboard->set("robot_base_frame", std::string(kRobotBaseFrame));
    blackboard->set("default_planner_id", std::string(kDefaultPlannerId));
    blackboard->set("default_controller_id", std::string(kDefaultControllerId));
    blackboard->set("default_smoother_id", std::string(kDefaultSmootherId));
    blackboard->set("goal_reached_tol", kDefaultGoalReachedTol);
    blackboard->set("map_name", exploration_client_->map_name());

    if (!active_goal_.has_value()) {
        return;
    }

    const auto& goal = *active_goal_;
    if (goal.params_case() == exp_proto::ExplorationGoal::kMapName &&
        !goal.map_name().empty()) {
        blackboard->set("map_name", goal.map_name());
    }
}

std::string ExplorationTask::ResolveTreeForGoal(
    const exp_proto::ExplorationGoal& /*goal*/) const
{
    return profile().DefaultTreePath(config_directory());
}

bool ExplorationTask::OnGoal(const exp_proto::ExplorationGoal& goal)
{
    using Command = exp_proto::ExplorationCommand;
    switch (goal.command()) {
    case Command::EXPLORATION_CMD_START:
    case Command::EXPLORATION_CMD_SET_AREA: {
        if (!EnsureExplorationClient()) {
            return false;
        }
        active_goal_ = goal;
        ApplyGoalParams(goal);
        if (goal.command() == Command::EXPLORATION_CMD_START &&
            goal.params_case() != exp_proto::ExplorationGoal::kArea) {
            exploration_client_->UseDefaultExplorationArea();
        }
        if (!exploration_client_->HasFrontier() &&
            exploration_client_->IsExplorationFinished()) {
            return false;
        }

        const auto tree = ResolveTreeForGoal(goal);
        if (!StartTree(tree)) {
            active_goal_.reset();
            return false;
        }
        SetLifecycle(TaskLifecycle::kRunning);
        SetProgress(0.f, "bt:" + tree);
        return true;
    }
    case Command::EXPLORATION_CMD_RESUME:
        if (!ResumeTree()) {
            return false;
        }
        return Resume();
    case Command::EXPLORATION_CMD_PAUSE:
        if (!PauseTree()) {
            return false;
        }
        return Pause();
    case Command::EXPLORATION_CMD_SAVE_MAP:
        if (goal.has_map_name()) {
            saved_map_name_ = goal.map_name();
            if (exploration_client_) {
                exploration_client_->SetMapName(goal.map_name());
            }
        }
        StopTree();
        SetLifecycle(TaskLifecycle::kSucceeded);
        return true;
    case Command::EXPLORATION_CMD_STOP:
    case Command::EXPLORATION_CMD_CANCEL:
        if (exploration_client_) {
            exploration_client_->CancelActiveMotion();
        }
        StopTree();
        active_goal_.reset();
        SetLifecycle(TaskLifecycle::kCanceled);
        return true;
    default:
        return false;
    }
}

void ExplorationTask::OnTreeTick()
{
    switch (runner()->state()) {
    case BtRunState::kSucceeded:
        SetLifecycle(TaskLifecycle::kSucceeded);
        SetProgress(1.f, "exploration complete: " + runner()->active_tree());
        if (exploration_client_) {
            saved_map_name_ = exploration_client_->map_name();
        }
        active_goal_.reset();
        return;
    case BtRunState::kFailed:
        SetLifecycle(TaskLifecycle::kFailed);
        SetProgress(progress_.progress(),
                    "exploration failed: " + runner()->active_tree());
        active_goal_.reset();
        return;
    case BtRunState::kCanceled:
        SetLifecycle(TaskLifecycle::kCanceled);
        active_goal_.reset();
        return;
    case BtRunState::kRunning:
    default:
        break;
    }

    if (Lifecycle() != TaskLifecycle::kRunning || !exploration_client_) {
        return;
    }

    float progress = exploration_client_->exploration_progress();
    progress = std::min(0.95f, std::max(progress, progress_.progress()));
    SetProgress(progress, "exploring: " + runner()->active_tree());
}

exp_proto::ExplorationStatus ExplorationTask::MapStatus() const
{
    using Status = exp_proto::ExplorationStatus;
    switch (Lifecycle()) {
    case TaskLifecycle::kIdle:
        return Status::EXPLORATION_STATUS_IDLE;
    case TaskLifecycle::kRunning:
        return Status::EXPLORATION_STATUS_EXPLORING;
    case TaskLifecycle::kPaused:
        return Status::EXPLORATION_STATUS_PAUSED;
    case TaskLifecycle::kSucceeded:
        return Status::EXPLORATION_STATUS_COMPLETED;
    case TaskLifecycle::kFailed:
        return Status::EXPLORATION_STATUS_FAILED;
    case TaskLifecycle::kCanceled:
        return Status::EXPLORATION_STATUS_CANCELED;
    default:
        return Status::EXPLORATION_STATUS_UNKNOWN;
    }
}

void ExplorationTask::FillFeedback(exp_proto::ExplorationFeedback* feedback) const
{
    feedback->set_status(MapStatus());
    *feedback->mutable_progress() = progress_;
    if (exploration_client_) {
        feedback->set_explored_area(exploration_client_->explored_area_m2());
        feedback->set_exploration_progress(
            exploration_client_->exploration_progress());
    }
}

void ExplorationTask::FillResult(exp_proto::ExplorationResult* result) const
{
    *result->mutable_result() = MakeTaskResult();
    result->set_final_status(MapStatus());
    if (!saved_map_name_.empty()) {
        result->set_saved_map_name(saved_map_name_);
    } else if (exploration_client_) {
        result->set_saved_map_name(exploration_client_->map_name());
    }
}

}  // namespace task
}  // namespace autonomy
