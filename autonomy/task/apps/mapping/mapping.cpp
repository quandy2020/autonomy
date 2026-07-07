/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/apps/mapping/mapping.hpp"

#include "autolink/autolink.hpp"
#include "autonomy/task/apps/navigation/navigation_client.hpp"

namespace autonomy {
namespace task {
namespace {

constexpr char kClearCostmapTree[] =
    "task/behavior_tree/mapping/map_clear_costmap.xml";

}  // namespace

using RobotTaskType = ::autonomy::commsgs::proto::vehicle_msgs::RobotTaskType;
namespace tp = ::autonomy::task::proto;

RobotTaskType MappingTask::GetTaskType() const
{
    return RobotTaskType::ROBOT_TASK_MAP;
}

void MappingTask::SetMappingClient(mapping::MappingClient::Ptr client)
{
    mapping_client_ = std::move(client);
    mapping::MappingClient::SetShared(mapping_client_);
}

bool MappingTask::EnsureMappingClient()
{
    if (mapping_client_) {
        return true;
    }
    if (!shared_navigation()) {
        return false;
    }
    mapping_client_ = mapping::MappingClient::Create(shared_navigation());
    mapping::MappingClient::SetShared(mapping_client_);
    return static_cast<bool>(mapping_client_);
}

bool MappingTask::OnTreeInitialize(const tp::TaskServerOptions& /*options*/)
{
    return EnsureMappingClient();
}

void MappingTask::PopulateBlackboard(const BT::Blackboard::Ptr& blackboard)
{
    if (!blackboard || !mapping_client_) {
        return;
    }

    blackboard->set(mapping::kMappingClientBlackboardKey, mapping_client_);
    blackboard->set(navigation::kNavigationClientBlackboardKey,
                    mapping_client_->navigation_ptr());
    blackboard->set("map_name", mapping_client_->map_name());
}

std::string MappingTask::ResolveTreeForGoal(const tp::MappingGoal& goal) const
{
    using Command = tp::MapCommand;
    switch (goal.command()) {
    case Command::MAP_CMD_SET_INITIAL_POSE:
        return profile().AlternateTreePath(config_directory());
    case Command::MAP_CMD_CLEAR_COSTMAP:
        return config_directory() + "/" + kClearCostmapTree;
    case Command::MAP_CMD_LOAD:
    case Command::MAP_CMD_SWITCH:
    default:
        return profile().DefaultTreePath(config_directory());
    }
}

bool MappingTask::OnGoal(const tp::MappingGoal& goal)
{
    using Command = tp::MapCommand;
    switch (goal.command()) {
    case Command::MAP_CMD_LOAD:
    case Command::MAP_CMD_SWITCH:
    case Command::MAP_CMD_SET_INITIAL_POSE:
    case Command::MAP_CMD_CLEAR_COSTMAP: {
        if (!EnsureMappingClient()) {
            return false;
        }
        active_goal_ = goal;
        mapping_client_->ApplyGoal(goal);

        const auto tree = ResolveTreeForGoal(goal);
        if (!StartTree(tree)) {
            active_goal_.reset();
            return false;
        }
        SetLifecycle(TaskLifecycle::kRunning);
        SetProgress(0.f, "map command");
        return true;
    }
    default:
        return false;
    }
}

void MappingTask::OnTreeTick()
{
    switch (runner()->state()) {
    case BtRunState::kSucceeded:
        SetLifecycle(TaskLifecycle::kSucceeded);
        SetProgress(1.f, "map command done");
        active_goal_.reset();
        return;
    case BtRunState::kFailed:
        SetLifecycle(TaskLifecycle::kFailed);
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

    if (Lifecycle() == TaskLifecycle::kRunning) {
        SetProgress(0.5f, "map command");
    }
}

tp::MapStatus MappingTask::MapStatus() const
{
    using Status = tp::MapStatus;
    switch (Lifecycle()) {
    case TaskLifecycle::kIdle:
        return Status::MAP_STATUS_IDLE;
    case TaskLifecycle::kRunning:
        return Status::MAP_STATUS_LOADING;
    case TaskLifecycle::kSucceeded:
        return Status::MAP_STATUS_SUCCEEDED;
    case TaskLifecycle::kFailed:
        return Status::MAP_STATUS_FAILED;
    default:
        return Status::MAP_STATUS_UNKNOWN;
    }
}

void MappingTask::FillFeedback(tp::MappingFeedback* feedback) const
{
    feedback->set_status(MapStatus());
    *feedback->mutable_progress() = progress_;
    if (mapping_client_) {
        feedback->set_current_map_name(mapping_client_->map_name());
    }
}

void MappingTask::FillResult(tp::MappingResult* result) const
{
    *result->mutable_result() = MakeTaskResult();
    result->set_final_status(MapStatus());
    if (mapping_client_) {
        result->set_current_map_name(mapping_client_->map_name());
    }
}

}  // namespace task
}  // namespace autonomy
