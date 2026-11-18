/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/charging/charging.hpp"

#include <algorithm>
#include <cmath>

#include "autolink/autolink.hpp"
#include "autonomy/common/conf_loader.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/task/navigation/navigation_client.hpp"

namespace autonomy {
namespace task {
namespace {

constexpr char kGlobalFrame[] = "map";
constexpr char kRobotBaseFrame[] = "base_link";
constexpr char kDefaultPlannerId[] = "navfn_planner";
constexpr char kDefaultControllerId[] = "FollowPath";
constexpr char kDefaultSmootherId[] = "simple_smoother";
constexpr double kDefaultGoalReachedTol = 0.15;

}  // namespace

using RobotTaskType = ::automsgs::msgs::vehicle_msgs::RobotTaskType;
namespace tp = ::autonomy::task::proto;

RobotTaskType ChargingTask::GetTaskType() const
{
    return RobotTaskType::ROBOT_TASK_DOCK;
}

void ChargingTask::SetChargingClient(charging::ChargingClient::Ptr client)
{
    charging_client_ = std::move(client);
    charging::ChargingClient::SetShared(charging_client_);
}

bool ChargingTask::EnsureChargingClient()
{
    if (charging_client_) {
        return true;
    }
    if (!navigation()) {
        return false;
    }
    charging_client_ = charging::ChargingClient::Create(navigation(), node());
    charging::ChargingClient::SetShared(charging_client_);
    return static_cast<bool>(charging_client_);
}

void ChargingTask::StartAutoRecharge(
    charging::AutoRechargeSupervisor::SubmitFn submit,
    charging::AutoRechargeSupervisor::TeleopFn teleop_active)
{
    if (!EnsureChargingClient() || !charging_client_) {
        return;
    }
    if (!auto_recharge_.Configure(node(), charging_client_->auto_recharge())) {
        return;
    }
    auto_recharge_.Start(
        std::move(submit), std::move(teleop_active),
        [this] { return IsActive(); },
        [this] {
            return charging_client_ ? charging_client_->phase()
                                    : tp::DOCK_PHASE_NONE;
        },
        [this] {
            return charging_client_ ? charging_client_->SelectDockId()
                                    : std::string();
        });
}

void ChargingTask::StopAutoRecharge() { auto_recharge_.Stop(); }

bool ChargingTask::OnTreeInitialize(const tp::TaskServerOptions& /*options*/)
{
    if (!EnsureChargingClient()) {
        return false;
    }
    tp::DockingOptions docking;
    if (!::autonomy::common::LoadModuleConf("task", "charging.pb.txt",
                                            &docking)) {
        AWARN << "charging.pb.txt missing; using docking defaults";
    }
    if (docking.docks_size() == 1 && docking.docks(0).x() == 0.0 &&
        docking.docks(0).y() == 0.0) {
        AWARN << "charging dock pose is (0,0); set docks{} in charging.pb.txt "
                 "to the mapped charger";
    }
    return charging_client_->Configure(docking);
}

void ChargingTask::PopulateBlackboard(const BT::Blackboard::Ptr& blackboard)
{
    if (!blackboard || !charging_client_) {
        return;
    }

    blackboard->set(charging::kChargingClientBlackboardKey, charging_client_);
    blackboard->set(navigation::kNavigationClientBlackboardKey,
                    charging_client_->navigation_client());
    blackboard->set("global_frame", std::string(kGlobalFrame));
    blackboard->set("robot_base_frame", std::string(kRobotBaseFrame));
    blackboard->set("default_planner_id", std::string(kDefaultPlannerId));
    blackboard->set("default_controller_id", std::string(kDefaultControllerId));
    blackboard->set("default_smoother_id", std::string(kDefaultSmootherId));
    blackboard->set("goal_reached_tol", kDefaultGoalReachedTol);
    blackboard->set("dock_pose", charging_client_->dock_pose());
    blackboard->set("dock_station_id", charging_client_->dock_station_id());
    blackboard->set("battery_target_percent",
                    charging_client_->battery_target_percent());
    blackboard->set("use_dock_id", charging_client_->use_dock_id());
    blackboard->set("dock_type", charging_client_->dock_type());
    blackboard->set("max_staging_time", charging_client_->max_staging_time());
    blackboard->set("navigate_to_staging_pose",
                    charging_client_->navigate_to_staging());
    blackboard->set("max_undocking_time",
                    charging_client_->max_undocking_time());
}

bool ChargingTask::OnGoal(const tp::ChargingGoal& goal)
{
    using Command = tp::DockCommand;
    switch (goal.command()) {
    case Command::DOCK_CMD_START: {
        if (!EnsureChargingClient()) {
            return false;
        }
        active_goal_ = goal;
        charging_client_->ApplyGoal(goal);

        if (!StartTree()) {
            active_goal_.reset();
            return false;
        }
        SetLifecycle(TaskLifecycle::kRunning);
        SetProgress(0.f, "docking");
        return true;
    }
    case Command::DOCK_CMD_RESUME:
        if (!ResumeTree()) {
            return false;
        }
        return Resume();
    case Command::DOCK_CMD_PAUSE:
        if (!PauseTree()) {
            return false;
        }
        return Pause();
    case Command::DOCK_CMD_UNDOCK: {
        if (!EnsureChargingClient()) {
            return false;
        }
        active_goal_ = goal;
        charging_client_->ApplyGoal(goal);
        const auto tree = profile().AlternateTreePath(config_directory());
        if (tree.empty() || !StartTree(tree)) {
            active_goal_.reset();
            return false;
        }
        SetLifecycle(TaskLifecycle::kRunning);
        SetProgress(0.f, "undocking");
        return true;
    }
    case Command::DOCK_CMD_STOP:
    case Command::DOCK_CMD_CANCEL:
        if (charging_client_) {
            charging_client_->CancelActiveMotion();
        }
        StopTree();
        active_goal_.reset();
        auto_recharge_.NotifyCancel();
        SetLifecycle(TaskLifecycle::kCanceled);
        return true;
    default:
        return false;
    }
}

void ChargingTask::OnTreeTick()
{
    switch (runner()->state()) {
    case BtRunState::kSucceeded:
        SetLifecycle(TaskLifecycle::kSucceeded);
        SetProgress(1.f, "dock succeeded");
        auto_recharge_.NotifySuccess();
        active_goal_.reset();
        return;
    case BtRunState::kFailed:
        SetLifecycle(TaskLifecycle::kFailed);
        SetProgress(progress_.progress(), "dock failed");
        auto_recharge_.NotifyFailure();
        active_goal_.reset();
        return;
    case BtRunState::kCanceled:
        SetLifecycle(TaskLifecycle::kCanceled);
        auto_recharge_.NotifyCancel();
        active_goal_.reset();
        return;
    case BtRunState::kRunning:
    default:
        break;
    }

    if (Lifecycle() != TaskLifecycle::kRunning || !charging_client_) {
        return;
    }
    float progress = 0.3f;
    const char* msg = "docking";
    switch (charging_client_->phase()) {
    case tp::DOCK_PHASE_NAV_TO_STAGING:
        progress = 0.2f;
        msg = "nav to staging";
        break;
    case tp::DOCK_PHASE_INITIAL_PERCEPTION:
        progress = 0.4f;
        msg = "detecting dock";
        break;
    case tp::DOCK_PHASE_CONTROLLING:
        progress = 0.6f;
        msg = "approaching dock";
        break;
    case tp::DOCK_PHASE_WAIT_FOR_CHARGE:
        progress = 0.75f;
        msg = "waiting for charge";
        break;
    case tp::DOCK_PHASE_HOLD_CHARGE: {
        const float soc = charging_client_->battery_percent();
        const float tgt = charging_client_->battery_target_percent();
        progress = 0.8f;
        if (std::isfinite(soc) && tgt > 0.f) {
            progress = 0.8f + 0.15f * std::clamp(soc / tgt, 0.f, 1.f);
        }
        msg = "charging";
        break;
    }
    case tp::DOCK_PHASE_RETRY:
        progress = 0.35f;
        msg = "retry staging";
        break;
    case tp::DOCK_PHASE_UNDOCKING:
        progress = 0.5f;
        msg = "undocking";
        break;
    default:
        break;
    }
    SetProgress(progress, msg);
}

tp::DockStatus ChargingTask::MapStatus() const
{
    using Status = tp::DockStatus;
    switch (Lifecycle()) {
    case TaskLifecycle::kIdle:
        return Status::DOCK_STATUS_IDLE;
    case TaskLifecycle::kRunning:
        if (charging_client_) {
            switch (charging_client_->phase()) {
            case tp::DOCK_PHASE_NAV_TO_STAGING:
                return Status::DOCK_STATUS_APPROACHING;
            case tp::DOCK_PHASE_INITIAL_PERCEPTION:
                return Status::DOCK_STATUS_SEARCHING;
            case tp::DOCK_PHASE_WAIT_FOR_CHARGE:
            case tp::DOCK_PHASE_HOLD_CHARGE:
                return Status::DOCK_STATUS_CHARGING;
            case tp::DOCK_PHASE_UNDOCKING:
                return Status::DOCK_STATUS_UNDOCKING;
            default:
                break;
            }
        }
        return Status::DOCK_STATUS_DOCKING;
    case TaskLifecycle::kPaused:
        return Status::DOCK_STATUS_APPROACHING;
    case TaskLifecycle::kSucceeded:
        return Status::DOCK_STATUS_SUCCEEDED;
    case TaskLifecycle::kFailed:
        return Status::DOCK_STATUS_FAILED;
    case TaskLifecycle::kCanceled:
        return Status::DOCK_STATUS_CANCELED;
    default:
        return Status::DOCK_STATUS_UNKNOWN;
    }
}

void ChargingTask::FillFeedback(tp::ChargingFeedback* feedback) const
{
    feedback->set_status(MapStatus());
    *feedback->mutable_progress() = progress_;
    if (charging_client_) {
        feedback->set_dock_station_id(charging_client_->dock_station_id());
        feedback->set_phase(charging_client_->phase());
        feedback->set_num_retries(charging_client_->num_retries());
        feedback->set_docking_time_sec(charging_client_->docking_time_sec());
        feedback->set_error_code(charging_client_->error_code());
        const float soc = charging_client_->battery_percent();
        if (std::isfinite(soc)) {
            feedback->set_battery_percent(soc);
        } else if (std::isfinite(auto_recharge_.soc_percent())) {
            feedback->set_battery_percent(auto_recharge_.soc_percent());
        }
        feedback->set_auto_recharge_enabled(auto_recharge_.enabled());
    }
}

void ChargingTask::FillResult(tp::ChargingResult* result) const
{
    *result->mutable_result() = MakeTaskResult();
    result->set_final_status(MapStatus());
    if (charging_client_) {
        result->set_num_retries(charging_client_->num_retries());
        result->set_error_code(charging_client_->error_code());
        result->set_success(Lifecycle() == TaskLifecycle::kSucceeded);
        const float soc = charging_client_->battery_percent();
        if (std::isfinite(soc)) {
            result->set_battery_percent(soc);
        }
    }
}

}  // namespace task
}  // namespace autonomy
