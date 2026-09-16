/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/exploration/exploration.hpp"

#include <cmath>

#include "autonomy/common/logging.hpp"
#include "autonomy/task/common/names.hpp"
#include <automsgs/msgs/vehicle_msgs/robot_task_type.pb.h>

namespace autonomy {
namespace task {
namespace {

namespace tp = ::autonomy::task::proto;
using RobotTaskType = ::automsgs::msgs::vehicle_msgs::RobotTaskType;

}  // namespace

RobotTaskType ExplorationTask::GetTaskType() const {
    return RobotTaskType::ROBOT_TASK_EXPLORATION;
}

void ExplorationTask::SetNode(std::shared_ptr<autolink::Node> node) {
    node_ = std::move(node);
}

void ExplorationTask::SetSubmitNavigation(SubmitNavigation submit) {
    submit_navigation_ = std::move(submit);
}

void ExplorationTask::SetSubmitMapping(SubmitMapping submit) {
    submit_mapping_ = std::move(submit);
}

void ExplorationTask::SetNavigationProbe(IsNavigationActive is_active,
                                         GetNavigationResult get_result) {
    is_navigation_active_ = std::move(is_active);
    get_navigation_result_ = std::move(get_result);
}

bool ExplorationTask::OnInitialize(const tp::TaskServerOptions& options) {
    if (!node_) {
        AERROR << "ExplorationTask: node is null";
        return false;
    }
    const auto period = std::chrono::milliseconds(
        options.scheduler().feedback_period_ms() > 0
            ? options.scheduler().feedback_period_ms()
            : 100);

    waypoint_reader_ =
        node_->CreateReader<::automsgs::msgs::geometry_msgs::PoseStamped>(
            kExplorationWaypoint,
            [this](const std::shared_ptr<
                   ::automsgs::msgs::geometry_msgs::PoseStamped>& pose) {
                HandleWaypoint(pose);
            });
    finished_reader_ = node_->CreateReader<::automsgs::msgs::std_msgs::Bool>(
        kExplorationFinished,
        [this](const std::shared_ptr<::automsgs::msgs::std_msgs::Bool>& msg) {
            HandleFinished(msg);
        });
    waypoint_reached_writer_ =
        node_->CreateWriter<::automsgs::msgs::std_msgs::Bool>(
            kExplorationWaypointReached);
    StartMonitor(period);
    return true;
}

void ExplorationTask::Shutdown() {
    StopMonitor();
    StopUnderlying();
    waypoint_reader_.reset();
    finished_reader_.reset();
    waypoint_reached_writer_.reset();
    TypedTaskAppBase::Shutdown();
    status_ = tp::EXPLORATION_STATUS_IDLE;
}

bool ExplorationTask::Cancel() {
    StopUnderlying();
    status_ = tp::EXPLORATION_STATUS_CANCELED;
    return TypedTaskAppBase::Cancel();
}

bool ExplorationTask::Pause() {
    if (!PublishNavigation(tp::NAV_CMD_PAUSE)) {
        return false;
    }
    status_ = tp::EXPLORATION_STATUS_PAUSED;
    return TypedTaskAppBase::Pause();
}

bool ExplorationTask::Resume() {
    if (!PublishNavigation(tp::NAV_CMD_RESUME)) {
        return false;
    }
    status_ = tp::EXPLORATION_STATUS_EXPLORING;
    return TypedTaskAppBase::Resume();
}

bool ExplorationTask::PublishNavigation(tp::NavigationCommand command) {
    if (!submit_navigation_) {
        return false;
    }
    tp::NavigationGoal goal;
    goal.set_command(command);
    return submit_navigation_(goal);
}

bool ExplorationTask::PublishMapping(tp::MapCommand command,
                                     const std::string& map_name) {
    if (!submit_mapping_) {
        return false;
    }
    tp::MappingGoal goal;
    goal.set_command(command);
    if (!map_name.empty()) {
        goal.set_map_name(map_name);
    }
    return submit_mapping_(goal);
}

bool ExplorationTask::PublishAreaWaypoint(
    const ::automsgs::msgs::geometry_msgs::Polygon& area) {
    if (area.points_size() == 0) {
        return false;
    }
    ::automsgs::msgs::geometry_msgs::PoseStamped waypoint;
    double x = 0., y = 0.;
    for (const auto& p : area.points()) {
        x += p.x();
        y += p.y();
    }
    const auto n = static_cast<double>(area.points_size());
    waypoint.mutable_pose()->mutable_position()->set_x(x / n);
    waypoint.mutable_pose()->mutable_position()->set_y(y / n);
    waypoint.mutable_pose()->mutable_orientation()->set_w(1.0);
    return NavigateToWaypoint(waypoint);
}

bool ExplorationTask::NavigateToWaypoint(
    const ::automsgs::msgs::geometry_msgs::PoseStamped& pose) {
    if (!submit_navigation_) {
        return false;
    }
    tp::NavigationGoal goal;
    goal.set_command(tp::NAV_CMD_START);
    goal.set_mode(tp::NAV_MODE_SINGLE_POSE);
    *goal.add_goals() = pose;
    if (!submit_navigation_(goal)) {
        return false;
    }
    last_waypoint_x_ = pose.pose().position().x();
    last_waypoint_y_ = pose.pose().position().y();
    nav_pending_.store(true);
    nav_was_active_.store(false);
    return true;
}

void ExplorationTask::StopUnderlying() {
    if (mapping_enabled_) {
        PublishMapping(tp::MAP_CMD_CANCEL);
        mapping_enabled_ = false;
    }
    PublishNavigation(tp::NAV_CMD_CANCEL);
    nav_pending_.store(false);
    nav_was_active_.store(false);
}

void ExplorationTask::StartMonitor(std::chrono::milliseconds period) {
    StopMonitor();
    monitor_running_.store(true);
    monitor_thread_ = std::thread([this, period]() {
        while (monitor_running_.load()) {
            if (is_navigation_active_ && is_navigation_active_()) {
                nav_was_active_.store(true);
            } else if (nav_pending_.load() && nav_was_active_.load()) {
                tp::NavigationResult result;
                if (get_navigation_result_ &&
                    get_navigation_result_(&result) &&
                    result.final_status() == tp::NAV_STATUS_SUCCEEDED) {
                    if (waypoint_reached_writer_) {
                        ::automsgs::msgs::std_msgs::Bool reached;
                        reached.set_data(true);
                        waypoint_reached_writer_->Write(reached);
                    }
                    nav_pending_.store(false);
                    nav_was_active_.store(false);
                    AINFO << "ExplorationTask: waypoint reached";
                }
            }
            std::this_thread::sleep_for(period);
        }
    });
}

void ExplorationTask::StopMonitor() {
    monitor_running_.store(false);
    if (monitor_thread_.joinable()) {
        monitor_thread_.join();
    }
}

void ExplorationTask::HandleWaypoint(
    const std::shared_ptr<::automsgs::msgs::geometry_msgs::PoseStamped>& pose) {
    if (!pose || Lifecycle() != TaskLifecycle::kRunning) {
        return;
    }
    const double x = pose->pose().position().x();
    const double y = pose->pose().position().y();
    if (nav_pending_.load() &&
        std::hypot(x - last_waypoint_x_, y - last_waypoint_y_) < 0.12) {
        return;
    }
    if (NavigateToWaypoint(*pose)) {
        status_ = tp::EXPLORATION_STATUS_EXPLORING;
        AINFO << "ExplorationTask: waypoint (" << x << ", " << y << ")";
    }
}

void ExplorationTask::HandleFinished(
    const std::shared_ptr<::automsgs::msgs::std_msgs::Bool>& message) {
    if (!message || !message->data()) {
        return;
    }
    PublishNavigation(tp::NAV_CMD_CANCEL);
    nav_pending_.store(false);
    status_ = tp::EXPLORATION_STATUS_SUCCEEDED;
    SetProgress(1.f, "exploration finished");
    SetLifecycle(TaskLifecycle::kSucceeded);
}

bool ExplorationTask::OnGoal(const tp::ExplorationGoal& goal) {
    using Command = tp::ExplorationCommand;
    switch (goal.command()) {
        case Command::EXPLORATION_CMD_START: {
            mapping_enabled_ = goal.enable_mapping();
            if (!goal.map_name().empty()) {
                map_name_ = goal.map_name();
            }
            coverage_target_ = goal.coverage_target();
            if (mapping_enabled_) {
                PublishMapping(tp::MAP_CMD_LOAD, map_name_);
            }
            if (goal.has_area() && goal.area().points_size() > 0) {
                PublishAreaWaypoint(goal.area());
            }
            status_ = tp::EXPLORATION_STATUS_EXPLORING;
            SetProgress(0.f, "exploring");
            SetLifecycle(TaskLifecycle::kRunning);
            return true;
        }
        case Command::EXPLORATION_CMD_PAUSE:
            return Pause();
        case Command::EXPLORATION_CMD_RESUME:
            return Resume();
        case Command::EXPLORATION_CMD_STOP:
        case Command::EXPLORATION_CMD_CANCEL:
            return Cancel();
        case Command::EXPLORATION_CMD_SET_AREA:
            if (goal.has_area()) {
                PublishAreaWaypoint(goal.area());
            }
            return true;
        case Command::EXPLORATION_CMD_SAVE_MAP: {
            const std::string name =
                goal.map_name().empty() ? map_name_ : goal.map_name();
            return PublishMapping(tp::MAP_CMD_SWITCH, name);
        }
        default:
            AWARN << "ExplorationTask: unsupported command "
                  << static_cast<int>(goal.command());
            return false;
    }
}

void ExplorationTask::FillFeedback(tp::ExplorationFeedback* feedback) const {
    feedback->set_status(status_);
    *feedback->mutable_progress() = progress_;
    feedback->set_map_name(map_name_);
}

void ExplorationTask::FillResult(tp::ExplorationResult* result) const {
    *result->mutable_result() = MakeTaskResult();
    result->set_final_status(status_);
    result->set_map_name(map_name_);
}

}  // namespace task
}  // namespace autonomy
