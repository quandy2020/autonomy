/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/control/controller_server.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include "autolink/action/simple_action_server.hpp"
#include "autolink/autolink.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/tasks/navigators/action_type.hpp"
#include "autonomy/transform/tf2/utils.h"

namespace autonomy {
namespace control {
namespace {

namespace task_proto = tasks::behavior_tree::task_proto;
using FollowPathServer =
    autolink::action::SimpleActionServer<tasks::behavior_tree::FollowPathActionTraits>;
using SpinServer =
    autolink::action::SimpleActionServer<tasks::behavior_tree::SpinActionTraits>;
using BackUpServer =
    autolink::action::SimpleActionServer<tasks::behavior_tree::BackUpActionTraits>;
using DriveOnHeadingServer = autolink::action::SimpleActionServer<
    tasks::behavior_tree::DriveOnHeadingActionTraits>;
using WaitServer =
    autolink::action::SimpleActionServer<tasks::behavior_tree::WaitActionTraits>;

commsgs::planning_msgs::Path PathFromGoal(
    const task_proto::FollowPathAction_Goal& goal) {
    if (goal.has_path()) {
        return commsgs::planning_msgs::FromProto(goal.path());
    }
    return {};
}

std::chrono::milliseconds ControlPeriod() {
    return std::chrono::milliseconds(100);
}

}  // namespace

struct ControllerServer::AutolinkActionServers {
    std::shared_ptr<FollowPathServer> follow_path;
    std::shared_ptr<SpinServer> spin;
    std::shared_ptr<BackUpServer> backup;
    std::shared_ptr<DriveOnHeadingServer> drive_on_heading;
    std::shared_ptr<WaitServer> wait;
};

bool ControllerServer::AttachAutolinkNode(std::shared_ptr<autolink::Node> node) {
    if (!node) {
        return false;
    }
    if (autolink_actions_) {
        return true;
    }
    autolink_actions_ = new AutolinkActionServers();
    ControllerServer* self = this;
    const auto period = ControlPeriod();

    autolink_actions_->follow_path = std::make_shared<FollowPathServer>(
        node, tasks::behavior_tree::kFollowPathActionName,
        [self, period]() {
            auto goal = self->autolink_actions_->follow_path->GetCurrentGoal();
            if (!goal) {
                return;
            }
            const auto path = PathFromGoal(*goal);
            if (!self->BeginFollowPath(
                    path, goal->controller_id(), goal->goal_checker_id(),
                    goal->progress_checker_id())) {
                auto result = std::make_shared<task_proto::FollowPathAction_Result>();
                result->set_error_code(task_proto::FOLLOW_PATH_UNKNOWN);
                self->autolink_actions_->follow_path->TerminateCurrent(result);
                return;
            }
            auto cancel = [&]() {
                return self->autolink_actions_->follow_path->IsCancelRequested();
            };
            while (autolink::OK() && !cancel()) {
                const auto tick = self->TickFollowPath(cancel);
                if (tick == FollowPathTickResult::Succeeded) {
                    auto result =
                        std::make_shared<task_proto::FollowPathAction_Result>();
                    result->set_error_code(task_proto::FOLLOW_PATH_NONE);
                    self->autolink_actions_->follow_path->SucceededCurrent(result);
                    return;
                }
                if (tick == FollowPathTickResult::Cancelled) {
                    auto result =
                        std::make_shared<task_proto::FollowPathAction_Result>();
                    result->set_error_code(task_proto::FOLLOW_PATH_UNKNOWN);
                    self->autolink_actions_->follow_path->TerminateCurrent(result);
                    return;
                }
                if (tick == FollowPathTickResult::Failed) {
                    auto result =
                        std::make_shared<task_proto::FollowPathAction_Result>();
                    result->set_error_code(task_proto::FOLLOW_PATH_UNKNOWN);
                    self->autolink_actions_->follow_path->TerminateCurrent(result);
                    return;
                }
                std::this_thread::sleep_for(period);
            }
        });

    autolink_actions_->spin = std::make_shared<SpinServer>(
        node, tasks::behavior_tree::kSpinActionName, [self, period]() {
            auto goal = self->autolink_actions_->spin->GetCurrentGoal();
            if (!goal) {
                return;
            }
            commsgs::geometry_msgs::PoseStamped pose;
            if (!self->GetRobotPose(pose)) {
                auto result = std::make_shared<task_proto::SpinAction_Result>();
                result->set_error_code(task_proto::SPIN_TF_ERROR);
                self->autolink_actions_->spin->TerminateCurrent(result);
                return;
            }
            const double current_yaw =
                transform::tf2::getYaw(pose.pose.orientation);
            RecoveryMotionCommand cmd;
            cmd.type = RecoveryMotionType::Spin;
            cmd.distance = goal->target_yaw() - current_yaw;
            cmd.speed = 0.5;
            cmd.time_allowance_sec =
                goal->has_time_allowance()
                    ? commsgs::builtin_interfaces::FromProto(goal->time_allowance())
                          .Seconds()
                    : 10.0;
            if (!self->BeginRecoveryMotion(cmd)) {
                auto result = std::make_shared<task_proto::SpinAction_Result>();
                result->set_error_code(task_proto::SPIN_UNKNOWN);
                self->autolink_actions_->spin->TerminateCurrent(result);
                return;
            }
            auto cancel = [&]() {
                return self->autolink_actions_->spin->IsCancelRequested();
            };
            while (autolink::OK() && !cancel()) {
                const auto tick = self->TickRecoveryMotion(cancel);
                if (tick == RecoveryTickResult::Succeeded) {
                    auto result = std::make_shared<task_proto::SpinAction_Result>();
                    result->set_error_code(task_proto::SPIN_NONE);
                    self->autolink_actions_->spin->SucceededCurrent(result);
                    return;
                }
                if (tick == RecoveryTickResult::Cancelled) {
                    auto result = std::make_shared<task_proto::SpinAction_Result>();
                    result->set_error_code(task_proto::SPIN_UNKNOWN);
                    self->autolink_actions_->spin->TerminateCurrent(result);
                    return;
                }
                if (tick == RecoveryTickResult::Failed) {
                    auto result = std::make_shared<task_proto::SpinAction_Result>();
                    result->set_error_code(task_proto::SPIN_UNKNOWN);
                    self->autolink_actions_->spin->TerminateCurrent(result);
                    return;
                }
                std::this_thread::sleep_for(period);
            }
        });

    auto run_recovery = [self, period](RecoveryMotionType type,
                                       const task_proto::BackUpAction_Goal& goal,
                                       autolink::action::SimpleActionServer<
                                           tasks::behavior_tree::BackUpActionTraits>*
                                           backup_server,
                                       autolink::action::SimpleActionServer<
                                           tasks::behavior_tree::DriveOnHeadingActionTraits>*
                                           drive_server) {
        commsgs::geometry_msgs::PoseStamped pose;
        if (!self->GetRobotPose(pose)) {
            return false;
        }
        if (!goal.has_target()) {
            return false;
        }
        const double dx = goal.target().x() - pose.pose.position.x;
        const double dy = goal.target().y() - pose.pose.position.y;
        RecoveryMotionCommand cmd;
        cmd.type = type;
        cmd.distance = std::hypot(dx, dy);
        cmd.speed = goal.speed();
        cmd.time_allowance_sec =
            goal.has_time_allowance()
                ? commsgs::builtin_interfaces::FromProto(goal.time_allowance())
                      .Seconds()
                : 10.0;
        cmd.disable_collision_checks = goal.disable_collision_checks();
        if (!self->BeginRecoveryMotion(cmd)) {
            return false;
        }
        auto cancel = [&]() {
            if (backup_server && backup_server->IsCancelRequested()) {
                return true;
            }
            if (drive_server && drive_server->IsCancelRequested()) {
                return true;
            }
            return false;
        };
        while (autolink::OK() && !cancel()) {
            const auto tick = self->TickRecoveryMotion(cancel);
            if (tick == RecoveryTickResult::Succeeded) {
                return true;
            }
            if (tick == RecoveryTickResult::Cancelled ||
                tick == RecoveryTickResult::Failed) {
                return false;
            }
            std::this_thread::sleep_for(period);
        }
        return false;
    };

    autolink_actions_->backup = std::make_shared<BackUpServer>(
        node, tasks::behavior_tree::kBackUpActionName,
        [self, period, run_recovery]() {
            auto goal = self->autolink_actions_->backup->GetCurrentGoal();
            if (!goal) {
                return;
            }
            const bool ok = run_recovery(
                RecoveryMotionType::BackUp, *goal,
                self->autolink_actions_->backup.get(), nullptr);
            auto result = std::make_shared<task_proto::BackUpAction_Result>();
            if (ok) {
                result->set_error_code(task_proto::BACK_UP_NONE);
                self->autolink_actions_->backup->SucceededCurrent(result);
            } else if (self->autolink_actions_->backup->IsCancelRequested()) {
                result->set_error_code(task_proto::BACK_UP_UNKNOWN);
                self->autolink_actions_->backup->TerminateCurrent(result);
            } else {
                result->set_error_code(task_proto::BACK_UP_UNKNOWN);
                self->autolink_actions_->backup->TerminateCurrent(result);
            }
        });

    autolink_actions_->drive_on_heading =
        std::make_shared<DriveOnHeadingServer>(
            node, tasks::behavior_tree::kDriveOnHeadingActionName,
            [self, period, run_recovery]() {
                auto goal =
                    self->autolink_actions_->drive_on_heading->GetCurrentGoal();
                if (!goal) {
                    return;
                }
                task_proto::BackUpAction_Goal shared_goal;
                *shared_goal.mutable_target() = goal->target();
                shared_goal.set_speed(goal->speed());
                if (goal->has_time_allowance()) {
                    *shared_goal.mutable_time_allowance() = goal->time_allowance();
                }
                shared_goal.set_disable_collision_checks(
                    goal->disable_collision_checks());
                const bool ok = run_recovery(
                    RecoveryMotionType::DriveOnHeading, shared_goal, nullptr,
                    self->autolink_actions_->drive_on_heading.get());
                auto result =
                    std::make_shared<task_proto::DriveOnHeadingAction_Result>();
                if (ok) {
                    result->set_error_code(task_proto::DRIVE_ON_HEADING_NONE);
                    self->autolink_actions_->drive_on_heading->SucceededCurrent(
                        result);
                } else if (self->autolink_actions_->drive_on_heading
                               ->IsCancelRequested()) {
                    result->set_error_code(task_proto::DRIVE_ON_HEADING_UNKNOWN);
                    self->autolink_actions_->drive_on_heading->TerminateCurrent(
                        result);
                } else {
                    result->set_error_code(task_proto::DRIVE_ON_HEADING_UNKNOWN);
                    self->autolink_actions_->drive_on_heading->TerminateCurrent(
                        result);
                }
            });

    autolink_actions_->wait = std::make_shared<WaitServer>(
        node, tasks::behavior_tree::kWaitActionName, [self]() {
            auto goal = self->autolink_actions_->wait->GetCurrentGoal();
            if (!goal || !goal->has_time()) {
                auto result = std::make_shared<task_proto::WaitAction_Result>();
                result->set_error_code(task_proto::WAIT_UNKNOWN);
                self->autolink_actions_->wait->TerminateCurrent(result);
                return;
            }
            const double wait_sec =
                commsgs::builtin_interfaces::FromProto(goal->time()).Seconds();
            const auto deadline = std::chrono::steady_clock::now() +
                                  std::chrono::duration<double>(wait_sec);
            while (autolink::OK()) {
                if (self->autolink_actions_->wait->IsCancelRequested()) {
                    auto result = std::make_shared<task_proto::WaitAction_Result>();
                    result->set_error_code(task_proto::WAIT_UNKNOWN);
                    self->autolink_actions_->wait->TerminateCurrent(result);
                    return;
                }
                if (std::chrono::steady_clock::now() >= deadline) {
                    auto result = std::make_shared<task_proto::WaitAction_Result>();
                    result->set_error_code(task_proto::WAIT_NONE);
                    self->autolink_actions_->wait->SucceededCurrent(result);
                    return;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        });

    AINFO << "ControllerServer autolink actions attached.";
    return true;
}

ControllerServer::~ControllerServer() {
    DetachAutolinkNode();
    Shutdown();
    AINFO << "Control server shutdown successfully.";
}

void ControllerServer::DetachAutolinkNode() {
    if (autolink_actions_) {
        if (autolink_actions_->follow_path) {
            autolink_actions_->follow_path->Deactivate();
        }
        if (autolink_actions_->spin) {
            autolink_actions_->spin->Deactivate();
        }
        if (autolink_actions_->wait) {
            autolink_actions_->wait->Deactivate();
        }
        if (autolink_actions_->backup) {
            autolink_actions_->backup->Deactivate();
        }
        if (autolink_actions_->drive_on_heading) {
            autolink_actions_->drive_on_heading->Deactivate();
        }
        delete autolink_actions_;
        autolink_actions_ = nullptr;
    }
}

}  // namespace control
}  // namespace autonomy
