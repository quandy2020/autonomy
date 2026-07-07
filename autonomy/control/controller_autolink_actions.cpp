/*
 * Copyright 2026 The Openbot Authors
 *
 * Minimal autolink action servers for ControllerServer (task BT RPC).
 */

#include "autonomy/control/controller_server.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include "autolink/action/simple_action_server.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/commsgs/proto/error_code.pb.h"
#include "autonomy/commsgs/proto/nav_msgs.pb.h"
#include "autonomy/common/logging.hpp"
#include "autonomy/control/constants.hpp"
#include "autonomy/map/costmap_2d/utils/geometry_utils.hpp"

namespace autonomy {
namespace control {
namespace {

namespace nav_proto = commsgs::proto::nav_msgs;
namespace err_proto = commsgs::proto::error_code;

using FollowPathServer =
    autolink::action::SimpleActionServer<nav_proto::FollowPathAction>;
using SpinServer = autolink::action::SimpleActionServer<nav_proto::SpinAction>;
using BackUpServer =
    autolink::action::SimpleActionServer<nav_proto::BackUpAction>;
using WaitServer = autolink::action::SimpleActionServer<nav_proto::WaitAction>;

constexpr auto kControlPeriod = std::chrono::milliseconds(100);

commsgs::planning_msgs::Path PathFromGoal(
    const nav_proto::FollowPathAction::Goal& goal)
{
    if (goal.has_path()) {
        return commsgs::planning_msgs::FromProto(goal.path());
    }
    return {};
}

double DistanceToPathGoal(const commsgs::planning_msgs::Path& path,
                          const commsgs::geometry_msgs::PoseStamped& pose)
{
    if (path.poses.empty()) {
        return 0.0;
    }
    const auto& goal = path.poses.back().pose.position;
    const auto& current = pose.pose.position;
    return map::costmap_2d::utils::euclidean_distance(goal, current);
}

}  // namespace

struct ControllerServer::AutolinkActionServers {
    std::shared_ptr<FollowPathServer> follow_path;
    std::shared_ptr<SpinServer> spin;
    std::shared_ptr<BackUpServer> backup;
    std::shared_ptr<WaitServer> wait;
};

bool ControllerServer::AttachAutolinkNode(std::shared_ptr<autolink::Node> node)
{
    if (!node) {
        return false;
    }
    if (autolink_actions_) {
        return true;
    }

    autolink_actions_ = new AutolinkActionServers();
    ControllerServer* self = this;

    autolink_actions_->follow_path = std::make_shared<FollowPathServer>(
        node, kFollowPathActionName, [self]() {
            auto& server = self->autolink_actions_->follow_path;
            auto goal = server->GetCurrentGoal();
            if (!goal) {
                return;
            }

            const auto path = PathFromGoal(*goal);
            if (path.poses.empty()) {
                auto result =
                    std::make_shared<nav_proto::FollowPathAction::Result>();
                result->set_error_code(err_proto::FOLLOW_PATH_INVALID_PATH);
                server->TerminateCurrent(result);
                return;
            }

            self->current_path_ = path;
            self->follow_path_active_ = true;

            auto cancel = [&]() { return server->IsCancelRequested(); };
            while (autolink::OK() && !cancel()) {
                commsgs::geometry_msgs::PoseStamped pose;
                if (!self->GetRobotPose(pose)) {
                    std::this_thread::sleep_for(kControlPeriod);
                    continue;
                }

                const double distance = DistanceToPathGoal(path, pose);
                auto feedback =
                    std::make_shared<nav_proto::FollowPathAction::Feedback>();
                feedback->set_distance_to_goal(static_cast<float>(distance));
                server->PublishFeedback(feedback);

                if (distance <= self->goal_reached_tolerance_) {
                    auto result =
                        std::make_shared<nav_proto::FollowPathAction::Result>();
                    result->set_error_code(err_proto::FOLLOW_PATH_NONE);
                    self->OnGoalExit();
                    server->SucceededCurrent(result);
                    return;
                }

                self->ComputeAndPublishVelocity();
                std::this_thread::sleep_for(kControlPeriod);
            }

            if (server->IsCancelRequested()) {
                auto result =
                    std::make_shared<nav_proto::FollowPathAction::Result>();
                result->set_error_code(err_proto::FOLLOW_PATH_UNKNOWN);
                self->OnGoalExit();
                server->TerminateCurrent(result);
            }
        });

    autolink_actions_->spin = std::make_shared<SpinServer>(
        node, kSpinActionName, [self]() {
            auto& server = self->autolink_actions_->spin;
            auto goal = server->GetCurrentGoal();
            if (!goal) {
                return;
            }

            const double allowance_sec =
                goal->has_time_allowance()
                    ? commsgs::builtin_interfaces::FromProto(goal->time_allowance())
                          .Seconds()
                    : 10.0;
            const auto deadline = std::chrono::steady_clock::now() +
                                  std::chrono::duration<double>(allowance_sec);

            while (autolink::OK()) {
                if (server->IsCancelRequested()) {
                    auto result = std::make_shared<nav_proto::SpinAction::Result>();
                    result->set_error_code(err_proto::SPIN_UNKNOWN);
                    server->TerminateCurrent(result);
                    return;
                }
                if (std::chrono::steady_clock::now() >= deadline) {
                    auto result = std::make_shared<nav_proto::SpinAction::Result>();
                    result->set_error_code(err_proto::SPIN_NONE);
                    server->SucceededCurrent(result);
                    return;
                }
                std::this_thread::sleep_for(kControlPeriod);
            }
        });

    autolink_actions_->backup = std::make_shared<BackUpServer>(
        node, kBackUpActionName, [self]() {
            auto& server = self->autolink_actions_->backup;
            auto goal = server->GetCurrentGoal();
            if (!goal) {
                return;
            }

            const double speed = goal->speed() > 0.f ? goal->speed() : 0.1f;
            const double dist =
                goal->has_target()
                    ? std::hypot(goal->target().x(), goal->target().y())
                    : 0.3;
            const double allowance_sec =
                goal->has_time_allowance()
                    ? commsgs::builtin_interfaces::FromProto(goal->time_allowance())
                          .Seconds()
                    : std::max(10.0, dist / speed);
            const auto deadline = std::chrono::steady_clock::now() +
                                  std::chrono::duration<double>(allowance_sec);

            while (autolink::OK()) {
                if (server->IsCancelRequested()) {
                    auto result =
                        std::make_shared<nav_proto::BackUpAction::Result>();
                    result->set_error_code(err_proto::BACK_UP_UNKNOWN);
                    server->TerminateCurrent(result);
                    return;
                }
                if (std::chrono::steady_clock::now() >= deadline) {
                    auto result =
                        std::make_shared<nav_proto::BackUpAction::Result>();
                    result->set_error_code(err_proto::BACK_UP_NONE);
                    server->SucceededCurrent(result);
                    return;
                }
                std::this_thread::sleep_for(kControlPeriod);
            }
        });

    autolink_actions_->wait = std::make_shared<WaitServer>(
        node, kWaitActionName, [self]() {
            auto& server = self->autolink_actions_->wait;
            auto goal = server->GetCurrentGoal();
            if (!goal || !goal->has_time()) {
                auto result = std::make_shared<nav_proto::WaitAction::Result>();
                result->set_error_code(err_proto::WAIT_UNKNOWN);
                server->TerminateCurrent(result);
                return;
            }

            const double wait_sec =
                commsgs::builtin_interfaces::FromProto(goal->time()).Seconds();
            const auto deadline = std::chrono::steady_clock::now() +
                                  std::chrono::duration<double>(wait_sec);

            while (autolink::OK()) {
                if (server->IsCancelRequested()) {
                    auto result = std::make_shared<nav_proto::WaitAction::Result>();
                    result->set_error_code(err_proto::WAIT_UNKNOWN);
                    server->TerminateCurrent(result);
                    return;
                }
                if (std::chrono::steady_clock::now() >= deadline) {
                    auto result = std::make_shared<nav_proto::WaitAction::Result>();
                    result->set_error_code(err_proto::WAIT_NONE);
                    server->SucceededCurrent(result);
                    return;
                }
                std::this_thread::sleep_for(kControlPeriod);
            }
        });

    AINFO << "ControllerServer autolink actions attached.";
    return true;
}

void ControllerServer::DetachAutolinkNode()
{
    if (!autolink_actions_) {
        return;
    }

    if (autolink_actions_->follow_path) {
        autolink_actions_->follow_path->Deactivate();
    }
    if (autolink_actions_->spin) {
        autolink_actions_->spin->Deactivate();
    }
    if (autolink_actions_->backup) {
        autolink_actions_->backup->Deactivate();
    }
    if (autolink_actions_->wait) {
        autolink_actions_->wait->Deactivate();
    }

    delete autolink_actions_;
    autolink_actions_ = nullptr;
}

}  // namespace control
}  // namespace autonomy
