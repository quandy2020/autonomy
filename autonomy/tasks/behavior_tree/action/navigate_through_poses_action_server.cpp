/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/behavior_tree/action/navigate_through_poses_action_server.hpp"

#include <chrono>
#include <thread>

#include "autolink/action/simple_action_server.hpp"
#include "autolink/autolink.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/tasks/behavior_tree/action/action_msg_conversions.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_engine.hpp"
#include "autonomy/tasks/behavior_tree/error_codes.hpp"
#include "autonomy/tasks/behavior_tree/navigator/bt_navigator.hpp"
#include "autonomy/tasks/constants.hpp"
#include "autonomy/tasks/utils/robot_utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

namespace {

using ActionServer =
    autolink::action::SimpleActionServer<NavigateThroughPosesActionTraits>;
using GoalPtr = std::shared_ptr<const NavigateThroughPosesActionTraits::Goal>;
using FeedbackPtr =
    std::shared_ptr<NavigateThroughPosesActionTraits::Feedback>;
using ResultPtr = std::shared_ptr<NavigateThroughPosesActionTraits::Result>;

}  // namespace

NavigateThroughPosesActionServer::NavigateThroughPosesActionServer(
    BtNavigator* navigator)
    : navigator_(navigator) {}

bool NavigateThroughPosesActionServer::Activate(
    std::shared_ptr<autolink::Node> node) {
    if (!navigator_ || !node) {
        return false;
    }
    node_ = std::move(node);
    server_ = std::make_shared<ActionServer>(
        node_, kNavigatorNavigateThroughPoses,
        [this]() { ExecuteCallback(); }, nullptr,
        std::chrono::milliseconds(500));
    AINFO << "NavigateThroughPoses autolink action server active on '"
          << kNavigatorNavigateThroughPoses << "'";
    return true;
}

void NavigateThroughPosesActionServer::Deactivate() {
    if (server_) {
        server_->Deactivate();
        server_.reset();
    }
    node_.reset();
}

bool NavigateThroughPosesActionServer::IsActive() const {
    return server_ && server_->IsServerActive();
}

void NavigateThroughPosesActionServer::ExecuteCallback() {
    if (!navigator_ || !server_) {
        return;
    }

    GoalPtr goal = server_->GetCurrentGoal();
    if (!goal) {
        return;
    }

    auto run_once = [this](GoalPtr g) -> bool {
        const auto poses = PosesFromActionGoal(*g);
        if (poses.empty()) {
            ResultPtr result =
                std::make_shared<NavigateThroughPosesActionTraits::Result>();
            result->set_error_code(proto::NAV_THROUGH_POSES_INVALID_GOAL);
            result->set_error_msg("Empty goals list.");
            server_->TerminateCurrent(result);
            return false;
        }
        if (!navigator_->StartNavigateThroughPoses(poses, g->behavior_tree())) {
            ResultPtr result =
                std::make_shared<NavigateThroughPosesActionTraits::Result>();
            result->set_error_code(
                proto::NAV_THROUGH_POSES_FAILED_TO_LOAD_BEHAVIOR_TREE);
            result->set_error_msg("Failed to start navigate_through_poses.");
            server_->TerminateCurrent(result);
            return false;
        }
        return true;
    };

    if (!run_once(goal)) {
        return;
    }

    std::vector<commsgs::geometry_msgs::PoseStamped> targets =
        PosesFromActionGoal(*goal);
    const auto loop_ms = std::chrono::milliseconds(
        navigator_->GetContext() &&
                navigator_->GetContext()->options.bt_loop_duration() > 0
            ? navigator_->GetContext()->options.bt_loop_duration()
            : 10);

    while (autolink::OK() && navigator_->IsActive()) {
        if (server_->IsCancelRequested()) {
            navigator_->Cancel();
            ResultPtr result =
                std::make_shared<NavigateThroughPosesActionTraits::Result>();
            result->set_error_code(proto::NAV_THROUGH_POSES_CANCELED);
            server_->TerminateCurrent(result);
            return;
        }

        if (server_->IsPreemptRequested()) {
            navigator_->Cancel();
            for (int i = 0; i < 50 && navigator_->IsActive(); ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            goal = server_->AcceptPendingGoal();
            if (!goal) {
                return;
            }
            if (!run_once(goal)) {
                return;
            }
            targets = PosesFromActionGoal(*goal);
        }

        if (auto ctx = navigator_->GetContext()) {
            commsgs::geometry_msgs::PoseStamped current;
            if (utils::getGlobalRobotPose(
                    current, ctx->tf_buffer, ctx->controller->GetOdomSmoother(),
                    ctx->options.global_frame(), ctx->options.robot_base_frame())) {
                FeedbackPtr fb =
                    std::make_shared<NavigateThroughPosesActionTraits::Feedback>(
                        MakeNavigateThroughPosesFeedback(
                            current, targets, ctx->number_recoveries,
                            ctx->navigation_start));
                server_->PublishFeedback(fb);
            }
        }

        std::this_thread::sleep_for(loop_ms);
    }

    ResultPtr result =
        std::make_shared<NavigateThroughPosesActionTraits::Result>();
    switch (navigator_->GetLastBtStatus()) {
        case BtStatus::SUCCEEDED:
            result->set_error_code(proto::NAV_THROUGH_POSES_NONE);
            server_->SucceededCurrent(result);
            break;
        case BtStatus::CANCELED:
            result->set_error_code(proto::NAV_THROUGH_POSES_CANCELED);
            server_->TerminateCurrent(result);
            break;
        default:
            result->set_error_code(proto::NAV_THROUGH_POSES_UNKNOWN);
            result->set_error_msg("Behavior tree navigation failed.");
            server_->TerminateCurrent(result);
            break;
    }
}

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
