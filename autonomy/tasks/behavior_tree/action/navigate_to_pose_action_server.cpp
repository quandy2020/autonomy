/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/behavior_tree/action/navigate_to_pose_action_server.hpp"

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
    autolink::action::SimpleActionServer<NavigateToPoseActionTraits>;
using GoalPtr = std::shared_ptr<const NavigateToPoseActionTraits::Goal>;
using FeedbackPtr = std::shared_ptr<NavigateToPoseActionTraits::Feedback>;
using ResultPtr = std::shared_ptr<NavigateToPoseActionTraits::Result>;

}  // namespace

NavigateToPoseActionServer::NavigateToPoseActionServer(BtNavigator* navigator)
    : navigator_(navigator) {}

bool NavigateToPoseActionServer::Activate(std::shared_ptr<autolink::Node> node) {
    if (!navigator_ || !node) {
        return false;
    }
    node_ = std::move(node);
    server_ = std::make_shared<ActionServer>(
        node_, kNavigatorNavigateToPose,
        [this]() { ExecuteCallback(); }, nullptr,
        std::chrono::milliseconds(500));
    AINFO << "NavigateToPose autolink action server active on '"
          << kNavigatorNavigateToPose << "'";
    return true;
}

void NavigateToPoseActionServer::Deactivate() {
    if (server_) {
        server_->Deactivate();
        server_.reset();
    }
    node_.reset();
}

bool NavigateToPoseActionServer::IsActive() const {
    return server_ && server_->IsServerActive();
}

void NavigateToPoseActionServer::ExecuteCallback() {
    if (!navigator_ || !server_) {
        return;
    }

    GoalPtr goal = server_->GetCurrentGoal();
    if (!goal) {
        AERROR << "NavigateToPoseActionServer: no current goal.";
        return;
    }

    auto run_once = [this](GoalPtr g) -> bool {
        const auto pose = PoseFromActionGoal(*g);
        const std::string bt_file = g->behavior_tree();
        if (!navigator_->StartNavigateToPose(pose, bt_file)) {
            ResultPtr result = std::make_shared<NavigateToPoseActionTraits::Result>();
            result->set_error_code(
                proto::NAV_TO_POSE_FAILED_TO_LOAD_BEHAVIOR_TREE);
            result->set_error_msg("Failed to start navigate_to_pose.");
            server_->TerminateCurrent(result);
            return false;
        }
        return true;
    };

    if (!run_once(goal)) {
        return;
    }

    commsgs::geometry_msgs::PoseStamped target = PoseFromActionGoal(*goal);
    const auto loop_ms = std::chrono::milliseconds(
        navigator_->GetContext() && navigator_->GetContext()->options.bt_loop_duration() > 0
            ? navigator_->GetContext()->options.bt_loop_duration()
            : 10);

    while (autolink::OK() && navigator_->IsActive()) {
        if (server_->IsCancelRequested()) {
            navigator_->Cancel();
            ResultPtr result = std::make_shared<NavigateToPoseActionTraits::Result>();
            result->set_error_code(proto::NAV_TO_POSE_CANCELED);
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
            target = PoseFromActionGoal(*goal);
        }

        if (auto ctx = navigator_->GetContext()) {
            commsgs::geometry_msgs::PoseStamped current;
            if (utils::getGlobalRobotPose(
                    current, ctx->tf_buffer, ctx->controller->GetOdomSmoother(),
                    ctx->options.global_frame(), ctx->options.robot_base_frame())) {
                FeedbackPtr fb = std::make_shared<NavigateToPoseActionTraits::Feedback>(
                    MakeNavigateToPoseFeedback(current, target,
                                             ctx->number_recoveries,
                                             ctx->navigation_start));
                server_->PublishFeedback(fb);
            }
        }

        std::this_thread::sleep_for(loop_ms);
    }

    ResultPtr result = std::make_shared<NavigateToPoseActionTraits::Result>();
    switch (navigator_->GetLastBtStatus()) {
        case BtStatus::SUCCEEDED:
            result->set_error_code(proto::NAV_TO_POSE_NONE);
            server_->SucceededCurrent(result);
            break;
        case BtStatus::CANCELED:
            result->set_error_code(proto::NAV_TO_POSE_CANCELED);
            server_->TerminateCurrent(result);
            break;
        default:
            result->set_error_code(proto::NAV_TO_POSE_UNKNOWN);
            result->set_error_msg("Behavior tree navigation failed.");
            server_->TerminateCurrent(result);
            break;
    }
}

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
