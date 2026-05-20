/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/behavior_tree/plugins/action/spin_action.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/control/controller_server.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

SpinAction::SpinAction(const std::string& xml_tag_name,
                       const BT::NodeConfiguration& conf)
    : BtStatefulActionNode(xml_tag_name, conf) {}

BT::NodeStatus SpinAction::onStart() {
    auto ctx = taskContext();
    if (!ctx || !ctx->controller) {
        setOutput("error_code_id",
                  static_cast<int32_t>(proto::SpinErrorCode::SPIN_ERROR_UNKNOWN));
        setOutput("error_msg", std::string("Controller unavailable"));
        return BT::NodeStatus::FAILURE;
    }

    double spin_dist = 1.57;
    double time_allowance = 10.0;
    getInput("spin_dist", spin_dist);
    getInput("time_allowance", time_allowance);
    getInput("is_recovery", is_recovery_);

    control::ControllerServer::RecoveryMotionCommand cmd;
    cmd.type = control::ControllerServer::RecoveryMotionType::Spin;
    cmd.distance = spin_dist;
    cmd.speed = 0.5;
    cmd.time_allowance_sec = time_allowance;

    if (!ctx->controller->BeginRecoveryMotion(cmd)) {
        setOutput("error_code_id",
                  static_cast<int32_t>(proto::SpinErrorCode::SPIN_ERROR_UNKNOWN));
        setOutput("error_msg", std::string("Failed to start spin"));
        return BT::NodeStatus::FAILURE;
    }

    if (is_recovery_) {
        incrementRecoveryCount();
    }
    started_motion_ = true;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SpinAction::onRunning() {
    auto ctx = taskContext();
    if (!ctx || !ctx->controller) {
        return BT::NodeStatus::FAILURE;
    }

    const auto result = ctx->controller->TickRecoveryMotion(
        ctx->CancelChecker());
    if (result == control::ControllerServer::RecoveryTickResult::Running) {
        return BT::NodeStatus::RUNNING;
    }
    if (result == control::ControllerServer::RecoveryTickResult::Succeeded) {
        setOutput("error_code_id",
                  static_cast<int32_t>(proto::SpinErrorCode::SPIN_ERROR_NONE));
        setOutput("error_msg", std::string(""));
        return BT::NodeStatus::SUCCESS;
    }
    setOutput("error_code_id",
              static_cast<int32_t>(
                  result == control::ControllerServer::RecoveryTickResult::Cancelled
                      ? proto::SpinErrorCode::SPIN_ERROR_NONE
                      : proto::SpinErrorCode::SPIN_ERROR_TIMEOUT));
    setOutput("error_msg", std::string("Spin recovery failed or timed out"));
    return result == control::ControllerServer::RecoveryTickResult::Cancelled
               ? BT::NodeStatus::SUCCESS
               : BT::NodeStatus::FAILURE;
}

void SpinAction::onHalted() {
    if (started_motion_) {
        if (auto ctx = taskContext()) {
            if (ctx->controller) {
                ctx->controller->EndRecoveryMotion();
            }
        }
        started_motion_ = false;
    }
}

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<
        autonomy::tasks::behavior_tree::plugins::action::SpinAction>("Spin");
}
