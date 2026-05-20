/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/behavior_tree/plugins/action/drive_on_heading_action.hpp"

#include "autonomy/control/controller_server.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

DriveOnHeadingAction::DriveOnHeadingAction(const std::string& xml_tag_name,
                                           const BT::NodeConfiguration& conf)
    : BtStatefulActionNode(xml_tag_name, conf) {}

BT::NodeStatus DriveOnHeadingAction::onStart() {
    auto ctx = taskContext();
    if (!ctx || !ctx->controller) {
        setOutput("error_code_id",
                  static_cast<int32_t>(
                      proto::DriveOnHeadingErrorCode::DRIVE_ON_HEADING_ERROR_UNKNOWN));
        setOutput("error_msg", std::string("Controller unavailable"));
        return BT::NodeStatus::FAILURE;
    }

    double dist = 0.15;
    double speed = 0.025;
    double time_allowance = 10.0;
    getInput("dist_to_travel", dist);
    getInput("speed", speed);
    getInput("time_allowance", time_allowance);

    control::ControllerServer::RecoveryMotionCommand cmd;
    cmd.type = control::ControllerServer::RecoveryMotionType::DriveOnHeading;
    cmd.distance = std::abs(dist);
    cmd.speed = std::abs(speed);
    cmd.time_allowance_sec = time_allowance;

    if (!ctx->controller->BeginRecoveryMotion(cmd)) {
        setOutput("error_code_id",
                  static_cast<int32_t>(
                      proto::DriveOnHeadingErrorCode::DRIVE_ON_HEADING_ERROR_UNKNOWN));
        setOutput("error_msg", std::string("Failed to start drive on heading"));
        return BT::NodeStatus::FAILURE;
    }

    started_motion_ = true;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DriveOnHeadingAction::onRunning() {
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
                  static_cast<int32_t>(
                      proto::DriveOnHeadingErrorCode::DRIVE_ON_HEADING_ERROR_NONE));
        setOutput("error_msg", std::string(""));
        return BT::NodeStatus::SUCCESS;
    }
    setOutput("error_code_id",
              static_cast<int32_t>(
                  proto::DriveOnHeadingErrorCode::DRIVE_ON_HEADING_ERROR_TIMEOUT));
    setOutput("error_msg", std::string("DriveOnHeading failed or timed out"));
    return result == control::ControllerServer::RecoveryTickResult::Cancelled
               ? BT::NodeStatus::SUCCESS
               : BT::NodeStatus::FAILURE;
}

void DriveOnHeadingAction::onHalted() {
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
        autonomy::tasks::behavior_tree::plugins::action::DriveOnHeadingAction>(
        "DriveOnHeading");
}
