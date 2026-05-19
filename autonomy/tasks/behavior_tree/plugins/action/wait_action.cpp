/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autonomy/tasks/behavior_tree/plugins/action/wait_action.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

WaitAction::WaitAction(const std::string& xml_tag_name,
                       const std::string& action_name,
                       const BT::NodeConfiguration& conf)
    : BtActionNode<proto::WaitAction>(xml_tag_name, action_name, conf) {}

void WaitAction::on_tick() {
    double duration;
    if (!getInput("wait_duration", duration)) {
        AWARN << "wait_duration port is missing. Assuming 0.0 seconds.";
        duration = 0.0;
    }
    if (duration <= 0) {
        AWARN << "Wait duration is negative or zero (" << duration
              << "). Setting to positive.";
        duration *= -1;
    }

    auto duration_obj =
        commsgs::builtin_interfaces::Duration::FromSeconds(duration);
    int64_t ns = duration_obj.Nanoseconds();
    int32_t sec = static_cast<int32_t>(ns / 1'000'000'000LL);
    uint32_t nanosec = static_cast<uint32_t>(ns % 1'000'000'000LL);
    goal_.mutable_time()->mutable_stamp()->set_sec(sec);
    goal_.mutable_time()->mutable_stamp()->set_nanosec(nanosec);
}

BT::NodeStatus WaitAction::on_success() {
    setOutput("error_code_id",
              static_cast<int32_t>(proto::WaitErrorCode::WAIT_ERROR_NONE));
    setOutput("error_msg", std::string(""));
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus WaitAction::on_aborted() {
    if (result_.result) {
        setOutput("error_code_id",
                  static_cast<int32_t>(result_.result->error_code()));
        setOutput("error_msg", result_.result->error_msg());
    } else {
        setOutput(
            "error_code_id",
            static_cast<int32_t>(proto::WaitErrorCode::WAIT_ERROR_UNKNOWN));
        setOutput("error_msg", std::string("Unknown error"));
    }
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus WaitAction::on_cancelled() {
    setOutput("error_code_id",
              static_cast<int32_t>(proto::WaitErrorCode::WAIT_ERROR_NONE));
    setOutput("error_msg", std::string(""));
    return BT::NodeStatus::SUCCESS;
}

void WaitAction::on_timeout() {
    setOutput("error_code_id",
              static_cast<int32_t>(proto::WaitErrorCode::WAIT_ERROR_TIMEOUT));
    setOutput("error_msg",
              std::string("Behavior Tree action client timed out waiting."));
}

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    BT::NodeBuilder builder = [](const std::string& name,
                                 const BT::NodeConfiguration& config) {
        return std::make_unique<
            autonomy::tasks::behavior_tree::plugins::action::WaitAction>(
            name, "wait", config);
    };

    factory.registerBuilder<
        autonomy::tasks::behavior_tree::plugins::action::WaitAction>("Wait",
                                                                     builder);
}
