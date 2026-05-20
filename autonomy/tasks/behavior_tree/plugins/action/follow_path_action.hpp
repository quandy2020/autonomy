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

#pragma once

#include <string>

#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/tasks/behavior_tree/bt_stateful_action_node.hpp"
#include "autonomy/tasks/behavior_tree/json_utils.hpp"
#include "autonomy/tasks/navigator/proto/action.pb.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

/**
 * @brief In-process FollowPath: one ControllerServer control step per BT tick.
 */
class FollowPathAction : public BtStatefulActionNode
{
public:
    FollowPathAction(const std::string& xml_tag_name,
                     const BT::NodeConfiguration& conf);

    static BT::PortsList providedPorts() {
        BT::RegisterJsonDefinition<commsgs::planning_msgs::Path>();
        return {
            BT::InputPort<commsgs::planning_msgs::Path>("path",
                                                        "Path to follow"),
            BT::InputPort<std::string>("controller_id", ""),
            BT::InputPort<std::string>("goal_checker_id", ""),
            BT::InputPort<std::string>("progress_checker_id", ""),
            BT::OutputPort<int32_t>("error_code_id",
                                    "The follow path error code"),
            BT::OutputPort<std::string>("error_msg",
                                        "The follow path error msg"),
        };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void setFailure(int32_t code, const std::string& msg);
    void maybeUpdatePathFromPorts();

    commsgs::planning_msgs::Path path_;
    std::string controller_id_;
    std::string goal_checker_id_;
    std::string progress_checker_id_;
    bool follow_started_{false};
};

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
