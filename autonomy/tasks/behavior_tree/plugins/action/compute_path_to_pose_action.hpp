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

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "autonomy/tasks/behavior_tree/json_utils.hpp"
#include "behaviortree_cpp/json_export.h"
#include "autonomy/tasks/proto/bt_action.pb.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

/**
 * @brief In-process ComputePathToPose: calls PlannerServer::GetPlan via TaskContext.
 */
class ComputePathToPoseAction : public BtStatefulActionNode
{
public:
    ComputePathToPoseAction(const std::string& xml_tag_name,
                            const BT::NodeConfiguration& conf);

    static BT::PortsList providedPorts() {
        BT::RegisterJsonDefinition<commsgs::planning_msgs::Path>();
        BT::RegisterJsonDefinition<commsgs::geometry_msgs::PoseStamped>();

        return {
            BT::InputPort<commsgs::geometry_msgs::PoseStamped>(
                "goal", "Destination to plan to"),
            BT::InputPort<commsgs::geometry_msgs::PoseStamped>(
                "start",
                "Planner start pose when use_start is true"),
            BT::InputPort<bool>("use_start",
                                "Use the provided start pose instead of the "
                                "current robot pose"),
            BT::InputPort<std::string>(
                "planner_id", "",
                "Planner plugin id; empty uses TaskContext default"),
            BT::OutputPort<commsgs::planning_msgs::Path>("path",
                                                         "Planned path"),
            BT::OutputPort<int32_t>("error_code_id",
                                    "Compute path error code"),
            BT::OutputPort<std::string>("error_msg",
                                        "Compute path error message"),
        };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void setFailure(int32_t code, const std::string& msg);

    commsgs::geometry_msgs::PoseStamped start_pose_;
    commsgs::geometry_msgs::PoseStamped goal_pose_;
    std::string planner_id_;
    bool poses_ready_{false};
};

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
