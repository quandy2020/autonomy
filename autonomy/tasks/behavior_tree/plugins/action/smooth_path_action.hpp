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
#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "autonomy/tasks/behavior_tree/json_utils.hpp"
#include "autonomy/tasks/proto/bt_action.pb.h"
#include "behaviortree_cpp/json_export.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

/**
 * @brief In-process SmoothPath via SmootherServer::SmoothPath.
 */
class SmoothPathAction : public BtStatefulActionNode
{
public:
    SmoothPathAction(const std::string& xml_tag_name,
                     const BT::NodeConfiguration& conf);

    static BT::PortsList providedPorts() {
        BT::RegisterJsonDefinition<commsgs::planning_msgs::Path>();

        return {
            BT::InputPort<commsgs::planning_msgs::Path>(
                "unsmoothed_path", "Path to be smoothed"),
            BT::InputPort<double>("max_smoothing_duration", 3.0,
                                    "Maximum smoothing duration"),
            BT::InputPort<bool>(
                "check_for_collisions", false,
                "If true collision check will be performed after smoothing"),
            BT::InputPort<std::string>("smoother_id", "",
                                       "Smoother plugin id"),
            BT::OutputPort<commsgs::planning_msgs::Path>(
                "smoothed_path", "Path smoothed by SmootherServer"),
            BT::OutputPort<double>("smoothing_duration",
                                   "Time taken to smooth path"),
            BT::OutputPort<bool>(
                "was_completed",
                "True if smoothing was not interrupted by time limit"),
            BT::OutputPort<int32_t>("error_code_id",
                                    "The smooth path error code"),
            BT::OutputPort<std::string>("error_msg",
                                        "The smooth path error msg"),
        };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void setFailure(int32_t code, const std::string& msg);

    commsgs::planning_msgs::Path input_path_;
    std::string smoother_id_;
    double max_smoothing_duration_{3.0};
    bool check_for_collisions_{false};
    bool input_ready_{false};
};

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
