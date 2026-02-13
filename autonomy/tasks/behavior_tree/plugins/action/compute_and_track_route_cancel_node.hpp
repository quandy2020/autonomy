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

#include "behaviortree_cpp/action_node.h"

#include "autonomy/tasks/behavior_tree/behavior_tree_cancel_action_node.hpp"
#include "autonomy/tasks/navigator/proto/action.pb.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

/**
 * @brief A nav2_behavior_tree::BtActionNode class
 * that wraps nav2_msgs::action::ComputeAndTrackRoute cancellation
 */
class ComputeAndTrackRouteCancel : public BtCancelActionNode<proto::ComputeAndTrackRouteAction>
{
public:
    /**
     * @brief A constructor for nav2_behavior_tree::ComputeAndTrackRouteCancel
     * @param xml_tag_name Name for the XML tag for this node
     * @param action_name Action name this node creates a client for
     * @param conf BT node configuration
     */
    ComputeAndTrackRouteCancel(const std::string& xml_tag_name, const std::string& action_name,
                               const BT::NodeConfiguration& conf);

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing basic ports along with node-specific
     * ports
     */
    static BT::PortsList providedPorts() {
        return providedBasicPorts({});
    }
};

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
