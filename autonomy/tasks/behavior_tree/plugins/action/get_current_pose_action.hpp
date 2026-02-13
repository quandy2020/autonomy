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

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/transform/buffer.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

/**
 * @brief A BT::ActionNodeBase to shorten path by some distance
 */
class GetCurrentPoseAction : public BT::ActionNodeBase
{
public:
    /**
     * @brief A nav2_behavior_tree::GetCurrentPoseAction constructor
     * @param xml_tag_name Name for the XML tag for this node
     * @param conf BT node configuration
     */
    GetCurrentPoseAction(const std::string& xml_tag_name, const BT::NodeConfiguration& conf);

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing basic ports along with node-specific
     * ports
     */
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("global_frame", "Global reference frame"),
            BT::InputPort<std::string>("robot_base_frame", "robot base frame"),
            BT::OutputPort<commsgs::geometry_msgs::PoseStamped>("current_pose", "Current pose output"),
        };
    }

private:
    /**
     * @brief The other (optional) override required by a BT action.
     */
    void halt() override {}

    /**
     * @brief The main override required by a BT action
     * @return BT::NodeStatus Status of tick execution
     */
    BT::NodeStatus tick() override;

    std::shared_ptr<::autolink::Node> node_;
    std::string global_frame_, robot_base_frame_;
    std::shared_ptr<autonomy::transform::Buffer> tf_;
    float transform_tolerance_;
};

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
