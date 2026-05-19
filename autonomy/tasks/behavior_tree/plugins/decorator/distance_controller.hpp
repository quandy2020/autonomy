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

#include <memory>
#include <string>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_utils.hpp"
#include "autonomy/transform/buffer.hpp"
#include "behaviortree_cpp/decorator_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace decorator {

/**
 * @brief A BT::DecoratorNode that ticks its child every time the robot
 * travels a specified distance
 * @note This is an Asynchronous (long-running) node which may return a RUNNING
 * state while executing. It will re-initialize when halted.
 */
class DistanceController : public BT::DecoratorNode
{
public:
    /**
     * @brief A constructor for nav2_behavior_tree::DistanceController
     * @param name Name for the XML tag for this node
     * @param conf BT node configuration
     */
    DistanceController(const std::string& name,
                       const BT::NodeConfiguration& conf);

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing node-specific ports
     */
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("distance", 1.0, "Distance"),
            BT::InputPort<std::string>("global_frame", "Global frame"),
            BT::InputPort<std::string>("robot_base_frame", "Robot base frame")};
    }

private:
    /**
     * @brief The main override required by a BT action
     * @return BT::NodeStatus Status of tick execution
     */
    BT::NodeStatus tick() override;

    std::shared_ptr<autonomy::transform::Buffer> tf_;
    double transform_tolerance_;

    commsgs::geometry_msgs::PoseStamped start_pose_;
    double distance_;
    std::string global_frame_, robot_base_frame_;

    bool first_time_;
};

}  // namespace decorator
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy