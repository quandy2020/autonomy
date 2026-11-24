/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file initial_pose_received_condition.cpp
 * @brief Initial pose received condition.
 */

#include "autonomy/task/behavior_tree/plugins/bt_node_base.hpp"

namespace autonomy::task::plugins::navigation {

class InitialPoseReceivedCondition : public BtCondition
{
public:
    InitialPoseReceivedCondition(const std::string& name,
                                 const BT::NodeConfig& config)
        : BtCondition(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<bool>("initial_pose_received", true,
                                    "blackboard / port pose-init flag")};
    }

protected:
    BT::NodeStatus OnEvaluate() override
    {
        bool received = true;
        if (!getInput("initial_pose_received", received)) {
            if (config().blackboard) {
                (void)config().blackboard->get("initial_pose_received",
                                               received);
            }
        }
        return received ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

}  // namespace autonomy::task::plugins::navigation

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<
        autonomy::task::plugins::navigation::InitialPoseReceivedCondition>(
        "InitialPoseReceived");
}
