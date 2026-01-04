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

#include <set>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"

#include "autolink/autolink.hpp"
#include "autonomy/commsgs/std_msgs.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_service_node.hpp"
#include "autonomy/tasks/navigator/proto/srv.pb.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

/**
 * @brief A autonomy::tasks::behavior_tree::plugins::action::BtServiceNode class
 * that wraps proto::ClearEntireCostmap
 * @note This is an Asynchronous (long-running) node which may return a RUNNING
 * state while executing. It will re-initialize when halted.
 */
class ClearEntireCostmapService
    : public BtServiceNode<proto::ClearEntireCostmap>
{
public:
    /**
     * @brief A constructor for
     * autonomy::tasks::behavior_tree::plugins::action::ClearEntireCostmapService
     * @param service_node_name Service name this node creates a client for
     * @param conf BT node configuration
     */
    ClearEntireCostmapService(const std::string& service_node_name,
                              const BT::NodeConfiguration& conf);

    /**
     * @brief The main override required by a BT service
     * @return BT::NodeStatus Status of tick execution
     */
    void on_tick() override;
};

/**
 * @brief A autonomy::tasks::behavior_tree::plugins::action::BtServiceNode class
 * that wraps proto::ClearCostmapExceptRegion
 * @note This is an Asynchronous (long-running) node which may return a RUNNING
 * state while executing. It will re-initialize when halted.
 */
class ClearCostmapExceptRegionService
    : public BtServiceNode<proto::ClearCostmapExceptRegion>
{
public:
    /**
     * @brief A constructor for
     * autonomy::tasks::behavior_tree::plugins::action::ClearCostmapExceptRegionService
     * @param service_node_name Service name this node creates a client for
     * @param conf BT node configuration
     */
    ClearCostmapExceptRegionService(const std::string& service_node_name,
                                    const BT::NodeConfiguration& conf);

    /**
     * @brief The main override required by a BT service
     * @return BT::NodeStatus Status of tick execution
     */
    void on_tick() override;

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing basic ports along with node-specific
     * ports
     */
    static BT::PortsList providedPorts() {
        return providedBasicPorts({BT::InputPort<double>(
            "reset_distance", 1,
            "Distance from the robot above which obstacles are cleared")});
    }
};

/**
 * @brief A autonomy::tasks::behavior_tree::plugins::action::BtServiceNode class
 * that wraps proto::ClearCostmapAroundRobot
 * @note This is an Asynchronous (long-running) node which may return a RUNNING
 * state while executing. It will re-initialize when halted.
 */
class ClearCostmapAroundRobotService
    : public BtServiceNode<proto::ClearCostmapAroundRobot>
{
public:
    /**
     * @brief A constructor for
     * autonomy::tasks::behavior_tree::plugins::action::ClearCostmapAroundRobotService
     * @param service_node_name Service name this node creates a client for
     * @param conf BT node configuration
     */
    ClearCostmapAroundRobotService(const std::string& service_node_name,
                                   const BT::NodeConfiguration& conf);

    /**
     * @brief The main override required by a BT service
     * @return BT::NodeStatus Status of tick execution
     */
    void on_tick() override;

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing basic ports along with node-specific
     * ports
     */
    static BT::PortsList providedPorts() {
        return providedBasicPorts({BT::InputPort<double>(
            "reset_distance", 1,
            "Distance from the robot under which obstacles are cleared")});
    }
};

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy