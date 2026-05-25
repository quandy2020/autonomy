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

#include "autonomy/tasks/behavior_tree/plugins/action/clear_costmap_service.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/tasks/utils/costmap_clear_utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

std::string BtCostmapClearNode::serviceNameFromPorts() const {
    std::string service_name;
    if (getInput("service_name", service_name)) {
        return service_name;
    }
    return "";
}

ClearEntireCostmapService::ClearEntireCostmapService(
    const std::string& /*service_node_name*/, const BT::NodeConfiguration& conf)
    : BtCostmapClearNode("ClearEntireCostmap", conf) {}

BT::NodeStatus ClearEntireCostmapService::onStart() {
    incrementRecoveryCount();
    auto ctx = taskContext();
    auto costmap = utils::ResolveCostmap(ctx, serviceNameFromPorts());
    if (!costmap) {
        AERROR << "ClearEntireCostmap: no costmap available.";
        return BT::NodeStatus::FAILURE;
    }
    utils::ClearEntireCostmap(costmap);
    return BT::NodeStatus::SUCCESS;
}

ClearCostmapExceptRegionService::ClearCostmapExceptRegionService(
    const std::string& /*service_node_name*/, const BT::NodeConfiguration& conf)
    : BtCostmapClearNode("ClearCostmapExceptRegion", conf) {}

BT::NodeStatus ClearCostmapExceptRegionService::onStart() {
    incrementRecoveryCount();
    double reset_distance = 1.0;
    getInput("reset_distance", reset_distance);

    auto ctx = taskContext();
    auto costmap = utils::ResolveCostmap(ctx, serviceNameFromPorts());
    if (!costmap) {
        AERROR << "ClearCostmapExceptRegion: no costmap available.";
        return BT::NodeStatus::FAILURE;
    }
    utils::ClearCostmapExceptRegion(costmap, reset_distance);
    return BT::NodeStatus::SUCCESS;
}

ClearCostmapAroundRobotService::ClearCostmapAroundRobotService(
    const std::string& /*service_node_name*/, const BT::NodeConfiguration& conf)
    : BtCostmapClearNode("ClearCostmapAroundRobot", conf) {}

BT::NodeStatus ClearCostmapAroundRobotService::onStart() {
    incrementRecoveryCount();
    double reset_distance = 1.0;
    getInput("reset_distance", reset_distance);

    auto ctx = taskContext();
    auto costmap = utils::ResolveCostmap(ctx, serviceNameFromPorts());
    if (!costmap) {
        AERROR << "ClearCostmapAroundRobot: no costmap available.";
        return BT::NodeStatus::FAILURE;
    }
    utils::ClearCostmapAroundRobot(costmap, reset_distance);
    return BT::NodeStatus::SUCCESS;
}

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<autonomy::tasks::behavior_tree::plugins::action::
                                 ClearEntireCostmapService>(
        "ClearEntireCostmap");
    factory.registerNodeType<autonomy::tasks::behavior_tree::plugins::action::
                                 ClearCostmapExceptRegionService>(
        "ClearCostmapExceptRegion");
    factory.registerNodeType<autonomy::tasks::behavior_tree::plugins::action::
                                 ClearCostmapAroundRobotService>(
        "ClearCostmapAroundRobot");
}
