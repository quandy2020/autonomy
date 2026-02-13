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

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

ClearEntireCostmapService::ClearEntireCostmapService(const std::string& service_node_name,
                                                     const BT::NodeConfiguration& conf)
    : BtServiceNode<proto::ClearEntireCostmap>(service_node_name, conf) {}

void ClearEntireCostmapService::on_tick() {
    increment_recovery_count();
}

ClearCostmapExceptRegionService::ClearCostmapExceptRegionService(const std::string& service_node_name,
                                                                 const BT::NodeConfiguration& conf)
    : BtServiceNode<proto::ClearCostmapExceptRegion>(service_node_name, conf) {}

void ClearCostmapExceptRegionService::on_tick() {
    double reset_distance;
    getInput("reset_distance", reset_distance);
    request_->set_reset_distance(reset_distance);
    increment_recovery_count();
}

ClearCostmapAroundRobotService::ClearCostmapAroundRobotService(const std::string& service_node_name,
                                                               const BT::NodeConfiguration& conf)
    : BtServiceNode<proto::ClearCostmapAroundRobot>(service_node_name, conf) {}

void ClearCostmapAroundRobotService::on_tick() {
    double reset_distance;
    getInput("reset_distance", reset_distance);
    request_->set_reset_distance(reset_distance);
    increment_recovery_count();
}

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<autonomy::tasks::behavior_tree::plugins::action::ClearEntireCostmapService>(
        "ClearEntireCostmap");
    factory.registerNodeType<autonomy::tasks::behavior_tree::plugins::action::ClearCostmapExceptRegionService>(
        "ClearCostmapExceptRegion");
    factory.registerNodeType<autonomy::tasks::behavior_tree::plugins::action::ClearCostmapAroundRobotService>(
        "ClearCostmapAroundRobot");
}
