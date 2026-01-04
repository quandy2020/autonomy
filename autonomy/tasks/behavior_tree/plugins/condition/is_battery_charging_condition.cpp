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

#include "autonomy/tasks/behavior_tree/plugins/condition/is_battery_charging_condition.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace condition {

IsBatteryChargingCondition::IsBatteryChargingCondition(
    const std::string& condition_name, const BT::NodeConfiguration& conf)
    : BT::ConditionNode(condition_name, conf),
      battery_topic_("/battery_status"),
      is_battery_charging_(false) {
    initialize();
}

void IsBatteryChargingCondition::initialize() {
    createROSInterfaces();
}

void IsBatteryChargingCondition::createROSInterfaces() {
    std::string battery_topic_new;
    getInput("battery_topic", battery_topic_new);

    // Only create a new subscriber if the topic has changed or subscriber is
    // empty
    if (battery_topic_new != battery_topic_ || !battery_reader_) {
        battery_topic_ = battery_topic_new;
        auto node =
            config().blackboard->get<std::shared_ptr<::autolink::Node>>("node");
        battery_reader_ =
            node->CreateReader<commsgs::sensor_msgs::BatteryState>(
                battery_topic_,
                std::bind(&IsBatteryChargingCondition::batteryCallback, this,
                          std::placeholders::_1));
    }
}

BT::NodeStatus IsBatteryChargingCondition::tick() {
    if (!BT::isStatusActive(status())) {
        initialize();
    }
    if (is_battery_charging_) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

void IsBatteryChargingCondition::batteryCallback(
    const std::shared_ptr<commsgs::sensor_msgs::BatteryState>& msg) {
    is_battery_charging_ = (msg->power_supply_status() ==
                            commsgs::proto::sensor_msgs::BatteryState::
                                POWER_SUPPLY_STATUS_CHARGING);
}

}  // namespace condition
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<autonomy::tasks::behavior_tree::plugins::
                                 condition::IsBatteryChargingCondition>(
        "IsBatteryCharging");
}
