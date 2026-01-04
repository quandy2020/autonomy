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

#include "autonomy/tasks/navigator/navigation/navigate_to_pose.hpp"

namespace autonomy {
namespace tasks {
namespace navigator {
namespace navigation {

bool NavigateToPoseNavigator::Configure(
    const std::shared_ptr<autolink::Node> node) {
    // TODO: Implement configuration
    return true;
}

bool NavigateToPoseNavigator::Cleanup() {
    // TODO: Implement cleanup
    return true;
}

std::string NavigateToPoseNavigator::GetDefaultBTFilepath(
    std::weak_ptr<autolink::Node> node) {
    // TODO: Return default BT file path
    return std::string("navigate_to_pose_w_replanning_and_recovery.xml");
}

bool NavigateToPoseNavigator::GoalReceived(
    const std::shared_ptr<typename ActionT::Goal> goal) {
    // TODO: Implement goal validation and initialization
    InitializeGoalPose(goal);
    return true;
}

void NavigateToPoseNavigator::OnLoop() {
    // TODO: Implement loop callback
}

void NavigateToPoseNavigator::OnPreempt(const typename ActionT::Goal& goal) {
    // TODO: Implement preempt callback
}

void NavigateToPoseNavigator::GoalCompleted(
    const std::shared_ptr<typename ActionT::Result> result,
    const autonomy::tasks::behavior_tree::BtStatus final_bt_status) {
    // TODO: Implement goal completion callback
}

void NavigateToPoseNavigator::InitializeGoalPose(
    const std::shared_ptr<typename ActionT::Goal> goal) {
    // TODO: Initialize goal pose on blackboard
}

void NavigateToPoseNavigator::OnGoalPoseReceived(
    const commsgs::geometry_msgs::PoseStamped::SharedPtr pose) {
    // TODO: Implement goal pose received callback
}

}  // namespace navigation
}  // namespace navigator
}  // namespace tasks
}  // namespace autonomy