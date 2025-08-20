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

#include "autonomy/tasks/navigation/navigator_task.hpp"
#include "autonomy/common/logging.hpp"

#include "absl/strings/str_cat.h"
namespace autonomy {
namespace tasks {
namespace navigation {


NavigatorTask::NavigatorTask()
{
    // Create the class that registers our custom nodes and executes the BT
    bt_ = std::make_shared<behavior_tree::BehaviorTreeEngine>(plugin_lib_names_, nullptr);

    // Create the blackboard that will be shared by all of the nodes in the tree
    blackboard_ = BT::Blackboard::create();
}

NavigatorTask::~NavigatorTask()
{
    
}

void NavigatorTask::ExecuteCallback()
{
    auto is_canceling = [&]() {
        return false;
    };

    auto on_loop = [&]() {
        OnLoop();
    };

    // Execute the BT that was previously created in the configure step
    behavior_tree::BtStatus rc = bt_->Run(&tree_, on_loop, is_canceling, bt_loop_duration_);

    // Make sure that the Bt is not in a running state from a previous execution
    // note: if all the ControlNodes are implemented correctly, this is not needed.
    bt_->HaltAllActions(tree_);

    // result
    switch (rc) {
        case behavior_tree::BtStatus::SUCCEEDED:
            LOG(INFO) << "Goal succeeded";
            break;

        case behavior_tree::BtStatus::FAILED:
            LOG(INFO) << "Goal failed";
            break;

        case behavior_tree::BtStatus::CANCELED:
            LOG(INFO) << "Goal canceled";
            break;
    }
}

void NavigatorTask::OnLoop()
{

}

std::string NavigatorTask::GetDefaultBTFilepath()
{
    std::string default_bt_xml_filename;
    return default_bt_xml_filename;
}

bool NavigatorTask::HandleGoalReceived(const commsgs::geometry_msgs::PoseStamped& goal)
{
    // auto bt_xml_filename = goal->behavior_tree;
    std::string bt_xml_filename;
    if (LoadBehaviorTree(bt_xml_filename)) {
        LOG(ERROR) << absl::StrCat("BT file not found: ", bt_xml_filename, " . Navigation canceled.");
        return false;
    }

  return InitializeGoalPose(goal);
}

bool NavigatorTask::LoadBehaviorTree(const std::string& bt_xml_filename)
{
    return true;
}

bool NavigatorTask::InitializeGoalPose(const commsgs::geometry_msgs::PoseStamped& goal)
{
    auto blackboard = GetBlackboard();
    blackboard->set("number_recoveries", 0);  // NOLINT

    commsgs::geometry_msgs::PoseStamped goal_pose;

    // Update the goal pose on the blackboard
    blackboard->set(goal_blackboard_id_, goal_pose);
    return true;
}

}  // namespace navigation
}  // namespace tasks
}  // namespace autonomy