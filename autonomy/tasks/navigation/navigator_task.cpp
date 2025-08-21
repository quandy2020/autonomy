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
#include "autonomy/tasks/proto/extend_command.pb.h"
#include "autonomy/commsgs/geometry_msgs.hpp"
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

NavigatorTask::NavigatorTask(const std::string& name)
    : NavigatorTask()
{
    name_ = name;
    state_ = TaskState::IDLE;
}

bool NavigatorTask::Resume()
{
    if (state_ != TaskState::PAUSED) {
        LOG(INFO) << "[" << name_ << "] Cannot resume - not paused.";
        return false;
    }
    
    LOG(INFO) << "[" << name_ << "] Resuming task.";
    state_ = TaskState::RUNNING;
    return true;
}

bool NavigatorTask::Cancel()
{
    if (state_ == TaskState::COMPLETED || state_ == TaskState::FAILED) {
        LOG(INFO) << "[" << name_ << "] Cannot cancel - already finished.";
        return false;
    }
        
    LOG(INFO) << "[" << name_ << "] Canceling task.";
    state_ = TaskState::CANCELED;

    return true;
}

bool NavigatorTask::Stop()
{
    if (state_ == TaskState::COMPLETED || state_ == TaskState::FAILED) {
        LOG(INFO) << "[" << name_ << "] Cannot stop - already finished.";
        return false;
    }
    
    LOG(INFO) << "[" << name_ << "] Stopping task.";
    state_ = TaskState::STOPPED;
    return true;
}

common::TaskInterface::TaskState NavigatorTask::GetState() const
{
    return state_;
}

std::string NavigatorTask::GetName() const 
{
    return name_;
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

bool NavigatorTask::StartImpl(std::vector<std::any>&& args)
{
    try {
        CheckArgumentCount(args, 1, name_);
        auto command = GetArgument<proto::ExtendCommand>(args[0], 0, "ExtendCommand");
        return HandleGoalReceived(command.plan().goal());
    } catch (const std::exception& e) {
        LOG(ERROR) << "[" << name_ << "] Error processing command: " << e.what();
        state_ = TaskState::FAILED;
        return false;
    }
    return true;
}

void NavigatorTask::OnLoop()
{

}

std::string NavigatorTask::GetDefaultBTFilepath()
{
    std::string default_bt_xml_filename;
    return default_bt_xml_filename;
}

bool NavigatorTask::HandleGoalReceived(const proto::NavigateToPose::Goal& goal)
{
    auto bt_xml_filename = goal.behavior_tree();
    if (LoadBehaviorTree(bt_xml_filename)) {
        LOG(ERROR) << absl::StrCat("BT file not found: ", bt_xml_filename, " . Navigation canceled.");
        return false;
    }

  return InitializeGoalPose(commsgs::geometry_msgs::FromProto(goal.pose()));
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

    execution_future_ = std::async(std::launch::async, [this]() {
        ExecuteCallback();
    });

    return true;
}

}  // namespace navigation
}  // namespace tasks
}  // namespace autonomy