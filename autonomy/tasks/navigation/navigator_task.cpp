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



#include "absl/strings/str_cat.h"

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/tasks/navigation/navigator_task.hpp"
#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace tasks {
namespace navigation {


NavigatorTask::NavigatorTask()
{
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

void NavigatorTask::Shutdown()
{
    LOG(INFO) << "[" << name_ << "] Shutdown task.";
    state_ = TaskState::SHUTDOWN;
}

common::TaskInterface::TaskState NavigatorTask::GetState() const
{
    return state_;
}

std::string NavigatorTask::GetName() const 
{
    return name_;
}

bool NavigatorTask::StartImpl(std::vector<std::any>&& args)
{
    return true;
}

}  // namespace navigation
}  // namespace tasks
}  // namespace autonomy