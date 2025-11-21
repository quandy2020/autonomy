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

#include "autonomy/tasks/task_bridge.hpp"
#include "autonomy/tasks/navigation/navigator_task.hpp"
#include "autonomy/tasks/constants.hpp"
#include "autonomy/common/json_util.hpp"

namespace autonomy {
namespace tasks {

TaskBridge::TaskBridge()
    : thread_pool_(std::thread::hardware_concurrency())
{
    std::vector<std::string> kTaskName = {
        kTaskNavigation,
    };

    for (auto const& name : kTaskName) {
        if (name == kTaskNavigation) {
            tasks_[name] = std::make_shared<navigation::NavigatorTask>(name);
        }
    }
}

TaskBridge::~TaskBridge()
{
    
}

void TaskBridge::Start()
{
    for (auto const& task : tasks_) {
        task.second->Start();
    }
    RunTasks();
}

void TaskBridge::Shutdown()
{
    for (auto const& task : tasks_) {
        task.second->Shutdown();
    }
}

void TaskBridge::RunTasks()
{
    while (!cmd_queue_.IsFinished()) {
        cmd_queue_.Flush();
        LOG(INFO) << "Task Bridge run tasks.";
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// bool TaskBridge::HandleCommandMessageCallback(const proto::ExtendCommand& msgs)
// {
//     LOG(INFO) << "Task Bridge handle command msgs: " 
//               << autonomy::common::JsonUtil::ProtoToJson(msgs);

//     auto task = std::make_unique<autonomy::common::Task>();
//     switch (msgs.action()) {
//     case proto::ExtendCommand::START:
//         task->SetWorkItem([this, msgs]() {
//             LOG(INFO) << "Exec start action command.";
//             tasks_.at(name(msgs.type()))->Start(msgs);
//         });
//         break;

//     case proto::ExtendCommand::STOP:
//         task->SetWorkItem([this, msgs]() {
//             LOG(INFO) << "Exec stop action command.";
//             tasks_.at(name(msgs.type()))->Stop();
//         });
//         break;

//     case proto::ExtendCommand::RESUME:
//         task->SetWorkItem([this, msgs]() {
//             LOG(INFO) << "Exec resume action command.";
//             tasks_.at(name(msgs.type()))->Resume();
//         });
//         break;

//     case proto::ExtendCommand::CANCEL:
//         task->SetWorkItem([this, msgs]() {
//             LOG(INFO) << "Exec cancel action command.";
//             tasks_.at(name(msgs.type()))->Cancel();
//         });
//         break;

//     default:
//         LOG(INFO) << "Task Bridge unknown command msgs: " 
//               << autonomy::common::JsonUtil::ProtoToJson(msgs);
//         break;
//     }

//     thread_pool_.Schedule(std::move(task));
//     return true;
// }

}   // namespace tasks
}   // namespace autonomy