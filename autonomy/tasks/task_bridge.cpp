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
{
    std::vector<std::string> kTaskName = {
        kPlanTaskName,
        kMappingTaskName,
        kRelocalizationTaskName
    };

    for (auto const& name : kTaskName) {
        if (name == kPlanTaskName) {
            tasks_[name] = std::make_shared<navigation::NavigatorTask>(name);
        }
    }
}

TaskBridge::~TaskBridge()
{
    
}

void TaskBridge::Shutdown()
{

}

bool TaskBridge::HandleCommandMessageCallback(const proto::ExtendCommand& msgs)
{
    LOG(INFO) << "Task Bridge handle command msgs: " 
              << autonomy::common::JsonUtil::ProtoToJson(msgs);
    switch (msgs.action()) {
    case proto::ExtendCommand::START:
        LOG(INFO) << "Exec start action command.";
        tasks_.at(name(msgs.type()))->Start(msgs);
        break;

    case proto::ExtendCommand::STOP:
        LOG(INFO) << "Exec stop action command.";
        tasks_.at(name(msgs.type()))->Stop();
        break;

    case proto::ExtendCommand::RESUME:
        LOG(INFO) << "Exec resume action command.";
        tasks_.at(name(msgs.type()))->Resume();
        break;

    case proto::ExtendCommand::CANCEL:
        LOG(INFO) << "Exec cancel action command.";
        tasks_.at(name(msgs.type()))->Cancel();
        break;

    default:
        break;
    }
   return true;
}
    
std::string TaskBridge::name(const proto::ExtendCommand::Type& msgs)
{
    switch (msgs) {
    case proto::ExtendCommand::PLAN:
        return kPlanTaskName;
        break;

    case proto::ExtendCommand::MAPPING:
        return kPlanTaskName;
        break;

    case proto::ExtendCommand::RELOCALIZATION:
        return kRelocalizationTaskName;
        break;
        
    default:
        return "";
        break;
    }
    return "";
}

}   // namespace tasks
}   // namespace autonomy