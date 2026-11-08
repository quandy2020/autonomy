/*
 * Copyright 2026 The Navigation Authors (navigation)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "navigation/behavior/tasks/relocalization_task.hpp"

namespace navigation
{
namespace behavior
{

RelocalizationTask::RelocalizationTask(std::int64_t timeout_ns)
    : timeout_ns_(timeout_ns > 0 ? timeout_ns : 10000000000LL)
{
}

void RelocalizationTask::Start(TaskContext& context)
{
    start_ns_ = context.now_ns;
    if (context.session == nullptr ||
        !context.session->RequestRecovery("localization_lost"))
    {
        status_ = TaskStatus::kFailed;
        return;
    }
    status_ = TaskStatus::kRunning;
}

TaskStatus RelocalizationTask::Tick(TaskContext& context)
{
    if (status_ != TaskStatus::kRunning)
    {
        return status_;
    }
    if (context.session != nullptr &&
        context.session->State() == lifecycle::MODULE_STATE_READY)
    {
        status_ = TaskStatus::kSucceeded;
        return status_;
    }
    if (context.now_ns - start_ns_ > timeout_ns_)
    {
        status_ = TaskStatus::kFailed;
    }
    return status_;
}

void RelocalizationTask::Reset()
{
    status_ = TaskStatus::kIdle;
    start_ns_ = 0;
}

std::string_view RelocalizationTask::Name() const
{
    return "relocalization";
}

}  // namespace behavior
}  // namespace navigation
