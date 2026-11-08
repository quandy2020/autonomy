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

#include "navigation/planning/tasks/recovery_task.hpp"

namespace navigation
{
namespace planning
{

void RecoveryTask::Start(RecoveryCause cause)
{
    executor_.Start(cause);
    last_action_ = RecoveryAction::kNone;
}

common::Status RecoveryTask::Execute(NavigationFrame& frame)
{
    last_action_ = executor_.Tick();
    if (last_action_ == RecoveryAction::kGiveUp)
    {
        frame.degraded = true;
        return common::Status::Unavailable("recovery exhausted");
    }
    frame.degraded = true;
    return common::Status{};
}

void RecoveryTask::Reset()
{
    executor_.Reset();
    last_action_ = RecoveryAction::kNone;
}

std::string_view RecoveryTask::Name() const
{
    return "recovery";
}

RecoveryAction RecoveryTask::LastAction() const
{
    return last_action_;
}

}  // namespace planning
}  // namespace navigation
