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

#include "navigation/planning/tasks/recovery_executor.hpp"

namespace navigation
{
namespace planning
{

void RecoveryExecutor::Start(RecoveryCause cause)
{
    cause_ = cause;
    step_ = cause == RecoveryCause::kNone ? 3U : 0U;
}

RecoveryAction RecoveryExecutor::Tick()
{
    switch (step_)
    {
        case 0:
            ++step_;
            return RecoveryAction::kClearGoal;
        case 1:
            ++step_;
            return cause_ == RecoveryCause::kLocalizationLost ? RecoveryAction::kRelocalize
                                                              : RecoveryAction::kRetry;
        case 2:
            ++step_;
            return RecoveryAction::kGiveUp;
        default:
            return RecoveryAction::kNone;
    }
}

void RecoveryExecutor::Reset()
{
    cause_ = RecoveryCause::kNone;
    step_ = 3U;
}

RecoveryCause RecoveryExecutor::Cause() const
{
    return cause_;
}

}  // namespace planning
}  // namespace navigation
