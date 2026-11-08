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

#include "navigation/behavior/engine/bt_engine_implementation.hpp"

namespace navigation
{
namespace behavior
{

bool BtEngineImplementation::Configure(const config::BehaviorOptions& options,
                                       planning::NavigationSession* session)
{
    if (session == nullptr)
    {
        return false;
    }
    options_ = options;
    session_ = session;
    state_ = State::kIdle;
    recovery_attempts_ = 0;
    return true;
}

common::Status BtEngineImplementation::Tick(std::int64_t now_ns)
{
    if (!options_.enabled() || session_ == nullptr)
    {
        return common::Status{};
    }
    if (now_ns < last_tick_ns_)
    {
        return common::Status::InvalidArgument("non-monotonic behavior clock");
    }
    last_tick_ns_ = now_ns;

    if (state_ == State::kDone)
    {
        if (session_->IsGoalReached())
        {
            return common::Status{};
        }
        state_ = State::kNavigate;
    }
    if (state_ == State::kIdle)
    {
        if (!session_->HasActiveGoal())
        {
            return common::Status{};
        }
        state_ = State::kNavigate;
    }
    if (state_ == State::kNavigate)
    {
        if (session_->IsGoalReached())
        {
            state_ = State::kDone;
            return common::Status{};
        }
        if (session_->State() == lifecycle::MODULE_STATE_FAILED)
        {
            state_ = State::kRecovery;
        }
        else
        {
            return common::Status{};
        }
    }
    if (state_ == State::kRecovery)
    {
        if (recovery_attempts_ >= options_.max_recovery_attempts())
        {
            return common::Status::Unavailable("recovery attempts exhausted");
        }
        ++recovery_attempts_;
        if (!session_->RequestRecovery("behavior_failure"))
        {
            return common::Status::Unavailable("recovery request rejected");
        }
        state_ = State::kNavigate;
    }
    return common::Status{};
}

void BtEngineImplementation::Reset()
{
    state_ = State::kIdle;
    recovery_attempts_ = 0;
    last_tick_ns_ = 0;
}

std::string_view BtEngineImplementation::Name() const
{
    return "simple_bt";
}

}  // namespace behavior
}  // namespace navigation
