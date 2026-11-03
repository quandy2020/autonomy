/*
 * Copyright 2026 The Openbot Authors
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

#include "atlas/mapping/local_mapping_idle_monitor.hpp"

namespace autonomy::localization::atlas::internal {

void LocalMappingIdleMonitor::SetAccepting(bool accepting) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        accepting_ = accepting;
    }
    state_changed_.notify_all();
}

void LocalMappingIdleMonitor::SetQueueEmpty(bool queue_empty) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_empty_ = queue_empty;
    }
    state_changed_.notify_all();
}

void LocalMappingIdleMonitor::Interrupt() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        interrupted_ = true;
        ++interruption_generation_;
    }
    state_changed_.notify_all();
}

void LocalMappingIdleMonitor::Resume() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        interrupted_ = false;
    }
    state_changed_.notify_all();
}

LocalMappingIdleWaitResult LocalMappingIdleMonitor::WaitUntilIdle(
    std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    const std::uint64_t initial_generation = interruption_generation_;
    const auto wait_complete = [this, initial_generation]() {
        return IsIdleLocked() || interrupted_ ||
               interruption_generation_ != initial_generation;
    };

    if (!state_changed_.wait_for(lock, timeout, wait_complete))
        return LocalMappingIdleWaitResult::kTimedOut;
    if (interrupted_ || interruption_generation_ != initial_generation)
        return LocalMappingIdleWaitResult::kInterrupted;
    return LocalMappingIdleWaitResult::kIdle;
}

bool LocalMappingIdleMonitor::IsIdle() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return !interrupted_ && IsIdleLocked();
}

bool LocalMappingIdleMonitor::IsIdleLocked() const {
    return accepting_ && queue_empty_;
}

}  // namespace autonomy::localization::atlas::internal
