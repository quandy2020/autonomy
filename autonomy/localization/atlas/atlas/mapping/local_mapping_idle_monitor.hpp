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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_ATLAS_MAPPING_LOCAL_MAPPING_IDLE_MONITOR_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_ATLAS_MAPPING_LOCAL_MAPPING_IDLE_MONITOR_HPP_

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>

namespace autonomy::localization::atlas::internal {

enum class LocalMappingIdleWaitResult {
    kIdle,
    kInterrupted,
    kTimedOut,
};

/**
 * Waitable mirror of the local-mapper admission and queue state.
 *
 * The mirror has its own mutex so waiters never acquire the admission and
 * keyframe-queue mutexes in the opposite order from the worker thread.
 */
class LocalMappingIdleMonitor final
{
public:
    void SetAccepting(bool accepting);
    void SetQueueEmpty(bool queue_empty);

    /** Interrupts current and newly arriving waiters until Resume(). */
    void Interrupt();
    void Resume();

    LocalMappingIdleWaitResult WaitUntilIdle(std::chrono::milliseconds timeout);
    bool IsIdle() const;

private:
    bool IsIdleLocked() const;

    mutable std::mutex mutex_;
    std::condition_variable state_changed_;
    bool accepting_ = true;
    bool queue_empty_ = true;
    bool interrupted_ = false;
    std::uint64_t interruption_generation_ = 0;
};

}  // namespace autonomy::localization::atlas::internal

#endif  // AUTONOMY_LOCALIZATION_ATLAS_ATLAS_MAPPING_LOCAL_MAPPING_IDLE_MONITOR_HPP_
