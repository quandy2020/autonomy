/*
 * Copyright 2026 The Openbot Authors
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

/**
 * @file task_muxer.hpp
 * @brief Single exclusive command slot + estop latch.
 *
 * All domain stubs that claim exclusive robot motion / long-running goals
 * share one TaskMuxer instance owned by Context. Session Gate TryAcquire
 * before scheduling Execute; Release / Clear free the slot. Estop latches
 * reject every subsequent TryAcquire until cleared.
 */

#pragma once

#include <mutex>
#include <string>

#include "autonomy/bridge/grpc/task_types.hpp"
#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {

/**
 * @brief Single exclusive command slot + estop latch for all domain stubs.
 *
 * Serializes START-like commands across navigator / teleop / follow / dock /
 * explore / map / voice so at most one exclusive TaskType is active. Estop is
 * an orthogonal latch: when set, TryAcquire always fails even if the slot is
 * free.
 *
 * @par Invariants
 * - At most one active TaskType; estop ⇒ TryAcquire always fails.
 * - Release is type-filtered (mismatched type is a no-op); Clear / CancelAll
 * paths clear unconditionally.
 * - GetSnapshot returns a value copy; never a live reference into mutex-guarded
 * state.
 *
 * @par Threading
 * every public method locks mutex_; safe from handler + worker
 * threads. Do not call while already holding another lock that workers may
 * acquire in the opposite order (avoid ABBA with stub mutexes).
 *
 * @par Ownership
 * typically Context::shared_ptr; stubs / Session keep SharedPtr
 * copies so the muxer outlives scheduled Execute lambdas.
 */
class TaskMuxer
{
public:
    /**
     * @brief Shared / weak / unique pointer aliases.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(TaskMuxer)

    /**
     * @brief Try to acquire the exclusive task slot.
     *
     * Succeeds only when !estop_ and active_type_ == TASK_TYPE_NONE. On
     * success records @p type, @p cmd_id, and @p client_id for GetSnapshot.
     *
     * @param[in] type      Task type claiming the slot.
     * @param[in] cmd_id    Command / goal id recorded in the snapshot.
     * @param[in] client_id Client id recorded in the snapshot.
     * @return              false if another task already holds the slot (or estop is on).
     *
     * @note Callers that already passed idempotency TryBegin must Forget the
     * cmd_id when this returns false (see BackgroundCommandSession).
     */
    bool TryAcquire(TaskType type, const std::string& cmd_id,
                    const std::string& client_id);

    /**
     * @brief Release the slot when @p type matches the active task.
     *
     * Type-filtered to avoid a late Release from task A clearing task B after
     * a legitimate handoff / Clear. Mismatched @p type is ignored.
     *
     * @param[in] type Task type that should release.
     */
    void Release(TaskType type);

    /**
     * @brief Clear the active slot unconditionally.
     *
     * Resets type to TASK_TYPE_NONE and clears cmd_id / client_id. Does not
     * change the estop latch — use SetEstop(false) separately.
     */
    void Clear();

    /**
     * @brief Whether any task currently holds the slot.
     *
     * @return true if active_type_ != TASK_TYPE_NONE.
     */
    bool HasActive() const;

    /**
     * @brief Return a snapshot of the active task (if any).
     *
     * @return ActiveTaskInfo copy under the muxer lock (status may be IDLE
     *        when no richer status is tracked here).
     */
    ActiveTaskInfo GetSnapshot() const;

    /**
     * @brief Latch or clear the emergency-stop flag.
     *
     * Does not Clear the active slot by itself; Context::EmergencyStop(true)
     * typically CancelAllTasks after SetEstop(true).
     *
     * @param[in] estop New estop state.
     */
    void SetEstop(bool estop);

    /**
     * @brief Whether emergency-stop is latched.
     *
     * @return true if TryAcquire will fail regardless of slot occupancy.
     */
    bool IsEstop() const;

private:
    /**
     * @brief Guards active slot fields and the estop latch.
     */
    mutable std::mutex mutex_;

    /**
     * @brief Exclusive TaskType holding the slot (TASK_TYPE_NONE when free).
     */
    TaskType active_type_{TASK_TYPE_NONE};

    /**
     * @brief Command / goal id recorded by the last successful TryAcquire.
     */
    std::string cmd_id_;

    /**
     * @brief Client id recorded by the last successful TryAcquire.
     */
    std::string client_id_;

    /**
     * @brief Emergency-stop latch; when true TryAcquire always fails.
     */
    bool estop_{false};
};

}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
