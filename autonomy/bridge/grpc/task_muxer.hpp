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

#pragma once

#include <mutex>
#include <string>

#include "autonomy/bridge/proto/external_command_service.pb.h"
#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {

/**
 * @brief Tracks the single active AutonomyService command task.
 *
 * Emergency-stop / cancel-all clear the slot; domain stubs call
 * `TryAcquire` / `Release`.
 */
class TaskMuxer
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(TaskMuxer)

    /**
     * @brief Try to acquire the exclusive task slot.
     * @param[in] type Task type claiming the slot.
     * @param[in] cmd_id Command id recorded in the snapshot.
     * @param[in] client_id Client id recorded in the snapshot.
     * @return false if another task already holds the slot (or estop is on).
     */
    bool TryAcquire(proto::TaskType type, const std::string& cmd_id,
                    const std::string& client_id);

    /**
     * @brief Release the slot when @p type matches the active task.
     * @param[in] type Task type that should release.
     */
    void Release(proto::TaskType type);

    /** @brief Clear the active slot unconditionally. */
    void Clear();

    /** @brief Check whether any task currently holds the slot. */
    bool CheckHasActive() const;

    /** @brief Return a snapshot of the active task (if any). */
    proto::ActiveTaskInfo GetSnapshot() const;

    /**
     * @brief Latch or clear the emergency-stop flag.
     * @param[in] estop New estop state.
     */
    void SetEstop(bool estop);

    /** @brief Check whether emergency-stop is latched. */
    bool CheckEstopActive() const;

private:
    mutable std::mutex mutex_;
    proto::TaskType active_type_{proto::TASK_TYPE_NONE};
    std::string cmd_id_;
    std::string client_id_;
    bool estop_{false};
};

}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
