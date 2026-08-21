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

#ifndef AUTONOMY_TASK_SCHEDULER_SCHEDULER_HPP_
#define AUTONOMY_TASK_SCHEDULER_SCHEDULER_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "autonomy/common/macros.hpp"
#include <automsgs/msgs/vehicle_msgs/robot_task_type.pb.h>
#include "autonomy/task/common/task_interface.hpp"
#include "autonomy/task/proto/task_options.pb.h"

namespace autonomy {
namespace task {

/**
 * Coordinates registered TaskInterface plugins.
 *
 * Responsibilities (Nav2 NavigatorMuxer + session reaper):
 *   1. Own the plugin registry keyed by RobotTaskType.
 *   2. Guard navigation-class exclusivity (navigation / follow / teleop /
 *      exploration / dock share one active slot when configured).
 *   3. Track active sessions and reclaim them when they reach a terminal
 *      lifecycle state.
 *
 * Not responsible for behavior-tree ticking or outbound planner/controller
 * RPCs; those stay inside each TaskInterface implementation.
 */
class TaskScheduler
{
public:
    using RobotTaskType = ::automsgs::msgs::vehicle_msgs::RobotTaskType;
    using TaskPointer = TaskInterface::SharedPtr;

    AUTONOMY_SMART_PTR_DEFINITIONS(TaskScheduler)

    explicit TaskScheduler(proto::SchedulerOptions options = {});
    ~TaskScheduler();

    TaskScheduler(const TaskScheduler&) = delete;
    TaskScheduler& operator=(const TaskScheduler&) = delete;

    /**
     * Applies server options and arms the scheduler for RegisterTask.
     * Idempotent with respect to repeated calls after Shutdown.
     */
    bool Initialize(const proto::TaskServerOptions& server_options);

    /** Stops reclaim, shuts down every registered plugin, clears state. */
    void Shutdown();

    /**
     * Inserts @p task into the registry and, if already Initialize()'d,
     * calls TaskInterface::Initialize with the stored server options.
     */
    void RegisterTask(TaskPointer task);

    /** Lookup by RobotTaskType; returns nullptr when unregistered. */
    [[nodiscard]] TaskPointer GetTask(RobotTaskType task_type) const;

    /**
     * Tries to take the exclusive navigation slot (when required) and
     * appends @p task to the active-session list.
     *
     * When allow_preemption is enabled (default) and the slot is held by
     * another navigation-class task, that holder is cancelled and the slot
     * is granted to @p task.
     *
     * @return false if uninitialized, unregistered, or the slot cannot be
     *         acquired (preemption disabled / preempt failed).
     */
    [[nodiscard]] bool RequestActivation(TaskPointer task);

    /** Removes @p task from active sessions and frees the exclusive slot. */
    void ReleaseActivation(TaskPointer task);

    /**
     * Invokes a member of TaskInterface on the plugin for @p task_type when
     * @p predicate holds. Example:
     *   InvokeIf(type, &TaskInterface::IsActive, &TaskInterface::Cancel);
     */
    template <typename Predicate, typename MemberFunction>
    [[nodiscard]] bool InvokeIf(RobotTaskType task_type, Predicate predicate,
                                MemberFunction member_function) {
        const TaskPointer task = GetTask(task_type);
        if (!task) {
            return false;
        }
        if constexpr (std::is_member_function_pointer_v<Predicate>) {
            if (!(task.get()->*predicate)()) {
                return false;
            }
        } else {
            if (!predicate(*task)) {
                return false;
            }
        }
        if constexpr (std::is_member_function_pointer_v<MemberFunction>) {
            return (task.get()->*member_function)();
        } else {
            return member_function(*task);
        }
    }

    [[nodiscard]] bool CancelTask(RobotTaskType task_type) {
        return InvokeIf(task_type, &TaskInterface::IsActive,
                        &TaskInterface::Cancel);
    }

    [[nodiscard]] bool PauseTask(RobotTaskType task_type) {
        return InvokeIf(task_type, &TaskInterface::IsActive,
                        &TaskInterface::Pause);
    }

    [[nodiscard]] bool ResumeTask(RobotTaskType task_type) {
        return InvokeIf(
            task_type,
            [](const TaskInterface& task) {
                return task.GetLifecycle() == TaskLifecycle::kPaused;
            },
            &TaskInterface::Resume);
    }

    /** Starts the background session-reclaim loop. Idempotent. */
    bool Start();

    /** Stops the reclaim loop and joins the worker thread. */
    void Stop();

    [[nodiscard]] bool IsRunning() const {
        return running_.load(std::memory_order_acquire);
    }

    /** Snapshots of currently active (non-terminal) sessions. */
    [[nodiscard]] std::vector<proto::ActiveTaskSnapshot>
    GetActiveSnapshots() const;

    /** True when exclusive mode is enabled in SchedulerOptions. */
    [[nodiscard]] bool ExclusiveNavigationEnabled() const {
        return exclusive_navigation_enabled_;
    }

    /** True when a new activation may cancel the current exclusive holder. */
    [[nodiscard]] bool AllowPreemption() const { return allow_preemption_; }

    /**
     * Navigation-class tasks that share the exclusive slot when exclusivity
     * is enabled. Mapping / localization are intentionally excluded.
     */
    [[nodiscard]] static constexpr bool IsNavigationClass(
        RobotTaskType task_type) {
        switch (task_type) {
            case RobotTaskType::ROBOT_TASK_NAVIGATION:
            case RobotTaskType::ROBOT_TASK_FOLLOW:
            case RobotTaskType::ROBOT_TASK_TELEOP:
            case RobotTaskType::ROBOT_TASK_EXPLORATION:
            case RobotTaskType::ROBOT_TASK_DOCK:
                return true;
            default:
                return false;
        }
    }

private:
    [[nodiscard]] std::chrono::milliseconds FeedbackPeriod() const {
        const int milliseconds = options_.feedback_period_ms();
        return std::chrono::milliseconds(milliseconds > 0 ? milliseconds
                                                          : 100);
    }

    // Returns false and leaves @p holder_snapshot unchanged on failure.
    [[nodiscard]] bool TryAcquireExclusiveSlot(
        RobotTaskType task_type, RobotTaskType* holder_snapshot = nullptr);
    void ReleaseExclusiveSlot(RobotTaskType task_type);

    // Cancels the holder (if still registered) and frees the exclusive slot.
    [[nodiscard]] bool PreemptExclusiveHolder(RobotTaskType holder_type);

    void ReclaimFinishedSessions();
    void ReclaimLoop();

    proto::SchedulerOptions options_;
    proto::TaskServerOptions server_options_;
    bool exclusive_navigation_enabled_ = true;
    bool allow_preemption_ = true;

    mutable std::mutex mutex_;
    std::unordered_map<int, TaskPointer> registry_;
    std::vector<TaskPointer> active_sessions_;

    // Exclusive navigation-class holder (ROBOT_TASK_NONE == free).
    RobotTaskType exclusive_holder_ = RobotTaskType::ROBOT_TASK_NONE;

    std::thread reclaim_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> initialized_{false};
};

}  // namespace task
}  // namespace autonomy

#endif  // AUTONOMY_TASK_SCHEDULER_SCHEDULER_HPP_
