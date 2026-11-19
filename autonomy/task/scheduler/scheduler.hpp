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

/** @file scheduler.hpp @brief Registry, revocable root ownership and native delegated navigation. */
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
#include <automsgs/task/task_options.pb.h>

namespace autonomy {
namespace task {

/**
 * @brief Coordinates registered TaskInterface plugins and movement ownership.
 *
 * Responsibilities (Nav2 NavigatorMuxer + session reaper):
 *   1. Own the plugin registry keyed by RobotTaskType.
 *   2. Guard navigation-class exclusivity, including exploration and its
 *      delegated NavigationTask. Ordinary navigation must acquire ownership
 *      during domain admission even when TaskServer skips RequestActivation.
 *   3. Reclaim finished native sessions only after admission and movement
 *      cleanup have drained; preserve reservations across initial submission.
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

    /** @brief Close admission, stop reclaim and dispose plugins; retain unconfirmed cleanup barriers. */
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
     * When preemption is enabled, retire the preceding root and delegated
     * permits, cancel outside scheduler locks, and wait up to two seconds for
     * native cleanup before granting a new generation. Unconfirmed cleanup
     * retains the retired ownership barrier and rejects activation.
     *
     * @return false if uninitialized, unregistered, or the slot cannot be
     *         acquired (preemption disabled / preempt failed).
     */
    [[nodiscard]] bool RequestActivation(TaskPointer task);

    /** @brief Release only the checked drained generation; active or pending work retains ownership. */
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

    /** @return Whether the reclaim worker is enabled; independent of domain admission. */
    [[nodiscard]] bool IsRunning() const;

    /** Snapshots of currently active (non-terminal) sessions. */
    [[nodiscard]] std::vector<proto::ActiveTaskSnapshot>
    GetActiveSnapshots() const;

    /** True when exclusive mode is enabled in SchedulerOptions. */
    [[nodiscard]] bool ExclusiveNavigationEnabled() const;

    /** True when a new activation may cancel the current exclusive holder. */
    [[nodiscard]] bool AllowPreemption() const;

    /**
     * Navigation-class tasks that share the exclusive slot when exclusivity
     * is enabled. Mapping / localization are intentionally excluded.
     */
    [[nodiscard]] static constexpr bool IsNavigationClass(
        RobotTaskType task_type) {
        switch (task_type) {
            case RobotTaskType::ROBOT_TASK_COVERAGE:
            case RobotTaskType::ROBOT_TASK_NAVIGATION:
            case RobotTaskType::ROBOT_TASK_FOLLOW:
            case RobotTaskType::ROBOT_TASK_TELEOP:
            case RobotTaskType::ROBOT_TASK_DOCK:
            case RobotTaskType::ROBOT_TASK_EXPLORATION:
                return true;
            default:
                return false;
        }
    }

private:
    class State;  ///< Shared ownership state; task callbacks never retain this scheduler object.
    std::shared_ptr<State> state_;  ///< Weak callback targets remain safe during scheduler destruction.
    mutable std::mutex worker_mutex_;  ///< Protects thread transfer, never held while joining a worker.
    std::thread reclaim_thread_;  ///< Reaper captures State rather than a raw scheduler pointer.
};

}  // namespace task
}  // namespace autonomy

#endif  // AUTONOMY_TASK_SCHEDULER_SCHEDULER_HPP_
