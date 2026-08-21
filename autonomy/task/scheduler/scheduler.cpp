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

#include "autonomy/task/scheduler/scheduler.hpp"

#include <algorithm>
#include <utility>

#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace task {

TaskScheduler::TaskScheduler(proto::SchedulerOptions options)
    : options_(std::move(options)),
      exclusive_navigation_enabled_(
          options_.exclusive_navigation_tasks()),
      allow_preemption_(options_.has_allow_preemption()
                            ? options_.allow_preemption()
                            : true) {}

TaskScheduler::~TaskScheduler() { Stop(); }

bool TaskScheduler::Initialize(const proto::TaskServerOptions& server_options) {
    std::lock_guard<std::mutex> lock(mutex_);
    server_options_ = server_options;
    if (server_options.has_scheduler()) {
        options_ = server_options.scheduler();
        exclusive_navigation_enabled_ = options_.exclusive_navigation_tasks();
        allow_preemption_ = options_.has_allow_preemption()
                                ? options_.allow_preemption()
                                : true;
        exclusive_holder_ = RobotTaskType::ROBOT_TASK_NONE;
    }
    initialized_.store(true, std::memory_order_release);
    AINFO << "TaskScheduler initialized exclusive_navigation="
          << exclusive_navigation_enabled_
          << " allow_preemption=" << allow_preemption_
          << " feedback_period_ms=" << FeedbackPeriod().count();
    return true;
}

void TaskScheduler::Shutdown() {
    Stop();

    std::vector<TaskPointer> plugins;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        plugins.reserve(registry_.size());
        for (const auto& [unused_type, task] : registry_) {
            (void)unused_type;
            plugins.push_back(task);
        }
        registry_.clear();
        active_sessions_.clear();
        exclusive_holder_ = RobotTaskType::ROBOT_TASK_NONE;
    }

    for (const auto& task : plugins) {
        if (task) {
            task->Shutdown();
        }
    }
    initialized_.store(false, std::memory_order_release);
}

void TaskScheduler::RegisterTask(TaskPointer task) {
    if (!task) {
        return;
    }

    const RobotTaskType task_type = task->GetTaskType();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        registry_[static_cast<int>(task_type)] = task;
    }

    if (!initialized_.load(std::memory_order_acquire)) {
        return;
    }

    if (!task->Initialize(server_options_)) {
        AERROR << "TaskScheduler::RegisterTask Initialize failed type="
               << static_cast<int>(task_type);
    }
}

TaskScheduler::TaskPointer TaskScheduler::GetTask(
    RobotTaskType task_type) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto iterator = registry_.find(static_cast<int>(task_type));
    return iterator == registry_.end() ? nullptr : iterator->second;
}

bool TaskScheduler::RequestActivation(TaskPointer task) {
    if (!task || !initialized_.load(std::memory_order_acquire)) {
        return false;
    }

    const RobotTaskType task_type = task->GetTaskType();
    for (int attempt = 0; attempt < 2; ++attempt) {
        RobotTaskType holder = RobotTaskType::ROBOT_TASK_NONE;
        if (!TryAcquireExclusiveSlot(task_type, &holder)) {
            if (attempt > 0 || !allow_preemption_ ||
                holder == RobotTaskType::ROBOT_TASK_NONE) {
                AWARN << "TaskScheduler::RequestActivation rejected: slot held "
                         "by "
                      << static_cast<int>(holder)
                      << ", requested=" << static_cast<int>(task_type)
                      << " allow_preemption=" << allow_preemption_;
                return false;
            }
            if (!PreemptExclusiveHolder(holder)) {
                AWARN << "TaskScheduler::RequestActivation preempt failed "
                         "holder="
                      << static_cast<int>(holder);
                return false;
            }
            continue;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        const auto iterator = registry_.find(static_cast<int>(task_type));
        if (iterator == registry_.end() || iterator->second != task) {
            if (exclusive_navigation_enabled_ && IsNavigationClass(task_type) &&
                exclusive_holder_ == task_type) {
                exclusive_holder_ = RobotTaskType::ROBOT_TASK_NONE;
            }
            return false;
        }

        active_sessions_.push_back(task);
        return true;
    }
    return false;
}

bool TaskScheduler::PreemptExclusiveHolder(RobotTaskType holder_type) {
    if (holder_type == RobotTaskType::ROBOT_TASK_NONE) {
        return false;
    }

    TaskPointer victim = GetTask(holder_type);
    AINFO << "TaskScheduler preempting exclusive holder type="
          << static_cast<int>(holder_type);
    if (victim && victim->IsActive()) {
        victim->Cancel();
    }
    if (victim) {
        ReleaseActivation(victim);
    } else {
        ReleaseExclusiveSlot(holder_type);
    }
    return true;
}

void TaskScheduler::ReleaseActivation(TaskPointer task) {
    if (!task) {
        return;
    }

    const RobotTaskType task_type = task->GetTaskType();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        active_sessions_.erase(
            std::remove(active_sessions_.begin(), active_sessions_.end(),
                        task),
            active_sessions_.end());
    }
    ReleaseExclusiveSlot(task_type);
}

bool TaskScheduler::Start() {
    if (!initialized_.load(std::memory_order_acquire)) {
        AERROR << "TaskScheduler::Start before Initialize";
        return false;
    }
    if (running_.exchange(true, std::memory_order_acq_rel)) {
        return true;
    }
    reclaim_thread_ = std::thread([this] { ReclaimLoop(); });
    return true;
}

void TaskScheduler::Stop() {
    if (!running_.exchange(false, std::memory_order_acq_rel)) {
        return;
    }
    if (reclaim_thread_.joinable()) {
        reclaim_thread_.join();
    }
}

std::vector<proto::ActiveTaskSnapshot> TaskScheduler::GetActiveSnapshots()
    const {
    std::vector<TaskPointer> sessions;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        sessions = active_sessions_;
    }

    std::vector<proto::ActiveTaskSnapshot> snapshots;
    snapshots.reserve(sessions.size());
    for (const auto& task : sessions) {
        if (task && task->IsActive()) {
            snapshots.push_back(task->GetSnapshot());
        }
    }
    return snapshots;
}

bool TaskScheduler::TryAcquireExclusiveSlot(RobotTaskType task_type,
                                            RobotTaskType* holder_snapshot) {
    if (!exclusive_navigation_enabled_ || !IsNavigationClass(task_type)) {
        return true;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (exclusive_holder_ != RobotTaskType::ROBOT_TASK_NONE) {
        if (holder_snapshot != nullptr) {
            *holder_snapshot = exclusive_holder_;
        }
        return false;
    }
    exclusive_holder_ = task_type;
    return true;
}

void TaskScheduler::ReleaseExclusiveSlot(RobotTaskType task_type) {
    if (!exclusive_navigation_enabled_ || !IsNavigationClass(task_type)) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (exclusive_holder_ == task_type) {
        exclusive_holder_ = RobotTaskType::ROBOT_TASK_NONE;
    }
}

void TaskScheduler::ReclaimLoop() {
    const auto period = FeedbackPeriod();
    while (running_.load(std::memory_order_acquire)) {
        ReclaimFinishedSessions();
        std::this_thread::sleep_for(period);
    }
}

void TaskScheduler::ReclaimFinishedSessions() {
    std::vector<TaskPointer> finished;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto iterator = active_sessions_.begin();
        while (iterator != active_sessions_.end()) {
            const TaskPointer& task = *iterator;
            if (!task) {
                iterator = active_sessions_.erase(iterator);
                continue;
            }
            if (IsTerminalLifecycle(task->GetLifecycle())) {
                finished.push_back(task);
                iterator = active_sessions_.erase(iterator);
                continue;
            }
            ++iterator;
        }
    }

    for (const auto& task : finished) {
        ReleaseExclusiveSlot(task->GetTaskType());
    }
}

}  // namespace task
}  // namespace autonomy
