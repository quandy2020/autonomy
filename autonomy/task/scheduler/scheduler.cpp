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
#include <thread>
#include <utility>

#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace task {
namespace {

using RobotTaskType = ::autonomy::commsgs::proto::vehicle_msgs::RobotTaskType;

std::chrono::milliseconds FeedbackPeriod(
    const ::autonomy::task::proto::SchedulerOptions& options)
{
    const auto ms = options.feedback_period_ms();
    if (ms == 0) {
        return std::chrono::milliseconds(100);
    }
    return std::chrono::milliseconds(ms);
}

void RunSteps(const std::vector<TaskPipelineStep>& steps)
{
    for (const auto& step : steps) {
        if (step) {
            step();
        }
    }
}

}  // namespace

struct TaskScheduler::Runtime {
    std::size_t worker_count{0};
    std::thread feedback_thread;
    std::mutex async_mutex;
    std::vector<std::thread> async_threads;

    explicit Runtime(std::size_t workers) : worker_count(workers) {}

    void JoinAsyncThreads()
    {
        std::vector<std::thread> pending;
        {
            std::lock_guard<std::mutex> lock(async_mutex);
            pending.swap(async_threads);
        }
        for (auto& thread : pending) {
            if (thread.joinable()) {
                thread.join();
            }
        }
    }
};

TaskScheduler::TaskScheduler(::autonomy::task::proto::SchedulerOptions options)
    : options_(std::move(options)),
      mux_(options_.exclusive_navigation_tasks()) {}

TaskScheduler::~TaskScheduler() { Stop(); }

bool TaskScheduler::Initialize(
    const ::autonomy::task::proto::TaskServerOptions& options)
{
    std::lock_guard<std::mutex> lock(mutex_);
    server_options_ = options;
    if (options.has_scheduler()) {
        options_ = options.scheduler();
        mux_.Reset(options_.exclusive_navigation_tasks());
    }

    const auto workers = std::max<std::size_t>(
        2U, std::thread::hardware_concurrency());
    runtime_ = std::make_unique<Runtime>(workers);
    initialized_.store(true);
    AINFO << "TaskScheduler initialized with " << workers << " workers, "
          << "feedback_period_ms=" << options_.feedback_period_ms();
    return true;
}

void TaskScheduler::Shutdown()
{
    Stop();

    std::vector<TaskInterface::SharedPtr> plugins;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        plugins.reserve(registry_.size());
        for (const auto& entry : registry_) {
            plugins.push_back(entry.second);
        }
        registry_.clear();
        active_sessions_.clear();
    }

    for (const auto& plugin : plugins) {
        if (plugin) {
            plugin->Shutdown();
        }
    }

    mux_.Reset(options_.exclusive_navigation_tasks());
    if (runtime_) {
        runtime_->JoinAsyncThreads();
    }
    runtime_.reset();
    initialized_.store(false);
}

void TaskScheduler::RegisterTask(TaskInterface::SharedPtr task)
{
    if (!task) {
        AWARN << "TaskScheduler::RegisterTask ignored null plugin";
        return;
    }

    const auto type = task->GetTaskType();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        registry_[static_cast<int>(type)] = task;
    }

    if (!initialized_.load()) {
        AWARN << "TaskScheduler plugin registered before Initialize: "
              << static_cast<int>(type);
        return;
    }

    if (!task->Initialize(server_options_)) {
        AERROR << "TaskScheduler failed to initialize plugin type="
               << static_cast<int>(type);
    } else {
        AINFO << "TaskScheduler registered plugin type="
              << static_cast<int>(type);
    }
}

TaskInterface::SharedPtr TaskScheduler::GetTask(RobotTaskType type) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = registry_.find(static_cast<int>(type));
    if (it == registry_.end()) {
        return nullptr;
    }
    return it->second;
}

bool TaskScheduler::RequestActivation(TaskInterface::SharedPtr task)
{
    if (!task || !initialized_.load()) {
        return false;
    }

    const auto type = task->GetTaskType();
    if (!mux_.TryAcquire(type)) {
        AWARN << "TaskScheduler rejected activation: mux busy with type="
              << static_cast<int>(mux_.ActiveNavigationTask())
              << ", requested=" << static_cast<int>(type);
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = registry_.find(static_cast<int>(type));
    if (it == registry_.end() || it->second != task) {
        mux_.Release(type);
        return false;
    }

    active_sessions_.push_back(task);
    AINFO << "TaskScheduler activated plugin type=" << static_cast<int>(type);
    return true;
}

void TaskScheduler::ReleaseActivation(TaskInterface::SharedPtr task)
{
    if (!task) {
        return;
    }

    const auto type = task->GetTaskType();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        active_sessions_.erase(
            std::remove(active_sessions_.begin(), active_sessions_.end(), task),
            active_sessions_.end());
    }
    mux_.Release(type);
    AINFO << "TaskScheduler released plugin type=" << static_cast<int>(type);
}

bool TaskScheduler::CancelActive(RobotTaskType type)
{
    const auto task = GetTask(type);
    if (!task || !task->IsActive()) {
        return false;
    }
    return task->Cancel();
}

bool TaskScheduler::PauseActive(RobotTaskType type)
{
    const auto task = GetTask(type);
    if (!task || !task->IsActive()) {
        return false;
    }
    return task->Pause();
}

bool TaskScheduler::ResumeActive(RobotTaskType type)
{
    const auto task = GetTask(type);
    if (!task || task->GetLifecycle() != TaskLifecycle::kPaused) {
        return false;
    }
    return task->Resume();
}

bool TaskScheduler::Start()
{
    if (!initialized_.load() || !runtime_) {
        AERROR << "TaskScheduler::Start called before Initialize";
        return false;
    }
    if (running_.exchange(true)) {
        return true;
    }

    const auto period = FeedbackPeriod(options_);
    runtime_->feedback_thread = std::thread([this, period]() {
        while (running_.load()) {
            PollActiveTasks();
            std::this_thread::sleep_for(period);
        }
    });

    AINFO << "TaskScheduler feedback loop started";
    return true;
}

void TaskScheduler::Stop()
{
    if (!running_.exchange(false)) {
        if (runtime_) {
            runtime_->JoinAsyncThreads();
        }
        return;
    }
    if (runtime_ && runtime_->feedback_thread.joinable()) {
        runtime_->feedback_thread.join();
    }
    if (runtime_) {
        runtime_->JoinAsyncThreads();
    }
    AINFO << "TaskScheduler feedback loop stopped";
}

std::vector<::autonomy::task::proto::ActiveTaskSnapshot>
TaskScheduler::GetActiveSnapshots() const
{
    std::vector<TaskInterface::SharedPtr> sessions;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        sessions = active_sessions_;
    }

    std::vector<::autonomy::task::proto::ActiveTaskSnapshot> snapshots;
    snapshots.reserve(sessions.size());
    for (const auto& task : sessions) {
        if (!task || !task->IsActive()) {
            continue;
        }
        snapshots.push_back(task->GetSnapshot());
    }
    return snapshots;
}

void TaskScheduler::RunPipeline(const std::vector<TaskPipelineStep>& steps)
{
    if (!runtime_) {
        AERROR << "TaskScheduler::RunPipeline called before Initialize";
        return;
    }
    RunSteps(steps);
}

void TaskScheduler::RunPipelineAsync(
    const std::vector<TaskPipelineStep>& steps)
{
    if (!runtime_) {
        AERROR << "TaskScheduler::RunPipelineAsync called before Initialize";
        return;
    }
    if (steps.empty()) {
        return;
    }

    std::lock_guard<std::mutex> lock(runtime_->async_mutex);
    runtime_->async_threads.emplace_back([steps]() { RunSteps(steps); });
}

std::size_t TaskScheduler::WorkerCount() const
{
    if (!runtime_) {
        return 0;
    }
    return runtime_->worker_count;
}

void TaskScheduler::PollActiveTasks()
{
    PruneInactiveSessions();
}

void TaskScheduler::PruneInactiveSessions()
{
    std::vector<TaskInterface::SharedPtr> finished;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = active_sessions_.begin();
        while (it != active_sessions_.end()) {
            const auto& task = *it;
            if (!task) {
                it = active_sessions_.erase(it);
                continue;
            }
            if (IsTerminalLifecycle(task->GetLifecycle())) {
                finished.push_back(task);
                it = active_sessions_.erase(it);
                continue;
            }
            ++it;
        }
    }

    for (const auto& task : finished) {
        mux_.Release(task->GetTaskType());
        AINFO << "TaskScheduler pruned finished plugin type="
              << static_cast<int>(task->GetTaskType());
    }
}

}  // namespace task
}  // namespace autonomy
