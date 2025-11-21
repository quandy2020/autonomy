/**
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include "autolink/task/task_manager.hpp"

#include "autolink/common/global_data.hpp"
#include "autolink/croutine/croutine.hpp"
#include "autolink/croutine/routine_factory.hpp"
#include "autolink/scheduler/scheduler_factory.hpp"

namespace autolink {

using autolink::common::GlobalData;
static const char* const task_prefix = "/internal/task";

TaskManager::TaskManager()
    : task_queue_size_(1000),
      task_queue_(
          std::make_shared<base::BoundedQueue<std::function<void()>>>()) {
    // BoundedQueue::Init will take ownership of the WaitStrategy
    auto wait_strategy = std::make_unique<base::BlockWaitStrategy>();
    if (!task_queue_->Init(task_queue_size_, wait_strategy.release())) {
        AERROR << "Task queue init failed";
        throw std::runtime_error("Task queue init failed");
    }
    auto func = [this]() {
        while (!stop_) {
            std::function<void()> task;
            if (!task_queue_->Dequeue(&task)) {
                auto routine = croutine::CRoutine::GetCurrentRoutine();
                routine->HangUp();
                continue;
            }
            task();
        }
    };

    num_threads_ = scheduler::Instance()->TaskPoolSize();
    auto factory = croutine::CreateRoutineFactory(std::move(func));
    tasks_.reserve(num_threads_);
    for (uint32_t i = 0; i < num_threads_; i++) {
        auto task_name = task_prefix + std::to_string(i);
        tasks_.push_back(common::GlobalData::RegisterTaskName(task_name));
        if (!scheduler::Instance()->CreateTask(factory, task_name)) {
            AERROR << "CreateTask failed:" << task_name;
        }
    }
}

TaskManager::~TaskManager() {
    Shutdown();
}

void TaskManager::Shutdown() {
    if (stop_.exchange(true)) {
        return;
    }
    for (uint32_t i = 0; i < num_threads_; i++) {
        scheduler::Instance()->RemoveTask(task_prefix + std::to_string(i));
    }
}

}  // namespace autolink