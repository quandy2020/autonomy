/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#include "autonomy/common/thread_pool.hpp"

#ifndef WIN32
#include <unistd.h>
#endif
#include <algorithm>
#include <chrono>
#include <numeric>

#include "autonomy/common/task.hpp"
#include "glog/logging.h"

namespace autonomy {
namespace common {

void ThreadPoolInterface::Execute(Task* task) {
    task->Execute();
}

void ThreadPoolInterface::SetThreadPool(Task* task) {
    task->SetThreadPool(this);
}

ThreadPool::ThreadPool(int num_threads) {
    CHECK_GT(num_threads, 0) << "ThreadPool requires a positive num_threads!";
    std::lock_guard<std::mutex> locker(mutex_);
    for (int i = 0; i != num_threads; ++i) {
        pool_.emplace_back([this]() { ThreadPool::DoWork(); });
    }
}

ThreadPool::~ThreadPool() {
    {
        std::lock_guard<std::mutex> locker(mutex_);
        CHECK(running_);
        running_ = false;
    }
    cv_.notify_all();
    for (std::thread& thread : pool_) {
        thread.join();
    }
}

void ThreadPool::NotifyDependenciesCompleted(Task* task) {
    std::lock_guard<std::mutex> locker(mutex_);
    auto it = tasks_not_ready_.find(task);
    CHECK(it != tasks_not_ready_.end());
    task_queue_.push_back(it->second);
    tasks_not_ready_.erase(it);
    cv_.notify_one();
}

std::weak_ptr<Task> ThreadPool::Schedule(std::unique_ptr<Task> task) {
    std::shared_ptr<Task> shared_task;
    {
        std::lock_guard<std::mutex> locker(mutex_);
        auto insert_result = tasks_not_ready_.insert(
            std::make_pair(task.get(), std::move(task)));
        CHECK(insert_result.second) << "Schedule called twice";
        shared_task = insert_result.first->second;
    }
    SetThreadPool(shared_task.get());
    return shared_task;
}

void ThreadPool::DoWork() {
#ifdef __linux__
    CHECK_NE(nice(10), -1);
#endif
    for (;;) {
        std::shared_ptr<Task> task;
        {
            std::unique_lock<std::mutex> locker(mutex_);
            cv_.wait(locker, [this]() {
                return !task_queue_.empty() || !running_;
            });
            if (!task_queue_.empty()) {
                task = std::move(task_queue_.front());
                task_queue_.pop_front();
            } else if (!running_) {
                return;
            }
        }
        CHECK(task);
        CHECK_EQ(task->GetState(), common::Task::DEPENDENCIES_COMPLETED);
        Execute(task.get());
    }
}

}  // namespace common
}  // namespace autonomy
