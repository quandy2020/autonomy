/*
 * Copyright 2018 The Cartographer Authors
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

#include "autonomy/localization/cartographer/common/internal/testing/thread_pool_for_testing.hpp"

#ifndef WIN32
#include <unistd.h>
#endif
#include <algorithm>
#include <chrono>
#include <numeric>

#include "autonomy/localization/cartographer/common/task.hpp"
#include "autonomy/localization/cartographer/common/time.hpp"
#include "glog/logging.h"

namespace cartographer {
namespace common {
namespace testing {

ThreadPoolForTesting::ThreadPoolForTesting() : thread_([this]() { ThreadPoolForTesting::DoWork(); }) {}

ThreadPoolForTesting::~ThreadPoolForTesting() {
    {
        std::lock_guard<std::mutex> locker(mutex_);
        CHECK(running_);
        running_ = false;
        CHECK_EQ(task_queue_.size(), 0);
        CHECK_EQ(tasks_not_ready_.size(), 0);
    }
    task_available_cv_.notify_all();
    thread_.join();
}

void ThreadPoolForTesting::NotifyDependenciesCompleted(Task* task) {
    {
        std::lock_guard<std::mutex> locker(mutex_);
        CHECK(running_);
        auto it = tasks_not_ready_.find(task);
        CHECK(it != tasks_not_ready_.end());
        task_queue_.push_back(it->second);
        tasks_not_ready_.erase(it);
    }
    task_available_cv_.notify_one();
}

std::weak_ptr<Task> ThreadPoolForTesting::Schedule(std::unique_ptr<Task> task) {
    std::shared_ptr<Task> shared_task;
    {
        std::lock_guard<std::mutex> locker(mutex_);
        idle_ = false;
        CHECK(running_);
        auto insert_result = tasks_not_ready_.insert(std::make_pair(task.get(), std::move(task)));
        CHECK(insert_result.second) << "ScheduleWhenReady called twice";
        shared_task = insert_result.first->second;
    }
    SetThreadPool(shared_task.get());
    return shared_task;
}

void ThreadPoolForTesting::WaitUntilIdle() {
    for (;;) {
        std::unique_lock<std::mutex> locker(mutex_);
        if (task_available_cv_.wait_for(locker, common::FromSeconds(0.1), [this] { return idle_; })) {
            return;
        }
    }
}

void ThreadPoolForTesting::DoWork() {
    for (;;) {
        std::shared_ptr<Task> task;
        {
            std::unique_lock<std::mutex> locker(mutex_);
            task_available_cv_.wait_for(locker, common::FromSeconds(0.1),
                                        [this] { return !task_queue_.empty() || !running_; });
            if (!task_queue_.empty()) {
                task = task_queue_.front();
                task_queue_.pop_front();
            }
            if (!running_) {
                return;
            }
            if (tasks_not_ready_.empty() && task_queue_.empty() && !task) {
                idle_ = true;
                task_available_cv_.notify_all();
            }
        }
        if (task) {
            Execute(task.get());
        }
    }
}

}  // namespace testing
}  // namespace common
}  // namespace cartographer
