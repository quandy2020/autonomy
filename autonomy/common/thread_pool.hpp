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

#ifndef AUTONOMY_COMMON_THREAD_POOL_HPP_
#define AUTONOMY_COMMON_THREAD_POOL_HPP_

#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

#include "autonomy/common/task.hpp"

namespace autonomy {
namespace common {

class Task;

class ThreadPoolInterface
{
public:
    ThreadPoolInterface() {}
    virtual ~ThreadPoolInterface() {}
    virtual std::weak_ptr<Task> Schedule(std::unique_ptr<Task> task) = 0;

protected:
    void Execute(Task* task);
    void SetThreadPool(Task* task);

private:
    friend class Task;

    virtual void NotifyDependenciesCompleted(Task* task) = 0;
};

class ThreadPool : public ThreadPoolInterface
{
public:
    explicit ThreadPool(int num_threads);
    ~ThreadPool();

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    std::weak_ptr<Task> Schedule(std::unique_ptr<Task> task) override;

private:
    void DoWork();

    void NotifyDependenciesCompleted(Task* task) override;

    std::mutex mutex_;
    std::condition_variable cv_;
    bool running_ = true;
    std::vector<std::thread> pool_;
    std::deque<std::shared_ptr<Task>> task_queue_;
    std::unordered_map<Task*, std::shared_ptr<Task>> tasks_not_ready_;
};

}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_THREAD_POOL_HPP_
