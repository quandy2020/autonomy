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

#ifndef AUTONOMY_COMMON_TASK_HPP_
#define AUTONOMY_COMMON_TASK_HPP_

#include <mutex>
#include <set>

#include "autonomy/common/thread_pool.hpp"
#include "glog/logging.h"

namespace autonomy {
namespace common {

class ThreadPoolInterface;

class Task
{
public:
    friend class ThreadPoolInterface;

    using WorkItem = std::function<void()>;
    enum State { NEW, DISPATCHED, DEPENDENCIES_COMPLETED, RUNNING, COMPLETED };

    Task() = default;
    ~Task();

    State GetState();

    void SetWorkItem(const WorkItem& work_item);

    void AddDependency(std::weak_ptr<Task> dependency);

private:
    void AddDependentTask(Task* dependent_task);

    void Execute();

    void SetThreadPool(ThreadPoolInterface* thread_pool);

    void OnDependenyCompleted();

    WorkItem work_item_;
    ThreadPoolInterface* thread_pool_to_notify_ = nullptr;
    State state_ = NEW;
    unsigned int uncompleted_dependencies_ = 0;
    std::set<Task*> dependent_tasks_;

    std::mutex mutex_;
};

}  // namespace common
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_TASK_HPP_
