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

#pragma once

#include <list>
#include <memory>
#include <mutex>

#include "autolink/timer/timer_task.hpp"

namespace autolink {

class TimerBucket
{
public:
    void AddTask(const std::shared_ptr<TimerTask>& task) {
        std::lock_guard<std::mutex> lock(mutex_);
        task_list_.push_back(task);
    }

    std::mutex& mutex() {
        return mutex_;
    }
    std::list<std::weak_ptr<TimerTask>>& task_list() {
        return task_list_;
    }

private:
    std::mutex mutex_;
    std::list<std::weak_ptr<TimerTask>> task_list_;
};

}  // namespace autolink