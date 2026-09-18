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

#pragma once

#include "autonomy/common/task.hpp"
#include "autonomy/common/thread_pool.hpp"

#include <functional>
#include <memory>
#include <utility>

namespace autonomy::localization::atlas {
namespace common_sched {

//! Fire-and-forget work on the shared Atlas ThreadPool (Cartographer-style Task).
inline std::weak_ptr<::autonomy::common::Task> Schedule(
    ::autonomy::common::ThreadPool* pool, std::function<void()> work) {
    if (!pool || !work) {
        return {};
    }
    auto task = std::make_unique<::autonomy::common::Task>();
    task->SetWorkItem(std::move(work));
    return pool->Schedule(std::move(task));
}

//! Schedule `dependent` to run after `dependency` completes (both on same pool).
inline std::weak_ptr<::autonomy::common::Task> ScheduleAfter(
    ::autonomy::common::ThreadPool* pool,
    const std::weak_ptr<::autonomy::common::Task>& dependency,
    std::function<void()> work) {
    if (!pool || !work) {
        return {};
    }
    auto task = std::make_unique<::autonomy::common::Task>();
    task->SetWorkItem(std::move(work));
    if (auto dep = dependency.lock()) {
        task->AddDependency(dependency);
    }
    return pool->Schedule(std::move(task));
}

}  // namespace common_sched
}  // namespace autonomy::localization::atlas
