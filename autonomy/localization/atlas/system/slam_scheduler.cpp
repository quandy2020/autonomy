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

/**
 * @file slam_scheduler.cpp
 * @brief SlamScheduler implementation: ThreadPool creation and Schedule enqueue.
 */

#include "autonomy/localization/atlas/system/slam_scheduler.hpp"

namespace autonomy {
namespace localization {
namespace atlas {

SlamScheduler::SlamScheduler(std::size_t num_threads, std::size_t max_tasks)
    : num_threads_(num_threads) {
    if (num_threads_ < 2) {
        num_threads_ = 2;
    }
    if (max_tasks < 16) {
        max_tasks = 16;
    }
    pool_ = std::make_unique<::autolink::base::ThreadPool>(num_threads_,
                                                           max_tasks);
}

SlamScheduler::~SlamScheduler() { pool_.reset(); }

void SlamScheduler::Schedule(WorkItem work) {
    if (!pool_ || !work) {
        return;
    }
    pool_->Enqueue(std::move(work));
}

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
