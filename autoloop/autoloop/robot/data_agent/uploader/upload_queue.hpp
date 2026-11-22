/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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
/**
 * @file upload_queue.hpp
 * @brief P0–P3 FIFO of local episode directories.
 */
#include <deque>
#include <mutex>
#include <string>
#include <vector>
#include "robot/data_agent/trigger_manager/trigger_types.hpp"

namespace autoloop {
namespace robot {

struct UploadItem {
    std::string episode_dir;
    Priority priority{Priority::kP3};
    uint64_t offset{0};
};

class UploadQueue {
public:
    void Push(UploadItem item);
    bool Pop(UploadItem* item);
    size_t size() const;

private:
    mutable std::mutex mu_;
    std::deque<UploadItem> q_[4];
};

}  // namespace robot
}  // namespace autoloop
