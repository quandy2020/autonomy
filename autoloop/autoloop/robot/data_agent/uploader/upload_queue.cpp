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

#include "robot/data_agent/uploader/upload_queue.hpp"

namespace autoloop {
namespace robot {

void UploadQueue::Push(UploadItem item) {
    std::lock_guard<std::mutex> lock(mu_);
    const int i = static_cast<int>(item.priority);
    q_[i < 0 ? 0 : (i > 3 ? 3 : i)].push_back(std::move(item));
}
bool UploadQueue::Pop(UploadItem* item) {
    if (item == nullptr) return false;
    std::lock_guard<std::mutex> lock(mu_);
    for (int i = 0; i < 4; ++i) {
        if (!q_[i].empty()) {
            *item = std::move(q_[i].front());
            q_[i].pop_front();
            return true;
        }
    }
    return false;
}
size_t UploadQueue::size() const {
    std::lock_guard<std::mutex> lock(mu_);
    size_t n = 0;
    for (int i = 0; i < 4; ++i) n += q_[i].size();
    return n;
}

}  // namespace robot
}  // namespace autoloop
