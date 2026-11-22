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

#include "robot/data_agent/uploader/uploader.hpp"
#include "common/base/path_util.hpp"
#include "robot/data_agent/uploader/resumable_upload.hpp"

namespace autoloop {
namespace robot {

Uploader::Uploader(std::string staging) : staging_(std::move(staging)) {}
void Uploader::Enqueue(UploadItem item) { q_.Push(std::move(item)); }
common::Status Uploader::PumpOnce() {
    UploadItem it;
    if (!q_.Pop(&it)) {
        return common::Status::OK();
    }
    common::EnsureDirectory(staging_);
    const auto slash = it.episode_dir.find_last_of('/');
    const std::string name = (slash == std::string::npos)
                                 ? it.episode_dir
                                 : it.episode_dir.substr(slash + 1);
    const auto dst = common::JoinPath(staging_, name);
    uint64_t off = it.offset;
    return ResumeCopy(it.episode_dir, dst, &off);
}

}  // namespace robot
}  // namespace autoloop
