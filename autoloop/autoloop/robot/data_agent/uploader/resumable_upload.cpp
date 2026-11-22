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

#include "robot/data_agent/uploader/resumable_upload.hpp"
#include "common/base/path_util.hpp"

namespace autoloop {
namespace robot {

common::Status ResumeCopy(const std::string& src_dir, const std::string& dst_dir,
                          uint64_t* offset) {
    if (!common::EnsureDirectory(dst_dir)) {
        return common::Status::Error("dst");
    }
    const auto files = common::ListFiles(src_dir);
    uint64_t n = 0;
    for (const auto& f : files) {
        const auto from = common::JoinPath(src_dir, f);
        const auto to = common::JoinPath(dst_dir, f);
        if (!common::CopyFile(from, to)) {
            return common::Status::Error("copy " + f);
        }
        ++n;
    }
    if (offset) *offset = n;
    return common::Status::OK();
}

}  // namespace robot
}  // namespace autoloop
