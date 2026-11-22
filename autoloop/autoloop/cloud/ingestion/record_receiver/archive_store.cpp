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

#include "cloud/ingestion/record_receiver/archive_store.hpp"
#include "common/base/path_util.hpp"

namespace autoloop {
namespace cloud {

ArchiveStore::ArchiveStore(std::string root) : root_(std::move(root)) {}

common::Status ArchiveStore::Put(const std::string& src_dir,
                                 const common::EpisodeSidecar& sc) {
    const auto dst = common::JoinPath(root_, sc.episode_id());
    if (!common::EnsureDirectory(dst)) return common::Status::Error("mkdir");
    for (const auto& f : common::ListFiles(src_dir)) {
        if (!common::CopyFile(common::JoinPath(src_dir, f), common::JoinPath(dst, f))) {
            return common::Status::Error(f);
        }
    }
    return common::Status::OK();
}

}  // namespace cloud
}  // namespace autoloop
