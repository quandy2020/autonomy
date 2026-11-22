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

#include "robot/model_sync/model_sync.hpp"
#include "common/base/path_util.hpp"
#include "robot/model_sync/model_fetcher.hpp"

namespace autoloop {
namespace robot {

ModelSync::ModelSync(std::string registry_root, std::string cache_root)
    : registry_(std::move(registry_root)), cache_root_(std::move(cache_root)) {}

common::Status ModelSync::Sync(const std::string& src_artifact,
                               const std::string& version) {
    common::EnsureDirectory(cache_root_);
    const auto dst = common::JoinPath(cache_root_, version);
    auto st = FetchModel(src_artifact, dst);
    if (!st.ok()) return st;
    return registry_.SwitchTo(version);
}

}  // namespace robot
}  // namespace autoloop
