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
 * @file model_sync.hpp
 * @brief Pull published policy and atomically switch local pointer.
 */
#include "common/base/status.hpp"
#include "robot/model_sync/local_registry.hpp"

namespace autoloop {
namespace robot {

class ModelSync {
public:
    ModelSync(std::string registry_root, std::string cache_root);
    common::Status Sync(const std::string& src_artifact, const std::string& version);
    std::string Current() const { return registry_.Current(); }

private:
    LocalRegistry registry_;
    std::string cache_root_;
};

}  // namespace robot
}  // namespace autoloop
