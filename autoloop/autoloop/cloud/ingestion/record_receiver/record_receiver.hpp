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
 * @file record_receiver.hpp
 * @brief Admit episode if hash + version quadruple pass.
 */
#include "common/base/status.hpp"
#include "cloud/ingestion/record_receiver/archive_store.hpp"

namespace autoloop {
namespace cloud {

class RecordReceiver {
public:
    explicit RecordReceiver(std::string archive_root);
    common::Status Ingest(const std::string& episode_dir);

private:
    ArchiveStore store_;
};

}  // namespace cloud
}  // namespace autoloop
