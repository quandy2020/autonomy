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

#include "cloud/ingestion/record_parser/record_parser.hpp"
#include "cloud/ingestion/record_parser/mcap_chunk_reader.hpp"
#include "common/base/path_util.hpp"

namespace autoloop {
namespace cloud {

common::Status RecordParser::Parse(const std::string& episode_dir,
                                   std::vector<robot::Sample>* samples) {
    if (samples == nullptr) return common::Status::Error("null");
    const auto p = common::JoinPath(episode_dir, "snapshot.mcap");
    if (!ReadMcapSamples(p, samples)) return common::Status::Error("mcap");
    return common::Status::OK();
}

}  // namespace cloud
}  // namespace autoloop
