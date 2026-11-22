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

#include "cloud/ingestion/record_parser/channel_extractor.hpp"

namespace autoloop {
namespace cloud {

std::map<std::string, std::vector<robot::Sample>> GroupByChannel(
    const std::vector<robot::Sample>& samples) {
    std::map<std::string, std::vector<robot::Sample>> g;
    for (const auto& s : samples) g[s.channel].push_back(s);
    return g;
}

}  // namespace cloud
}  // namespace autoloop
