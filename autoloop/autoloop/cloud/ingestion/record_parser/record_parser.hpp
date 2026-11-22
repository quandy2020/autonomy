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
 * @file record_parser.hpp
 * @brief Parse an episode directory into samples.
 */
#include <string>
#include <vector>
#include "common/base/status.hpp"
#include "robot/data_agent/ring_buffer/ring_buffer.hpp"

namespace autoloop {
namespace cloud {

class RecordParser {
public:
    common::Status Parse(const std::string& episode_dir,
                         std::vector<robot::Sample>* samples);
};

}  // namespace cloud
}  // namespace autoloop
