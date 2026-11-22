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
 * @file event_writer.hpp
 * @brief JSONL event append (no autolink append API).
 */

#include <fstream>
#include <mutex>
#include <string>

#include "robot/data_agent/structured_log/event_code.hpp"

namespace autoloop {
namespace robot {

class EventWriter {
public:
    explicit EventWriter(std::string path);
    void Write(EventCode code, const std::string& payload_json);

private:
    std::string path_;
    std::mutex mu_;
};

}  // namespace robot
}  // namespace autoloop
