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

/**
 * @file ring_buffer_config.cpp
 * @brief Parse window_sec from a minimal YAML snippet.
 */

#include "robot/data_agent/ring_buffer/ring_buffer_config.hpp"

#include <cstdlib>

namespace autoloop {
namespace robot {

RingBufferConfig LoadRingBufferConfig(const std::string& yaml_text) {
    RingBufferConfig cfg;
    const auto pos = yaml_text.find("window_sec:");
    if (pos != std::string::npos) {
        const double sec = std::atof(yaml_text.c_str() + pos + 11);
        if (sec > 0) {
            cfg.window_ns = static_cast<int64_t>(sec * 1e9);
        }
    }
    return cfg;
}

}  // namespace robot
}  // namespace autoloop
