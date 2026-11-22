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
 * @file ring_buffer_config.hpp
 * @brief Ring buffer window / capacity from YAML-like key values.
 */

#include <cstdint>
#include <string>

namespace autoloop {
namespace robot {

struct RingBufferConfig {
    int64_t window_ns{2'000'000'000};
    size_t max_samples{4096};
};

RingBufferConfig LoadRingBufferConfig(const std::string& yaml_text);

}  // namespace robot
}  // namespace autoloop
