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
 * @file mcap_exporter.hpp
 * @brief Minimal MCAP writer (no Foxglove SDK required).
 */

#include <string>
#include <vector>

#include "robot/data_agent/ring_buffer/ring_buffer.hpp"

namespace autoloop {
namespace robot {

/**
 * @brief Write samples as a tiny MCAP (magic + length-prefixed messages).
 * @param [in] path: Output file.
 * @param [in] samples: Window.
 * @return True on success.
 */
bool ExportMcap(const std::string& path, const std::vector<Sample>& samples);

}  // namespace robot
}  // namespace autoloop
