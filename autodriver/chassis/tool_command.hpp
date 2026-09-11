/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file tool_command.hpp
 * @brief Job/tool channel separate from locomotion twist (brush, blade, …).
 */

#ifndef AUTODRIVER_CHASSIS_TOOL_COMMAND_HPP_
#define AUTODRIVER_CHASSIS_TOOL_COMMAND_HPP_

#include <string>

namespace autodriver {
namespace chassis {

/**
 * @brief Parsed tool command from std_msgs/String.
 *
 * Formats accepted:
 * - `name=enable` / `name=disable` / `name=1` / `name=0`
 * - `name:value` (value kept as string for vendor)
 * - bare `name` → enable=true
 */
struct ToolCommand {
  std::string name;   ///< Tool id from CapabilityProfile.tools
  std::string value;  ///< Vendor-specific payload (often "1"/"0")
  bool enable = true;
};

/**
 * @brief Parse a tool command string.
 * @param[in] text Raw Autolink payload.
 * @param[out] out Filled on success.
 * @return false when @p text is empty or has no tool name.
 */
bool ParseToolCommand(const std::string& text, ToolCommand* out);

}  // namespace chassis
}  // namespace autodriver

#endif  // AUTODRIVER_CHASSIS_TOOL_COMMAND_HPP_
