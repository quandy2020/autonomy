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
 * @file tool_command.cpp
 * @brief ToolCommand parser (implementation).
 */

#include "chassis/tool_command.hpp"

#include <algorithm>
#include <cctype>

namespace autodriver {
namespace chassis {
namespace {

std::string Trim(std::string s) {
  auto not_space = [](unsigned char c) { return !std::isspace(c); };
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
  s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
  return s;
}

}  // namespace

bool ParseToolCommand(const std::string& text, ToolCommand* out) {
  if (out == nullptr) {
    return false;
  }
  const std::string raw = Trim(text);
  if (raw.empty()) {
    return false;
  }

  ToolCommand cmd;
  const auto sep = raw.find_first_of("=:");
  if (sep == std::string::npos) {
    cmd.name = raw;
    cmd.value = "1";
    cmd.enable = true;
  } else {
    cmd.name = Trim(raw.substr(0, sep));
    cmd.value = Trim(raw.substr(sep + 1));
    std::string lower = cmd.value;
    std::transform(
        lower.begin(), lower.end(), lower.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (lower == "0" || lower == "false" || lower == "off" ||
        lower == "disable" || lower == "disabled") {
      cmd.enable = false;
    } else {
      cmd.enable = true;
    }
  }
  if (cmd.name.empty()) {
    return false;
  }
  *out = std::move(cmd);
  return true;
}

}  // namespace chassis
}  // namespace autodriver
