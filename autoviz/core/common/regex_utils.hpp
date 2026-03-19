/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include <algorithm>
#include <regex>
#include <string>
#include <vector>

namespace foxglove_ws {

inline bool isWhitelisted(const std::string& name, const std::vector<std::regex>& regexPatterns) {
  return std::find_if(regexPatterns.begin(), regexPatterns.end(), [name](const auto& regex) {
           return std::regex_match(name, regex);
         }) != regexPatterns.end();
}

}  // namespace foxglove_ws
