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
 * @file json_util.hpp
 * @brief Minimal JSON string helpers for sidecar / event lines.
 */

#include <cstdint>
#include <map>
#include <string>
#include <string_view>

namespace autoloop {
namespace common {

/**
 * @brief Escape a string for JSON.
 * @param [in] s: Raw text.
 */
std::string JsonEscape(std::string_view s);

/**
 * @brief Quote and escape a JSON string value.
 * @param [in] s: Raw text.
 */
std::string JsonString(std::string_view s);

/**
 * @brief Build a flat JSON object from string fields.
 * @param [in] fields: Key → value (values are inserted quoted).
 */
std::string JsonObject(const std::map<std::string, std::string>& fields);

/**
 * @brief Extract a quoted string field from a flat JSON object.
 * @param [in] json: Object text.
 * @param [in] key: Field name.
 * @param [out] value: Unescaped value.
 * @return True if found.
 */
bool JsonGetString(std::string_view json, std::string_view key,
                   std::string* value);

}  // namespace common
}  // namespace autoloop
