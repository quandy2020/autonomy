/*
 * Copyright 2024 The OpenRobotic Beginner Authors
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
 * @file
 * @brief Some string util functions.
 */

#pragma once

#include <string>

#include "absl/strings/str_format.h"
#include "autonomy/common/future.hpp"

#define FORMAT_TIMESTAMP(timestamp) \
  std::fixed << std::setprecision(9) << timestamp

/**
 * @namespace autonomy::common
 * @brief autonomy::common
 */
namespace autonomy {
namespace common {

using absl::StrFormat;

struct DebugStringFormatter {
  template <class T>
  void operator()(std::string* out, const T& t) const {
    out->append(t.DebugString());
  }
};

std::string EncodeBase64(std::string_view in);

// Format string by replacing embedded format specifiers with their respective
// values, see `printf` for more details. This is a modified implementation
// of Google's BSD-licensed StringPrintf function.
std::string StringPrintf(const char* format, ...);

// Replace all occurrences of `old_str` with `new_str` in the given string.
std::string StringReplace(const std::string& str,
                          const std::string& old_str,
                          const std::string& new_str);

// Get substring of string after search key
std::string StringGetAfter(const std::string& str, const std::string& key);

// Split string into list of words using the given delimiters.
std::vector<std::string> StringSplit(const std::string& str,
                                     const std::string& delim);

// Check whether a string starts with a certain prefix.
bool StringStartsWith(const std::string& str, const std::string& prefix);

// Remove whitespace from string on both, left, or right sides.
void StringTrim(std::string* str);
void StringLeftTrim(std::string* str);
void StringRightTrim(std::string* str);

// Convert string to lower/upper case.
void StringToLower(std::string* str);
void StringToUpper(std::string* str);

// Check whether the sub-string is contained in the given string.
bool StringContains(const std::string& str, const std::string& sub_str);

}  // namespace common
}  // namespace autonomy
