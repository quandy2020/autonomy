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
 * @file path_util.hpp
 * @brief Path/file wrappers that forward to autolink::common when linked.
 */

#include <string>
#include <vector>

#include "google/protobuf/message.h"

namespace autoloop {
namespace common {

/**
 * @brief Read AUTOLOOP_* or AUTONOMY_* env with default.
 * @param [in] name: Variable name.
 * @param [in] fallback: Default when unset.
 */
std::string GetEnv(const std::string& name, const std::string& fallback = "");

bool PathExists(const std::string& path);
bool EnsureDirectory(const std::string& directory_path);
bool CopyFile(const std::string& from, const std::string& to);
bool ReadFile(const std::string& path, std::string* content);
bool WriteFile(const std::string& path, const std::string& content);
bool WriteProtoBinary(const google::protobuf::Message& message,
                      const std::string& path);
bool ReadProtoBinary(const std::string& path, google::protobuf::Message* message);
std::string DirName(const std::string& path);
std::string JoinPath(const std::string& a, const std::string& b);
std::vector<std::string> ListFiles(const std::string& directory_path);

}  // namespace common
}  // namespace autoloop
