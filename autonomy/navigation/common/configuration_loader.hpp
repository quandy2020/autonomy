/*
 * Copyright 2026 The Navigation Authors (navigation)
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
 * @file configuration_loader.hpp
 * @brief Protobuf text-format configuration loading with strict validation.
 */

#pragma once

#include <string>

#include "google/protobuf/message.h"
#include "navigation/common/status.hpp"

namespace navigation
{
namespace common
{

/**
 * @brief Load a protobuf text-format file into a message.
 *
 * @param[in] path Absolute path or a path relative to NAVIGATION_CONF_DIR.
 * @param[out] message Parsed message; unchanged on failure.
 * @return kOk on success, kUnavailable for I/O, kConfigurationError for parse.
 */
Status LoadTextProto(const std::string& path, google::protobuf::Message* message);

/**
 * @brief Parse protobuf text into a message.
 *
 * @param[in] text Text-format payload.
 * @param[out] message Parsed message; cleared on failure.
 * @return kOk or kConfigurationError; unknown fields are rejected.
 */
Status ParseTextProto(const std::string& text, google::protobuf::Message* message);

/**
 * @brief Resolve a configuration path.
 *
 * @param[in] path Absolute or relative path.
 * @return Absolute path unchanged, otherwise NAVIGATION_CONF_DIR/path.
 */
std::string ResolveConfigPath(const std::string& path);

}  // namespace common
}  // namespace navigation
