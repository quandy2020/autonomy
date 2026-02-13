/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <functional>
#include <memory>
#include <sstream>
#include <string>

/// Redirectable logging macros and functions.
/**
 * \file logging.hpp
 * This file contains logging macros which are used within this library.
 * The macros wrap some generic logging functions, which can be redirected
 * using custom logging handlers via the set_logging_handlers() function.
 * By default logging goes to stdout or stderr based on the severity.
 *
 * There are four logging levels: debug, info, warning, and error.
 * For each there is a simple macro which takes a single string, and a more
 * complex macro for taking stream arguments in the style of sstring.
 *
 * For example:
 *
 *   AVIZ_COMMON_LOG_INFO("hello world")
 *   AVIZ_COMMON_LOG_WARNING_STREAM("hello " << "world: " << 42)
 */

#define AVIZ_COMMON_LOG_DEBUG(msg)                        \
    do {                                                  \
        aviz::common::log_debug(msg, __FILE__, __LINE__); \
    } while (0)

#define AVIZ_COMMON_LOG_DEBUG_STREAM(args)                       \
    do {                                                         \
        std::stringstream __ss;                                  \
        __ss << args;                                            \
        aviz::common::log_debug(__ss.str(), __FILE__, __LINE__); \
    } while (0)

#define AVIZ_COMMON_LOG_INFO(msg)                        \
    do {                                                 \
        aviz::common::log_info(msg, __FILE__, __LINE__); \
    } while (0)

#define AVIZ_COMMON_LOG_INFO_STREAM(args)                       \
    do {                                                        \
        std::stringstream __ss;                                 \
        __ss << args;                                           \
        aviz::common::log_info(__ss.str(), __FILE__, __LINE__); \
    } while (0)

#define AVIZ_COMMON_LOG_WARNING(msg)                        \
    do {                                                    \
        aviz::common::log_warning(msg, __FILE__, __LINE__); \
    } while (0)

#define AVIZ_COMMON_LOG_WARNING_STREAM(args)                       \
    do {                                                           \
        std::stringstream __ss;                                    \
        __ss << args;                                              \
        aviz::common::log_warning(__ss.str(), __FILE__, __LINE__); \
    } while (0)

#define AVIZ_COMMON_LOG_ERROR(msg)                        \
    do {                                                  \
        aviz::common::log_error(msg, __FILE__, __LINE__); \
    } while (0)

#define AVIZ_COMMON_LOG_ERROR_STREAM(args)                       \
    do {                                                         \
        std::stringstream __ss;                                  \
        __ss << args;                                            \
        aviz::common::log_error(__ss.str(), __FILE__, __LINE__); \
    } while (0)

// Simplified aliases for convenience
#define AVIZ_DEBUG(msg) AVIZ_COMMON_LOG_DEBUG(msg)
#define AVIZ_INFO(msg) AVIZ_COMMON_LOG_INFO(msg)
#define AVIZ_WARN(msg) AVIZ_COMMON_LOG_WARNING(msg)
#define AVIZ_ERROR(msg) AVIZ_COMMON_LOG_ERROR(msg)

namespace aviz {
namespace common {

using LoggingHandler =
    std::function<void(const std::string& message, const std::string& file_name, size_t line_number)>;

/// Set the given logging handlers globally.
/**
 * All log traffic is routed through these logging functions.
 */
void set_logging_handlers(aviz::common::LoggingHandler debug_handler, aviz::common::LoggingHandler info_handler,
                          aviz::common::LoggingHandler warning_handler, aviz::common::LoggingHandler error_handler);

void log_debug(const std::string& message, const std::string& file_name, size_t line_number);

void log_info(const std::string& message, const std::string& file_name, size_t line_number);

void log_warning(const std::string& message, const std::string& file_name, size_t line_number);

void log_error(const std::string& message, const std::string& file_name, size_t line_number);

}  // namespace common
}  // namespace aviz
