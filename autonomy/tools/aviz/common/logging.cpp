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

#include "autonomy/tools/aviz/common/logging.hpp"

#include <cstdio>
#include <mutex>
#include <string>

namespace {

static aviz::common::LoggingHandler __debug_logging_handler = [](const std::string& message,
                                                                 const std::string& file_name, size_t line_number) {
    printf("[aviz_common:debug] %s, at %s:%zu\n", message.c_str(), file_name.c_str(), line_number);
};
static aviz::common::LoggingHandler __info_logging_handler = [](const std::string& message,
                                                                const std::string& file_name, size_t line_number) {
    printf("[aviz_common:info] %s, at %s:%zu\n", message.c_str(), file_name.c_str(), line_number);
};
static aviz::common::LoggingHandler __warning_logging_handler = [](const std::string& message,
                                                                   const std::string& file_name, size_t line_number) {
    fprintf(stderr, "[aviz_common:warning] %s, at %s:%zu\n", message.c_str(), file_name.c_str(), line_number);
};
static aviz::common::LoggingHandler __error_logging_handler = [](const std::string& message,
                                                                 const std::string& file_name, size_t line_number) {
    fprintf(stderr, "[aviz_common:error] %s, at %s:%zu\n", message.c_str(), file_name.c_str(), line_number);
};

static std::mutex __logging_mutex;

}  // namespace

namespace aviz {
namespace common {

void set_logging_handlers(aviz::common::LoggingHandler debug_handler, aviz::common::LoggingHandler info_handler,
                          aviz::common::LoggingHandler warning_handler, aviz::common::LoggingHandler error_handler) {
    std::lock_guard<std::mutex> logging_lock(__logging_mutex);
    __debug_logging_handler = debug_handler;
    __info_logging_handler = info_handler;
    __warning_logging_handler = warning_handler;
    __error_logging_handler = error_handler;
}

void log_debug(const std::string& message, const std::string& file_name, size_t line_number) {
    std::lock_guard<std::mutex> logging_lock(__logging_mutex);
    __debug_logging_handler(message, file_name, line_number);
}

void log_info(const std::string& message, const std::string& file_name, size_t line_number) {
    std::lock_guard<std::mutex> logging_lock(__logging_mutex);
    __info_logging_handler(message, file_name, line_number);
}

void log_warning(const std::string& message, const std::string& file_name, size_t line_number) {
    std::lock_guard<std::mutex> logging_lock(__logging_mutex);
    __warning_logging_handler(message, file_name, line_number);
}

void log_error(const std::string& message, const std::string& file_name, size_t line_number) {
    std::lock_guard<std::mutex> logging_lock(__logging_mutex);
    __error_logging_handler(message, file_name, line_number);
}

}  // namespace common
}  // namespace aviz
