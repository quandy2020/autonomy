/**
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

#include <mutex>

#include "glog/logging.h"

namespace autolink {
namespace logger {

class Logger : public google::base::Logger
{
public:
    explicit Logger(google::base::Logger* wrapped);
    ~Logger();
    void Write(bool force_flush, time_t timestamp, const char* message,
               size_t message_len) override;
    void Flush() override;
    uint32_t LogSize() override;

private:
    google::base::Logger* const wrapped_;
    std::mutex mutex_;
};

}  // namespace logger
}  // namespace autolink