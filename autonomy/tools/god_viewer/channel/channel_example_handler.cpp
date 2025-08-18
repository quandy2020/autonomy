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

#include "autonomy/tools/god_viewer/channel/channel_example_handler.hpp"

#include <chrono>

#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace tools { 
namespace god_viewer { 
namespace channel {

LogExampleHandler::LogExampleHandler(const std::string& topic)
    : topic_{topic}
{
    channel_ = std::make_unique<foxglove::schemas::LogChannel>(
        foxglove::schemas::LogChannel::create(topic_).value()); 
}

bool LogExampleHandler::Send()
{
    const auto now = std::chrono::system_clock::now();
    const auto nanos_since_epoch = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    const auto seconds_since_epoch = nanos_since_epoch / 1000000000;
    const auto remaining_nanos = nanos_since_epoch % 1000000000;

    foxglove::schemas::Log log;
    log.level = foxglove::schemas::Log::LogLevel::INFO;
    log.message = "Hello, Foxglove!";
    log.timestamp = foxglove::schemas::Timestamp{
        static_cast<uint32_t>(seconds_since_epoch),
        static_cast<uint32_t>(remaining_nanos)};

    channel_->log(log);
    return true;
}

}   // channel
}   // god_viewer
}   // namespace tools
}   // namespace autonomy