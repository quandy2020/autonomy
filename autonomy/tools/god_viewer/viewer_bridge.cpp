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

#include "autonomy/tools/god_viewer/viewer_bridge.hpp"
#include "autonomy/common/logging.hpp"

#include <thread>

namespace autonomy {
namespace tools { 
namespace god_viewer { 

ViewerBridge::ViewerBridge()
{
    thread_pool_ = std::make_shared<common::ThreadPool>(4);
    InitServer();
}  

void ViewerBridge::Run()
{
    auto channel = foxglove::schemas::LogChannel::create("/hello").value();

    while (true)
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

        channel.log(log);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

bool ViewerBridge::InitServer()
{
    // Start a server to communicate with the Foxglove app.
    foxglove::WebSocketServerOptions ws_options;
    ws_options.host = "0.0.0.0";
    ws_options.port = 8765;
    auto server_result = foxglove::WebSocketServer::create(std::move(ws_options));
    if (!server_result.has_value())
    {
        LOG(ERROR) << "Failed to create server: " << foxglove::strerror(server_result.error());
        return false;
    }
    auto server = std::move(server_result.value());
    LOG(INFO) << "Server listening on port " << server.port();
    return true;
}

}   // god_viewer
}   // namespace tools
}   // namespace autonomy