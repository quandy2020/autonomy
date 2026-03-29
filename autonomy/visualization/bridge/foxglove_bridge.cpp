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

#include "autonomy/visualization/bridge/foxglove_bridge.hpp"

#include <unistd.h>
#include <optional>
#include <string>
#include <string_view>

#include "autonomy/visualization/transport/auto_discovery.hpp"
#include "foxglove/server.hpp"

namespace autonomy {
namespace visualization {

FoxgloveBridge::FoxgloveBridge() : options_() {}

FoxgloveBridge::FoxgloveBridge(const Options& options) : options_(options) {
    // TODO: Set log level if API is available
    // ::foxglove::setLogLevel(foxglove::LogLevel::Warn);
}

FoxgloveBridge::~FoxgloveBridge() {
    Stop();
}

bool FoxgloveBridge::Start() {
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (is_running_) {
        AWARN << "FoxgloveBridge server is already running";
        return true;
    }

    ws_options_.name = "Autonomy Visualization Server";
    ws_options_.host = options_.host;
    ws_options_.port = options_.port;
    ws_options_.supported_encodings = {"protobuf", "json", "ros1", "cdr"};

    // Setup capabilities
    ::foxglove::WebSocketServerCapabilities capabilities =
        ::foxglove::WebSocketServerCapabilities(0);
    if (options_.enable_client_publish) {
        capabilities = capabilities |
                       ::foxglove::WebSocketServerCapabilities::ClientPublish;
    }
    if (options_.enable_connection_graph) {
        capabilities = capabilities |
                       ::foxglove::WebSocketServerCapabilities::ConnectionGraph;
    }
    if (options_.enable_parameters) {
        capabilities =
            capabilities | ::foxglove::WebSocketServerCapabilities::Parameters;
    }
    if (options_.enable_time) {
        capabilities =
            capabilities | ::foxglove::WebSocketServerCapabilities::Time;
    }
    if (options_.enable_services) {
        capabilities =
            capabilities | ::foxglove::WebSocketServerCapabilities::Services;
    }
    ws_options_.capabilities = capabilities;

    // Setup callbacks
    SetupCallbacks(ws_options_.callbacks);

    // Create server (retry ports if needed; common failure is "address already
    // in use")
    std::optional<::foxglove::FoxgloveError> last_error;
    std::optional<::foxglove::WebSocketServer> created_server;
    const uint16_t base_port = options_.port;
    constexpr int kMaxPortTries = 20;

    for (int i = 0; i < kMaxPortTries; ++i) {
        // create() takes an rvalue; keep ws_options_ intact and move a copy
        // each try
        auto opts = ws_options_;
        opts.port = static_cast<uint16_t>(base_port + i);
        auto server_result =
            ::foxglove::WebSocketServer::create(std::move(opts));
        if (server_result.has_value()) {
            created_server.emplace(std::move(server_result.value()));
            if (i > 0) {
                AWARN << "Visualization port " << base_port
                      << " unavailable, using " << (base_port + i);
            }
            break;
        }
        last_error = server_result.error();
    }

    if (!created_server.has_value()) {
        AERROR << "Failed to create Foxglove WebSocket server (last error): "
               << (last_error ? static_cast<int>(*last_error) : -1);
        return false;
    }

    // Start AutoDiscovery
    discovery_ = std::make_unique<transport::AutoDiscovery>(
        std::static_pointer_cast<FoxgloveBridge>(shared_from_this()));
    if (discovery_) {
        discovery_->Start();
        AINFO << "Starting AutoDiscovery";
    }

    // Start server
    server_ = std::make_unique<::foxglove::WebSocketServer>(
        std::move(*created_server));
    is_running_ = true;

    AINFO << "Foxglove WebSocket server started successfully on "
          << options_.host << ":" << server_->port();

    return true;
}

void FoxgloveBridge::Stop() {
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (!is_running_) {
        return;
    }

    // Stop AutoDiscovery
    if (discovery_) {
        discovery_->Stop();
        discovery_.reset();
    }

    // Stop server
    if (server_) {
        auto error = server_->stop();
        if (error != ::foxglove::FoxgloveError::Ok) {
            AERROR << "Error stopping Foxglove WebSocket server: "
                   << static_cast<int>(error);
        }
        server_.reset();
    }

    is_running_ = false;
}

bool FoxgloveBridge::IsRunning() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return is_running_;
}

uint16_t FoxgloveBridge::GetPort() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (server_ && is_running_) {
        return server_->port();
    }
    return 0;
}

void FoxgloveBridge::SetupCallbacks(
    ::foxglove::WebSocketServerCallbacks& callbacks) {
    callbacks.onSubscribe = [](uint64_t, const ::foxglove::ClientMetadata&) {};
    callbacks.onUnsubscribe = [](uint64_t, const ::foxglove::ClientMetadata&) {
    };
}

}  // namespace visualization
}  // namespace autonomy
