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

#include "absl/strings/str_cat.h"
#include "autonomy/common/logging.hpp"
#include "autonomy/visualization/common/server_handler.hpp"

namespace autonomy {
namespace visualization { 

ServerHandler::UniquePtr InitServerOptionsHandler(
    const std::string& host, 
    const uint32_t& port, 
    const std::string& mcap_name, 
    bool write_mcap_data)
{
    auto server_handler = std::make_unique<ServerHandler>();
    foxglove::setLogLevel(foxglove::LogLevel::Info);

    // mcap filename
    if (write_mcap_data && !mcap_name.empty()) {
        server_handler->mcap_options.path = mcap_name ;
        auto writer_result = foxglove::McapWriter::create(server_handler->mcap_options);
        if (!writer_result.has_value()) {
            LOG(ERROR) << "Failed to create writer: " << foxglove::strerror(writer_result.error());
            return nullptr;
        }
        server_handler->writer = std::make_unique<foxglove::McapWriter>(std::move(writer_result.value()));
    }

    // server
    server_handler->ws_options.name = "foxglove-bridge";
    server_handler->ws_options.host = host;
    server_handler->ws_options.port = port;
    server_handler->ws_options.capabilities = foxglove::WebSocketServerCapabilities::ClientPublish;
    server_handler->ws_options.supported_encodings = {"json"};

    // callbacks
    server_handler->ws_options.callbacks.onMessageData =
        [](uint32_t client_id, uint32_t client_channel_id, const std::byte* data, size_t data_len) {
     
    };

    auto server_result = foxglove::WebSocketServer::create(std::move(server_handler->ws_options));
    if (!server_result.has_value()) {
        LOG(ERROR) << "Failed to create server: " << foxglove::strerror(server_result.error());
        return nullptr;
    }
    server_handler->server = std::make_unique<foxglove::WebSocketServer>(std::move(server_result.value())); 
    LOG(INFO) << absl::StrCat("Server: ", server_handler->ws_options.host, ", port : ", server_handler->ws_options.port);

    // Create a schema for a JSON channel for logging {size: number}
    server_handler->schema.encoding = "jsonschema";
    std::string schema_data = R"({
        "type": "object",
        "properties": {
        "size": { "type": "number" }
        }
    })";
    server_handler->schema.data = reinterpret_cast<const std::byte *>(schema_data.data());
    server_handler->schema.data_len = schema_data.size();
    return server_handler;
}

}   // namespace visualization
}   // namespace autonomy