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


#include "autonomy/visualization/foxglove_bridge.hpp"

#include "autonomy/common/logging.hpp"


namespace autonomy {
namespace visualization { 

FoxgloveBridge::FoxgloveBridge()
{
    // // This doesn't affect what gets logged to the MCAP file, this is for troubleshooting the SDK integration
    // foxglove::setLogLevel(foxglove::LogLevel::Debug);

    // foxglove::McapWriterOptions mcap_options = {};
    // mcap_options.path = "quickstart-cpp.mcap";
    // auto writerResult = foxglove::McapWriter::create(mcap_options);
    // if (!writerResult.has_value()) {
    //     LOG(ERROR) << "Failed to create writer: " << foxglove::strerror(writerResult.error());
    //     return;
    // }
    // auto writer = std::move(writerResult.value());
}

FoxgloveBridge::~FoxgloveBridge()
{

}

bool FoxgloveBridge::InitWebSocketServer()
{
    // Start a server to communicate with the Foxglove app.
    foxglove::WebSocketServerOptions ws_options;
    ws_options.host = "127.0.0.1";
    ws_options.port = 8765;
    auto server_result = foxglove::WebSocketServer::create(std::move(ws_options));
    if (!server_result.has_value()) {
        std::cerr << "Failed to create server: " << foxglove::strerror(server_result.error()) << '\n';
        return false;
    }
    auto server = std::move(server_result.value());
    std::cerr << "Server listening on port " << server.port() << '\n';
    return true;
}

bool FoxgloveBridge::InitSchema()
{
    // Create a schema for a JSON channel for logging {size: number}
    foxglove::Schema schema;
    schema.encoding = "jsonschema";
    std::string schema_data = R"({
        "type": "object",
        "properties": {
        "size": { "type": "number" }
        }
    })";
    schema.data = reinterpret_cast<const std::byte *>(schema_data.data());
    schema.data_len = schema_data.size();
    auto channel_result = foxglove::RawChannel::create("/size", "json", std::move(schema));
    if (!channel_result.has_value())
    {
        std::cerr << "Failed to create channel: " << foxglove::strerror(channel_result.error()) << '\n';
        return 1;
    }
    auto size_channel = std::move(channel_result.value());

    // Create a SceneUpdateChannel for logging changes to a 3d scene
    auto scene_channel_result = foxglove::schemas::SceneUpdateChannel::create("/scene");
    if (!scene_channel_result.has_value())
    {
        std::cerr << "Failed to create scene channel: " << foxglove::strerror(scene_channel_result.error()) << '\n';
        return false;
    }
    auto scene_channel = std::move(scene_channel_result.value());
    return true;
}

}   // namespace visualization
}   // namespace autonomy