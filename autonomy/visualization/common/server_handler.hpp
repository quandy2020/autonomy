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

#pragma once

#include <memory>
#include <string>

#include <foxglove/foxglove.hpp>
#include <foxglove/context.hpp>
#include <foxglove/error.hpp>
#include <foxglove/mcap.hpp>
#include <foxglove/server.hpp>

#include "autonomy/common/macros.hpp"
#include "autonomy/common/function_traits.hpp"

namespace autonomy {
namespace visualization { 

struct ServerHandler
{
    /**
     * @brief Callback invoked when a client subscribes to a channel.
     * 
     * Only invoked if the channel is associated with the server and isn't already subscribed to by  the client.
     */
    using SubscribeCallback = std::function<void(uint64_t channel_id, 
        const foxglove::ClientMetadata& client_metadata)>;

    /**
     * @brief Callback invoked when a client unsubscribes from a channel.
     * 
     * Only invoked for channels that had an active subscription from the client.
     */
    using UnsubscribeCallback = std::function<void(uint64_t channel_id, 
        const foxglove::ClientMetadata& client_metadata)>;

    /**
     * @brief Callback invoked when a client advertises a client channel.
     * 
     * Requires the capability WebSocketServerCapabilities::ClientPublish
     */
    using ClientAdvertiseCallback = std::function<void(uint32_t client_id, 
        const foxglove::ClientChannel& channel)>;

    /**
     * @brief Callback invoked when a client message is received
     */
    using MessageDataCallback = std::function<void(uint32_t client_id, 
        uint32_t client_channel_id, 
        const std::byte* data, size_t data_len)>;
    
    /**
     * @brief Callback invoked when a client unadvertises a client channel.
     * 
     * Requires the capability WebSocketServerCapabilities::ClientPublish
     */
    using ClientUnadvertiseCallback = std::function<void(uint32_t client_id, uint32_t client_channel_id)>;

    /**
     * @brief Callback invoked when a client requests parameters.
     * 
     * Requires the capability WebSocketServerCapabilities::Parameters.
     * @param client_id The client ID.
     * @param request_id A request ID unique to this client. May be NULL.
     * @param param_names A list of parameter names to fetch. If empty, this method should return all parameters.
     */
    using GetParametersCallback = std::function<std::vector<foxglove::Parameter>(
        uint32_t client_id, std::optional<std::string_view> request_id,
        const std::vector<std::string_view>& param_names)>;

   /**
    * @brief Callback invoked when a client sets parameters.
    *
    * Requires the capability WebSocketServerCapabilities::Parameters.
    *
    * This function should return the updated parameters. All clients subscribed
    * to updates for the returned parameters will be notified.
    *
    * @param client_id The client ID.
    * @param request_id A request ID unique to this client. May be NULL.
    * @param param_names A list of updated parameter values.
    */
    using SetParametersCallback = std::function<std::vector<foxglove::Parameter>(
        uint32_t client_id, std::optional<std::string_view> request_id,
        const std::vector<foxglove::ParameterView>& params)>;

    /**
     * @brief Callback invoked when a client subscribes to the named parameters
     * for the first time.
     *
     * Requires the capability WebSocketServerCapabilities::Parameters.
     *
     * @param param_names A list of parameter names.
     */
    using ParametersSubscribeCallback = std::function<void(const std::vector<std::string_view>& param_names)>;

    /**
     *  @brief Callback invoked when the last client unsubscribes from the named
     * parameters.
     *
     * Requires the capability WebSocketServerCapabilities::Parameters.
     *
     * @param param_names A list of parameter names.
     */
    using ParametersUnsubscribeCallback = std::function<void(const std::vector<std::string_view>& param_names)>;

    /**
     * @brief Callback invoked when a client requests connection graph updates.
     *
     * Requires the capability WebSocketServerCapabilities::ConnectionGraph
     */
    using ConnectionGraphSubscribeCallback = std::function<void()>;

    /**
     * @brief Callback invoked when a client unsubscribes from connection graph updates.
     *
     * Requires the capability WebSocketServerCapabilities::ConnectionGraph
     */
    using ConnectionGraphUnsubscribeCallback = std::function<void()>;

    /**
     * Define ServerHandler::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(ServerHandler)

    /**
     * @brief WebSocket server options
     */
    foxglove::WebSocketServerOptions ws_options;

    /**
     * @brief WebSocket mcap options
     */
    foxglove::McapWriterOptions mcap_options;

    /**
     * @brief foxglove server
     */
    std::unique_ptr<foxglove::WebSocketServer> server;

    /**
     * @brief foxglove mcap
     */
    std::unique_ptr<foxglove::McapWriter> writer;

    /**
     * @brief schema config
     */
    foxglove::Schema schema;

};

ServerHandler::UniquePtr InitServerOptionsHandler(
    const std::string& host = "0.0.0.0", 
    const uint32_t& port = 8765, 
    const std::string& mcap_path = "",
    bool write_mcap_data = false);


}   // namespace visualization
}   // namespace autonomy