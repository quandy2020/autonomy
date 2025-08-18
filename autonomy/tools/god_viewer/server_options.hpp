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

#include <foxglove/foxglove.hpp>
#include <foxglove/context.hpp>
#include <foxglove/error.hpp>
#include <foxglove/mcap.hpp>
#include <foxglove/server.hpp>

#include "autonomy/common/macros.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"

namespace autonomy {
namespace tools { 
namespace god_viewer {

struct ServerHander
{
    /**
     * Define ServerHander::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(ServerHander)

    /**
     * @brief WebSocket server options
     */
    foxglove::WebSocketServerOptions ws_options;

    /**
     * @brief foxglove server
     */
    std::unique_ptr<foxglove::WebSocketServer> server;
};

ServerHander::SharedPtr CreateFoxgloveViewerOptions(
    common::LuaParameterDictionary* const parameter_dictionary);

}   // namespace god_viewer
}   // namespace tools
}   // namespace autonomy