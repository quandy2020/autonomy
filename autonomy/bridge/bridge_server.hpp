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

#include <unordered_map>

#include "autonomy/bridge/proto/bridge_options.pb.h"
#include "autonomy/common/macros.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"
#include "autonomy/bridge/bridge_server.hpp"

namespace autonomy {
namespace bridge { 

class BridgeServer
{
public:
    /**
     * Define BridgeServer::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(BridgeServer)

    /**
     * @brief A constructor for autonomy::bridge::BridgeServer
     * @param options Additional options to control creation of the node.
     */
    explicit BridgeServer();

    /**
     * @brief A Destructor for autonomy::bridge::BridgeServer
     */
    ~BridgeServer();

    /**
     * @brief Shutdown 
     */
    void Shutdown();

private:



};

proto::BridgeOptions CreateBridgeOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary);

}   // namespace bridge
}   // namespace autonomy