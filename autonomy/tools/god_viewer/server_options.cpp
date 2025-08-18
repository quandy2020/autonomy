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

#include "autonomy/tools/god_viewer/server_options.hpp"
#include "absl/strings/str_cat.h"

namespace autonomy {
namespace tools { 
namespace god_viewer {

ServerHander::SharedPtr CreateFoxgloveViewerOptions(
    common::LuaParameterDictionary* const parameter_dictionary)
{
    auto server_options = std::make_shared<ServerHander>();
    server_options->ws_options.host = parameter_dictionary->GetString("host");
    server_options->ws_options.port = parameter_dictionary->GetInt("port");
    auto server_result = foxglove::WebSocketServer::create(std::move(server_options->ws_options));
    if (!server_result.has_value())
    {
        LOG(ERROR) << "Failed to create server: " << foxglove::strerror(server_result.error());
        return nullptr;
    }
    server_options->server = std::make_unique<foxglove::WebSocketServer>(std::move(server_result.value())); 
    LOG(INFO) << absl::StrCat("Server: ", server_options->ws_options.host, ", port : ", server_options->ws_options.port);
    return server_options;
}

}   // namespace god_viewer
}   // namespace tools
}   // namespace autonomy