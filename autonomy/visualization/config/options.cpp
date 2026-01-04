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

#include "autonomy/visualization/config/options.hpp"

#include "autonomy/common/lua_parameter_dictionary.hpp"
#include "autonomy/visualization/proto/visualization_options.pb.h"

namespace autonomy {
namespace visualization {

autonomy::commsgs::proto::visualization::VisualizationOptions LoadOptions(
    autonomy::common::LuaParameterDictionary* const parameter_dictionary) {
    autonomy::commsgs::proto::visualization::VisualizationOptions options;

    // 服务器配置
    if (parameter_dictionary->HasKey("host")) {
        options.set_host(parameter_dictionary->GetString("host"));
    }
    if (parameter_dictionary->HasKey("port")) {
        options.set_port(parameter_dictionary->GetInt("port"));
    }

    // 功能开关
    if (parameter_dictionary->HasKey("enable_client_publish")) {
        options.set_enable_client_publish(
            parameter_dictionary->GetBool("enable_client_publish"));
    }
    if (parameter_dictionary->HasKey("enable_connection_graph")) {
        options.set_enable_connection_graph(
            parameter_dictionary->GetBool("enable_connection_graph"));
    }

    return options;
}

}  // namespace visualization
}  // namespace autonomy
