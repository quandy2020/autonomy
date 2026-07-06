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

#include "autonomy/bridge/common/bridge_interface.hpp"

#include "autonomy/bridge/common/bridge_option.hpp"
#include "autonomy/common/configuration_file_resolver.hpp"
namespace autonomy {
namespace bridge {
namespace common {

proto::BridgeOptions LoadOptions(
    autonomy::common::LuaParameterDictionary* const parameter_dictionary) {
    proto::BridgeOptions options;
    options.set_use_grpc(parameter_dictionary->GetBool("use_grpc"));
    options.set_use_mqtt(parameter_dictionary->GetBool("use_mqtt"));
    if (parameter_dictionary->HasKey("grpc")) {
        *options.mutable_grpc() = CreateGrpcOptions(
            parameter_dictionary->GetDictionary("grpc").get());
    }
    if (parameter_dictionary->HasKey("mqtt")) {
        *options.mutable_mqtt() = CreateMqttOptions(
            parameter_dictionary->GetDictionary("mqtt").get());
    }
    return options;
}

proto::BridgeOptions CreateOptions(
    const std::string& configuration_directory,
    const std::string& configuration_basename) {
    auto file_resolver =
        std::make_unique<::autonomy::common::ConfigurationFileResolver>(
            std::vector<std::string>{configuration_directory});
    const std::string code =
        ::autonomy::common::GetLuaScriptWithCommonOrDie(*file_resolver,
                                                        configuration_basename);
    ::autonomy::common::LuaParameterDictionary lua_parameter_dictionary(
        code, std::move(file_resolver));
    return LoadOptions(&lua_parameter_dictionary);
}

}  // namespace common
}  // namespace bridge
}  // namespace autonomy