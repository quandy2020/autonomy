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

#include "autonomy/bridge/common/bridge_option.hpp"

namespace autonomy {
namespace bridge { 
namespace common { 

proto::GrpcOptions CreateGrpcOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary)
{
    proto::GrpcOptions options;
    options.set_host(parameter_dictionary->GetString("host"));
    options.set_port(parameter_dictionary->GetInt("port"));
    options.set_num_grpc_threads(parameter_dictionary->GetInt("num_grpc_threads"));
    options.set_num_event_threads(parameter_dictionary->GetInt("num_event_threads"));
    return options;
}
    
proto::MqttOptions CreateMqttOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary)
{
    proto::MqttOptions options;
    options.set_host(parameter_dictionary->GetString("host"));
    options.set_port(parameter_dictionary->GetInt("port"));
    return options;
}

}   // namespace common 
}   // namespace bridge
}   // namespace autonomy