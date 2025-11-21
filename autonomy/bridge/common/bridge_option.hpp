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
namespace common { 

proto::GrpcOptions CreateGrpcOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary);
    
proto::MqttOptions CreateMqttOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary);
 
}   // namespace common 
}   // namespace bridge
}   // namespace autonomy
