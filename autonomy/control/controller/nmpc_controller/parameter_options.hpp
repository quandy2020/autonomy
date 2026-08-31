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

/**
 * @file parameter_options.hpp
 * @brief Lua loader for NMPCControllerOptions
 */

#pragma once

#include "autonomy/common/lua_parameter_dictionary.hpp"
#include "autonomy/control/proto/nmpc_controller.pb.h"

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc_controller {

/**
 * @brief Load NMPC controller options from a Lua parameter dictionary
 * @param parameter_dictionary Lua block for nmpc_controller
 * @return NMPC options with defaults applied
 */
proto::NMPCControllerOptions LoadOptions(
    ::autonomy::common::LuaParameterDictionary* parameter_dictionary);

}  // namespace nmpc_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
