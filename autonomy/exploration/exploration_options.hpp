/*
 * Copyright 2026 The Openbot Authors
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

#include <string>

#include "autonomy/exploration/proto/exploration_options.pb.h"

namespace autonomy {
namespace common {
class LuaParameterDictionary;
}
namespace exploration {

/**
 * @file exploration_options.hpp
 * @brief Load / create ExplorationOptions from Lua configuration.
 */

/**
 * @brief Populate ExplorationOptions from a Lua parameter dictionary.
 * @param parameter_dictionary Lua dictionary (may be nullptr)
 * @return Exploration options (defaults applied when keys are missing)
 */
proto::ExplorationOptions LoadOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary);

/**
 * @brief Load exploration.lua from a configuration directory.
 * @param configuration_directory Directory containing exploration.lua
 * @return Exploration options, or defaults on failure
 */
proto::ExplorationOptions CreateOptions(
    const std::string& configuration_directory);

/**
 * @brief Built-in default ExplorationOptions (D435-class FoV defaults).
 * @return Default exploration options
 */
proto::ExplorationOptions DefaultOptions();

}  // namespace exploration
}  // namespace autonomy
