/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#include "autonomy/map/proto/map_2d_option.pb.h"
#include "autonomy/map/proto/map_options.pb.h"

namespace autonomy {
namespace common {
class LuaParameterDictionary;
}
namespace map {

/**
 * @brief Loads MapServer options from a Lua parameter dictionary (e.g. AUTONOMY_MAP).
 * @param parameter_dictionary Lua table with map_name, map_file, map_topic,
 *        publish_frequency, and frame_id.
 * @return Populated MapOptions protobuf message.
 */
proto::MapOptions LoadOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary);

/**
 * @brief Builds Costmap2DOptions from a Lua costmap configuration table.
 * @param parameter_dictionary Lua dictionary for layers, plugins, and costmap params.
 * @return Populated Costmap2DOptions protobuf message.
 */
proto::Costmap2DOptions CreateCostmap2DOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary);

}  // namespace map
}  // namespace autonomy
