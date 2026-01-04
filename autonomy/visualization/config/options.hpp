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

#ifndef AUTONOMY_VISUALIZATION_CONFIG_OPTIONS_HPP_
#define AUTONOMY_VISUALIZATION_CONFIG_OPTIONS_HPP_

#include <string>

#include "autonomy/visualization/proto/visualization_options.pb.h"

namespace autonomy {
namespace common {
class LuaParameterDictionary;
}  // namespace common
}  // namespace autonomy

namespace autonomy {
namespace visualization {

/// 从 Lua 配置字典加载 VisualizationOptions
/// @param parameter_dictionary Lua 参数字典
/// @return VisualizationOptions protobuf 消息
autonomy::commsgs::proto::visualization::VisualizationOptions LoadOptions(
    autonomy::common::LuaParameterDictionary* const parameter_dictionary);

}  // namespace visualization
}  // namespace autonomy

#endif  // AUTONOMY_VISUALIZATION_CONFIG_OPTIONS_HPP_
