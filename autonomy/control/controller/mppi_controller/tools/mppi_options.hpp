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

#include "autonomy/control/proto/mppi_controller.pb.h"

namespace autonomy {
namespace common {
class LuaParameterDictionary;
}
namespace control {
namespace controller {
namespace mppi_controller {
namespace tools {

proto::MPPIControllerOptions LoadOptions(::autonomy::common::LuaParameterDictionary* const parameter_dictionary);

}  // namespace tools
}  // namespace mppi_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
