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

#include "autonomy/control/common/controller_interface.hpp"

namespace autonomy {
namespace control {
namespace common {

proto::ControllerOptions LoadOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary)
{
    proto::ControllerOptions options;
    return options;
}


}  // namespace common
}  // namespace control
}  // namespace autonomy

