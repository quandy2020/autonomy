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

#include "autonomy/map/common/map_interface.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/map/costmap_3d/costmap_3d_wrapper.hpp"

namespace autonomy {
namespace map {
namespace common {

proto::MapOptions LoadOptions(
    autonomy::common::LuaParameterDictionary* const parameter_dictionary)
{
    proto::MapOptions options;
    options.set_use_costmap_2d(parameter_dictionary->GetBool("use_costmap_2d"));
    options.set_use_costmap_3d(parameter_dictionary->GetBool("use_costmap_3d"));
    *options.mutable_costmap2d_options() = 
        costmap_2d::CreateCostmap2DOptions(parameter_dictionary->GetDictionary("costmap2d").get());
    *options.mutable_costmap3d_options() = 
        costmap_3d::CreateCostmap3DOptions(parameter_dictionary->GetDictionary("costmap3d").get());
    return options;
}

}  // namespace common
}  // namespace map
}  // namespace autonomy