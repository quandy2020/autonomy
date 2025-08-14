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

#include <vector>
#include <string>

#include "autonomy/map/proto/map_options.pb.h"
#include "autonomy/common/macros.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"
#include "autonomy/commsgs/map_msgs.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/map/common/map_interface.hpp"

namespace autonomy {
namespace map {
namespace costmap_3d {

class Costmap3DWrapper : public common::MapInterface
{
public:
    /**
     * Define Costmap3DWrapper::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(Costmap3DWrapper)

    /**
     * @brief A constructor for nautonomy::map::costmap_2d::Costmap3DWrapper
     * @param options Additional options to control creation of the node.
     */
    Costmap3DWrapper(const proto::Costmap3DOptions& options);

    /**
     * @brief A Destructor for autonomy::map::costmap_2d::Costmap3DWrapper
     */
    ~Costmap3DWrapper();

protected:
    // options for costmap 3D
    proto::Costmap3DOptions options_;
};


proto::Costmap3DOptions CreateCostmap3DOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary);

}  // namespace costmap_3d
}  // namespace map
}  // namespace autonomy