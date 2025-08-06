/*
 * Copyright 2024 The OpenRobotic Beginner Authors
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

#include "autonomy/map/proto/map_options.pb.h"
#include "autonomy/common/macros.hpp"
#include "autonomy/map/common/map_interface.hpp"

namespace autonomy {
namespace map {
namespace costmap_2d {

class Costmap2D : common::MapInterface
{
public:
    /**
     * Define Costmap2D::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(Costmap2D)

    /**
     * @brief A constructor for nautonomy::map::costmap_2d::Costmap2D
     * @param options Additional options to control creation of the node.
     */
    explicit Costmap2D();

    /**
     * @brief A Destructor for autonomy::map::costmap_2d::Costmap2D
     */
    ~Costmap2D();

private:
    proto::Costmap2DOptions options_;
};

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy