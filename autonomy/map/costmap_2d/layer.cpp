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

#include "autonomy/map/costmap_2d/layer.hpp"

namespace autonomy {
namespace map {
namespace costmap_2d {

Layer::Layer()
: layered_costmap_(nullptr),
    name_(),
    current_(false),
    enabled_(false) {}


void Layer::initialize(LayeredCostmap* parent, std::string name)
{
    layered_costmap_ = parent;
    name_ = name;
    onInitialize();
}

const std::vector<commsgs::geometry_msgs::Point>& Layer::getFootprint() const
{
    return layered_costmap_->getFootprint();
}

std::string Layer::getFullName(const std::string& param_name)
{
    return std::string(name_ + "." + param_name);
}

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy