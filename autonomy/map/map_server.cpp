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

#include "autonomy/common/logging.hpp"
#include "autonomy/map/constants.hpp"
#include "autonomy/map/map_server.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/map/costmap_3d/costmap_3d_wrapper.hpp"

namespace autonomy {
namespace map {

MapServer::MapServer(const proto::MapOptions& options)
    : options_{options}
{

    // // Create costmap 2D or 3D.
    // if (options_.use_costmap_2d()) {
    //     costmap_ = std::make_shared<costmap_2d::Costmap2DWrapper>(node_, options_.costmap2d_options());
    //     LOG(INFO) << "Use costmap 2D map.";
    // } else if (options_.use_costmap_3d()) {
    //     costmap_ = std::make_shared<costmap_3d::Costmap3DWrapper>(options_.costmap3d_options());
    //     LOG(INFO) << "Use costmap 3D map.";
    // }

    costmap_ = std::make_shared<costmap_2d::Costmap2DWrapper>(options_.costmap2d_options());

    LOG(INFO) << "Map server init successfully.";
}


void MapServer::Start()
{
    LOG(INFO) << "Start map server.";
    costmap_->Start();
}

void MapServer::WaitForShutdown()
{

}


}  // namespace map
}  // namespace autonomy


    