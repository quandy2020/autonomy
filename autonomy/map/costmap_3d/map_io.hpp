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

#include <memory>
#include <vector>
#include <string>
 
#include "autonomy/common/port.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/common//ply.hpp"
#include "autonomy/commsgs/sensor_msgs.hpp"

namespace autonomy {
namespace map {
namespace costmap_3d {


/**
 * @brief Load ply_filename convert to `pcl::PointCloud<pcl::PointXYZ>` format cloud
 * 
 * @param ply_filename The ply format filename
 * @param cloud Ourput map data 
 * @return return Sccess or failed
 */
bool LoadPlyFile(const std::string& ply_filename, commsgs::sensor_msgs::PointCloud& cloud);


}  // namespace costmap_3d
}  // namespace map
}  // namespace autonomy