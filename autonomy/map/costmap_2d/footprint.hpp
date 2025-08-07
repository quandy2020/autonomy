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

#include <string>
#include <vector>
#include <utility>

#include "autonomy/commsgs/geometry_msgs.hpp"

namespace autonomy {
namespace map {
namespace costmap_2d {

/**
 * @brief Calculate the extreme distances for the footprint
 *
 * @param footprint The footprint to examine
 * @param min_dist Output parameter of the minimum distance
 * @param max_dist Output parameter of the maximum distance
 */
std::pair<double, double> calculateMinAndMaxDistances(const std::vector<commsgs::geometry_msgs::Point>& footprint);
  
/**
 * @brief Convert Point32 to Point
 */
commsgs::geometry_msgs::Point toPoint(commsgs::geometry_msgs::Point32 pt);

/**
 * @brief Convert Point to Point32
 */
commsgs::geometry_msgs::Point32 toPoint32(commsgs::geometry_msgs::Point pt);

/**
 * @brief Convert vector of Points to Polygon msg
 */
commsgs::geometry_msgs::Polygon toPolygon(std::vector<commsgs::geometry_msgs::Point> pts);

/**
 * @brief Convert Polygon msg to vector of Points.
 */
std::vector<commsgs::geometry_msgs::Point> toPointVector(std::shared_ptr<commsgs::geometry_msgs::Polygon> polygon);

/**
 * @brief  Given a pose and base footprint, build the oriented footprint of the robot (list of Points)
 * @param  x The x position of the robot
 * @param  y The y position of the robot
 * @param  theta The orientation of the robot
 * @param  footprint_spec Basic shape of the footprint
 * @param  oriented_footprint Will be filled with the points in the oriented footprint of the robot
 */
void transformFootprint(
    double x, double y, double theta,
    const std::vector<commsgs::geometry_msgs::Point>& footprint_spec, 
    std::vector<commsgs::geometry_msgs::Point>& oriented_footprint);
  
/**
 * @brief  Given a pose and base footprint, build the oriented footprint of the robot (PolygonStamped)
 * @param  x The x position of the robot
 * @param  y The y position of the robot
 * @param  theta The orientation of the robot
 * @param  footprint_spec Basic shape of the footprint
 * @param  oriented_footprint Will be filled with the points in the oriented footprint of the robot
 */
void transformFootprint(
    double x, double y, double theta,
    const std::vector<commsgs::geometry_msgs::Point>& footprint_spec,
    commsgs::geometry_msgs::PolygonStamped& oriented_footprint);

/**
 * @brief Adds the specified amount of padding to the footprint (in place)
 */
void padFootprint(std::vector<commsgs::geometry_msgs::Point>& footprint, double padding);

/**
 * @brief Create a circular footprint from a given radius
 */
std::vector<commsgs::geometry_msgs::Point> makeFootprintFromRadius(double radius);

/**
 * @brief Make the footprint from the given string.
 *
 * Format should be bracketed array of arrays of floats, like so: [[1.0, 2.2], [3.3, 4.2], ...]
 *
 */
bool makeFootprintFromString(const std::string& footprint_string, std::vector<commsgs::geometry_msgs::Point>& footprint);

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy