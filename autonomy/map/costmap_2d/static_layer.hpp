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

#include <mutex>
#include <string>
#include <vector>

#include "autonomy/transform/tf2/buffer_core.h"
#include "autonomy/commsgs/map_msgs.hpp" 
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/map/costmap_2d/costmap_layer.hpp"
#include "autonomy/map/costmap_2d/layered_costmap.hpp"
#include "autonomy/map/costmap_2d/footprint.hpp"

namespace autonomy {
namespace map {
namespace costmap_2d {

/**
 * @class StaticLayer
 * @brief Takes in a map generated from SLAM to add costs to costmap
 */
class StaticLayer : public CostmapLayer
{
public:
    /**
        * @brief Static Layer constructor
        */
    StaticLayer();
    /**
        * @brief Static Layer destructor
        */
    virtual ~StaticLayer();

    /**
     * @brief Initialization process of layer on startup
     */
    virtual void onInitialize();

    /**
     * @brief Activate this layer
     */
    virtual void activate();

    /**
     * @brief Deactivate this layer
     */
    virtual void deactivate();

    /**
     * @brief Reset this costmap
     */
    virtual void reset();

    /**
     * @brief If clearing operations should be processed on this layer or not
     */
    virtual bool isClearable() {return false;}

    /**
     * @brief Update the bounds of the master costmap by this layer's update dimensions
     * @param robot_x X pose of robot
     * @param robot_y Y pose of robot
     * @param robot_yaw Robot orientation
     * @param min_x X min map coord of the window to update
     * @param min_y Y min map coord of the window to update
     * @param max_x X max map coord of the window to update
     * @param max_y Y max map coord of the window to update
     */
    virtual void updateBounds(
        double robot_x, double robot_y, double robot_yaw, double* min_x,
        double* min_y, double* max_x, double* max_y);

    /**
     * @brief Update the costs in the master costmap in the window
     * @param master_grid The master costmap grid to update
     * @param min_x X min map coord of the window to update
     * @param min_y Y min map coord of the window to update
     * @param max_x X max map coord of the window to update
     * @param max_y Y max map coord of the window to update
     */
    virtual void updateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    /**
     * @brief Match the size of the master costmap
     */
    virtual void matchSize();

protected:
    /**
     * @brief Get parameters of layer
     */
    void getParameters();

    /**
     * @brief Process a new map coming from a topic
     */
    void processMap(const commsgs::map_msgs::OccupancyGrid& new_map);

    /**
     * @brief  Callback to update the costmap's map from the map_server
     * @param new_map The map to put into the costmap. The origin of the new
     * map along with its size will determine what parts of the costmap's
     * static map are overwritten.
     */
    void incomingMap(const commsgs::map_msgs::OccupancyGrid::SharedPtr new_map);

    // /**
    //  * @brief Callback to update the costmap's map from the map_server (or SLAM)
    //  * with an update in a particular area of the map
    //  */
    // void incomingUpdate(map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr update);

    /**
     * @brief Interpret the value in the static map given on the topic to
     * convert into costs for the costmap to utilize
     */
    unsigned char interpretValue(unsigned char value);

    std::vector<commsgs::geometry_msgs::Point> transformed_footprint_;
    bool footprint_clearing_enabled_;
    /**
     * @brief Clear costmap layer info below the robot's footprint
     */
    void updateFootprint(
        double robot_x, double robot_y, double robot_yaw, 
        double* min_x,
        double* min_y,
        double* max_x,
        double* max_y);

    std::string global_frame_;  ///< @brief The global frame for the costmap
    std::string map_frame_;  /// @brief frame that map is located in

    bool has_updated_data_{false};

    unsigned int x_{0};
    unsigned int y_{0};
    unsigned int width_{0};
    unsigned int height_{0};

    // Parameters
    std::string map_topic_;
    bool map_subscribe_transient_local_;
    bool subscribe_to_updates_;
    bool track_unknown_space_;
    bool use_maximum_;
    unsigned char lethal_threshold_;
    unsigned char unknown_cost_value_;
    bool trinary_costmap_;
    bool map_received_{false};
    bool map_received_in_update_bounds_{false};

    transform::tf2::Duration transform_tolerance_;
    commsgs::map_msgs::OccupancyGrid::SharedPtr map_buffer_;
};

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy