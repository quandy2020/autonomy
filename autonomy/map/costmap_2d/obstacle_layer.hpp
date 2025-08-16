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
#include <string>
#include <vector>

#include "autonomy/commsgs/sensor_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/commsgs/map_msgs.hpp"
#include "autonomy/map/costmap_2d/costmap_layer.hpp"
#include "autonomy/map/costmap_2d/layered_costmap.hpp"
#include "autonomy/map/costmap_2d/observation_buffer.hpp"
#include "autonomy/map/costmap_2d/footprint.hpp"

namespace autonomy {
namespace map {
namespace costmap_2d {

/**
 * @class ObstacleLayer
 * @brief Takes in laser and pointcloud data to populate into 2D costmap
 */
class ObstacleLayer : public CostmapLayer
{
public:
    /**
     * @brief A constructor
     */
    ObstacleLayer()
    {
        costmap_ = NULL;  // this is the unsigned char* member of parent class Costmap2D.
    }

    /**
     * @brief A destructor
     */
    virtual ~ObstacleLayer();

    /**
     * @brief Initialization process of layer on startup
     */
    virtual void onInitialize();

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
        double robot_x, double robot_y, double robot_yaw, 
        double* min_x,
        double* min_y,
        double* max_x,
        double* max_y);

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
     * @brief Deactivate the layer
     */
    virtual void deactivate();

    /**
     * @brief Activate the layer
     */
    virtual void activate();

    /**
     * @brief Reset this costmap
     */
    virtual void reset();

    /**
     * @brief If clearing operations should be processed on this layer or not
     */
    virtual bool isClearable() {return true;}

    /**
     * @brief triggers the update of observations buffer
     */
    void resetBuffersLastUpdated();

    /**
     * @brief  A callback to handle buffering LaserScan messages
     * @param message The message returned from a message notifier
     * @param buffer A pointer to the observation buffer to update
     */
    void laserScanCallback(
        commsgs::sensor_msgs::LaserScan::ConstSharedPtr message,
        const std::shared_ptr<ObservationBuffer>& buffer);

    /**
     * @brief A callback to handle buffering LaserScan messages which need filtering to turn Inf values into range_max.
     * @param message The message returned from a message notifier
     * @param buffer A pointer to the observation buffer to update
     */
    void laserScanValidInfCallback(
        commsgs::sensor_msgs::LaserScan::ConstSharedPtr message,
        const std::shared_ptr<ObservationBuffer>& buffer);

    /**
     * @brief  A callback to handle buffering PointCloud2 messages
     * @param message The message returned from a message notifier
     * @param buffer A pointer to the observation buffer to update
     */
    void pointCloud2Callback(
        commsgs::sensor_msgs::PointCloud2::ConstSharedPtr message,
        const std::shared_ptr<ObservationBuffer>& buffer);

    // for testing purposes
    void addStaticObservation(Observation& obs, bool marking, bool clearing);
    void clearStaticObservations(bool marking, bool clearing);

protected:
    /**
     * @brief  Get the observations used to mark space
     * @param marking_observations A reference to a vector that will be populated with the observations
     * @return True if all the observation buffers are current, false otherwise
     */
    bool getMarkingObservations(std::vector<Observation>& marking_observations) const;

    /**
     * @brief  Get the observations used to clear space
     * @param clearing_observations A reference to a vector that will be populated with the observations
     * @return True if all the observation buffers are current, false otherwise
     */
    bool getClearingObservations(std::vector<Observation>& clearing_observations) const;

    /**
     * @brief  Clear freespace based on one observation
     * @param clearing_observation The observation used to raytrace
     * @param min_x
     * @param min_y
     * @param max_x
     * @param max_y
     */
    virtual void raytraceFreespace(
        const Observation& clearing_observation,
        double* min_x, 
        double* min_y,
        double* max_x,
        double* max_y);

    /**
     * @brief Process update costmap with raytracing the window bounds
     */
    void updateRaytraceBounds(
        double ox, double oy, double wx, double wy, double max_range, double min_range,
        double* min_x, 
        double* min_y,
        double* max_x,
        double* max_y);

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
    double min_obstacle_height_;  ///< @brief Max Obstacle Height
    double max_obstacle_height_;  ///< @brief Max Obstacle Height

    /// @brief Used to project laser scans into point clouds
    // laser_geometry::LaserProjection projector_;
  
    /// @brief Used to store observations from various sensors
    std::vector<std::shared_ptr<ObservationBuffer>> observation_buffers_;
    /// @brief Used to store observation buffers used for marking obstacles
    std::vector<std::shared_ptr<ObservationBuffer>> marking_buffers_;
    /// @brief Used to store observation buffers used for clearing obstacles
    std::vector<std::shared_ptr<ObservationBuffer>> clearing_buffers_;

    // Used only for testing purposes
    std::vector<Observation> static_clearing_observations_;
    std::vector<Observation> static_marking_observations_;

    bool rolling_window_;
    bool was_reset_;
    // CombinationMethod combination_method_;
};

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy