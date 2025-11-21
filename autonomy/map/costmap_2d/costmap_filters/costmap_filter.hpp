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

#pragma once

#include <string>
#include <mutex>
#include <memory>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/map_msgs.hpp"
#include "autonomy/transform/buffer.hpp"
#include "autonomy/map/costmap_2d/layer.hpp" 

namespace autonomy {
namespace map {
namespace costmap_2d {

/**
 * @brief: CostmapFilter basic class. It is inherited from Layer in order to avoid
 * hidden problems when the shared handling of costmap_ resource (PR #1936)
 */
class CostmapFilter : public Layer
{
public:
    /**
     * @brief A constructor
     */
    CostmapFilter();
    /**
     * @brief A destructor
     */
    ~CostmapFilter();

    /**
     * @brief: Provide a typedef to ease future code maintenance
     */
    typedef std::recursive_mutex mutex_t;

    /**
     * @brief: returns pointer to a mutex
     */
    mutex_t * getMutex()
    {
        return access_;
    }

    /**
     * @brief Initialization process of layer on startup
     */
    void onInitialize() final;

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
    void updateBounds(
        double robot_x, double robot_y, double robot_yaw,
        double * min_x, double * min_y, double * max_x, double * max_y) final;

    /**
     * @brief Update the costs in the master costmap in the window
     * @param master_grid The master costmap grid to update
     * @param min_x X min map coord of the window to update
     * @param min_y Y min map coord of the window to update
     * @param max_x X max map coord of the window to update
     * @param max_y Y max map coord of the window to update
     */
    void updateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) final;

    /**
     * @brief Activate the layer
     */
    void activate();

    /**
     * @brief Deactivate the layer
     */
    void deactivate();

    /**
     * @brief Reset the layer
     */
    void reset() final;

    /**
     * @brief If clearing operations should be processed on this layer or not
     */
    bool isClearable() {return false;}

    /** CostmapFilter API **/
    /**
     * @brief: Initializes costmap filter. Creates subscriptions to filter-related topics
     * @param: Name of costmap filter info topic
     */
    virtual void initializeFilter(const std::string& filter_info_topic) = 0;

    /**
     * @brief: An algorithm for how to use that map's information. Fills the Costmap2D with
     *         calculated data and makes an action based on processed data
     * @param: Reference to a master costmap2d
     * @param: Low window map boundary OX
     * @param: Low window map boundary OY
     * @param: High window map boundary OX
     * @param: High window map boundary OY
     * @param: Robot 2D-pose
     */
    virtual void process(Costmap2D& master_grid,
        int min_i, int min_j, int max_i, int max_j,
        const commsgs::geometry_msgs::Pose2D& pose) = 0;

    /**
     * @brief: Resets costmap filter. Stops all subscriptions
     */
    virtual void resetFilter() = 0;

protected:
    // /**
    //  * @brief Costmap filter enabling/disabling callback
    //  * @param request_header Service request header
    //  * @param request Service request
    //  * @param response Service response
    //  */
    // void enableCallback(
    //     const std::shared_ptr<rmw_request_id_t> request_header,
    //     const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    //     std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    /**
     * @brief:  Transforms robot pose from current layer frame to mask frame
     * @param:  global_frame Costmap frame to transform from
     * @param:  global_pose Robot pose in costmap frame
     * @param:  mask_frame Filter mask frame to transform to
     * @param:  mask_pose Output robot pose in mask frame
     * @return: True if the transformation was successful, false otherwise
     */
    bool transformPose(
        const std::string global_frame,
        const commsgs::geometry_msgs::Pose2D& global_pose,
        const std::string mask_frame,
        commsgs::geometry_msgs::Pose2D& mask_pose) const;

    /**
     * @brief: Convert from world coordinates to mask coordinates.
         Similar to Costmap2D::worldToMap() method but works directly with OccupancyGrid-s.
    * @param  filter_mask Filter mask on which to convert
    * @param  wx The x world coordinate
    * @param  wy The y world coordinate
    * @param  mx Will be set to the associated mask x coordinate
    * @param  my Will be set to the associated mask y coordinate
    * @return True if the conversion was successful (legal bounds) false otherwise
    */
    bool worldToMask(
        commsgs::map_msgs::OccupancyGrid::ConstSharedPtr filter_mask,
        double wx, double wy, unsigned int & mx, unsigned int & my) const;

    /**
     * @brief  Get the data of a cell in the filter mask
     * @param  filter_mask Filter mask to get the data from
     * @param  mx The x coordinate of the cell
     * @param  my The y coordinate of the cell
     * @return The data of the selected cell
     */
    inline int8_t getMaskData(
        commsgs::map_msgs::OccupancyGrid::ConstSharedPtr filter_mask,
        const unsigned int mx, const unsigned int my) const
    {
        return filter_mask->data[my * filter_mask->info.width + mx];
    }

    /**
     * @brief  Get the cost of a cell in the filter mask
     * @param  filter_mask Filter mask to get the cost from
     * @param  mx The x coordinate of the cell
     * @param  my The y coordinate of the cell
     * @return The cost to set the cell to
     */
    unsigned char getMaskCost(
        commsgs::map_msgs::OccupancyGrid::ConstSharedPtr filter_mask,
        const unsigned int mx, const unsigned int & my) const;

    /**
     * @brief: Name of costmap filter info topic
     */
    std::string filter_info_topic_;

    /**
     * @brief: Name of filter mask topic
     */
    std::string mask_topic_;

    /**
     * @brief: mask_frame->global_frame_ transform tolerance
     */
    transform::tf2::Duration transform_tolerance_;

    // /**
    //  * @brief: A service to enable/disable costmap filter
    //  */
    // rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;

private:
    /**
     * @brief: Latest robot position
     */
    commsgs::geometry_msgs::Pose2D latest_pose_;

    /**
     * @brief: Mutex for locking filter's resources
     */
    mutex_t * access_;
};


}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy
