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

#include <vector>

#include "autonomy/commsgs/map_msgs.hpp" 
#include "autonomy/commsgs/sensor_msgs.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/map/costmap_2d/layer.hpp"
#include "autonomy/map/costmap_2d/layered_costmap.hpp"
#include "autonomy/map/costmap_2d/observation_buffer.hpp"
#include "autonomy/map/costmap_2d/obstacle_layer.hpp"
#include "autonomy/transform/tf2/buffer_core.h"

namespace autonomy {
namespace map {
namespace costmap_2d {

/**
 * @class VoxelLayer
 * @brief Takes laser and pointcloud data to populate a 3D voxel representation of the environment
 */
class VoxelLayer : public ObstacleLayer
{
public:
    /**
     * @brief Voxel Layer constructor
     */
    VoxelLayer()
    {
        costmap_ = NULL;  // this is the unsigned char* member of parent class's parent class Costmap2D
    }

    /**
     * @brief Voxel Layer destructor
     */
    virtual ~VoxelLayer();

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
        double robot_x, double robot_y, double robot_yaw, double * min_x,
        double * min_y,
        double * max_x,
        double * max_y);

    /**
     * @brief Update the layer's origin to a new pose, often when in a rolling costmap
     */
    void updateOrigin(double new_origin_x, double new_origin_y);

    /**
     * @brief If layer is discretely populated
     */
    bool isDiscretized()
    {
        return true;
    }

    /**
     * @brief Match the size of the master costmap
     */
    virtual void matchSize();

    /**
     * @brief Reset this costmap
     */
    virtual void reset();

    /**
     * @brief If clearing operations should be processed on this layer or not
     */
    virtual bool isClearable() {return true;}

protected:
    /**
     * @brief Reset internal maps
     */
    virtual void resetMaps();

    /**
     * @brief Use raycasting between 2 points to clear freespace
     */
    virtual void raytraceFreespace(
        const Observation& clearing_observation,
        double * min_x, double * min_y,
        double * max_x,
        double * max_y);

    bool publish_voxel_;
    double z_resolution_, origin_z_;
    int unknown_threshold_, mark_threshold_, size_z_;

    /**
     * @brief Covert world coordinates into map coordinates
     */
    inline bool worldToMap3DFloat(double wx, double wy, double wz, double& mx, double& my, double& mz)
    {
        if (wx < origin_x_ || wy < origin_y_ || wz < origin_z_) {
            return false;
        }
        mx = ((wx - origin_x_) / resolution_);
        my = ((wy - origin_y_) / resolution_);
        mz = ((wz - origin_z_) / z_resolution_);
        if (mx < size_x_ && my < size_y_ && mz < size_z_) {
            return true;
        }

        return false;
    }

    /**
     * @brief Covert world coordinates into map coordinates
     */
    inline bool worldToMap3D(double wx, double wy, double wz, unsigned int& mx, unsigned int& my, unsigned int& mz)
    {
        if (wx < origin_x_ || wy < origin_y_ || wz < origin_z_) {
            return false;
        }

        mx = static_cast<unsigned int>((wx - origin_x_) / resolution_);
        my = static_cast<unsigned int>((wy - origin_y_) / resolution_);
        mz = static_cast<unsigned int>((wz - origin_z_) / z_resolution_);

        if (mx < size_x_ && my < size_y_ && mz < (unsigned int)size_z_) {
            return true;
        }

        return false;
    }

    /**
     * @brief Covert map coordinates into world coordinates
     */
    inline void mapToWorld3D(unsigned int mx, unsigned int my, unsigned int mz, double& wx, double& wy, double& wz)
    {
        // returns the center point of the cell
        wx = origin_x_ + (mx + 0.5) * resolution_;
        wy = origin_y_ + (my + 0.5) * resolution_;
        wz = origin_z_ + (mz + 0.5) * z_resolution_;
    }

    /**
     * @brief Find L2 norm distance in 3D
     */
    inline double dist(double x0, double y0, double z0, double x1, double y1, double z1)
    {
        return sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0) + (z1 - z0) * (z1 - z0));
    }

    /**
     * @brief Get the height of the voxel sizes in meters
     */
    double getSizeInMetersZ() const
    {
        return (size_z_ - 1 + 0.5) * z_resolution_;
    }
};

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy