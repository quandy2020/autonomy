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

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/sensor_msgs.hpp"

namespace autonomy {
namespace map {
namespace costmap_2d {

/**
 * @brief Stores an observation in terms of a point cloud and the origin of the source
 * @note Tried to make members and constructor arguments const but the compiler would not accept the default
 * assignment operator for vector insertion!
 */
class Observation
{
public:
    /**
     * @brief  Creates an empty observation
     */
    Observation()
    : cloud_(new commsgs::sensor_msgs::PointCloud2()), obstacle_max_range_(0.0), obstacle_min_range_(0.0),
        raytrace_max_range_(0.0),
        raytrace_min_range_(0.0)
    {
    }

    /**
     * @brief A destructor
     */
    virtual ~Observation()
    {
        delete cloud_;
    }

    /**
     * @brief  Copy assignment operator
     * @param obs The observation to copy
     */
    Observation& operator=(const Observation & obs)
    {
        origin_ = obs.origin_;
        cloud_ = new commsgs::sensor_msgs::PointCloud2(*(obs.cloud_));
        obstacle_max_range_ = obs.obstacle_max_range_;
        obstacle_min_range_ = obs.obstacle_min_range_;
        raytrace_max_range_ = obs.raytrace_max_range_;
        raytrace_min_range_ = obs.raytrace_min_range_;

        return *this;
    }

    /**
     * @brief  Creates an observation from an origin point and a point cloud
     * @param origin The origin point of the observation
     * @param cloud The point cloud of the observation
     * @param obstacle_max_range The range out to which an observation should be able to insert obstacles
     * @param obstacle_min_range The range from which an observation should be able to insert obstacles
     * @param raytrace_max_range The range out to which an observation should be able to clear via raytracing
     * @param raytrace_min_range The range from which an observation should be able to clear via raytracing
     */
    Observation(
        commsgs::geometry_msgs::Point & origin, const commsgs::sensor_msgs::PointCloud2& cloud,
        double obstacle_max_range, double obstacle_min_range, double raytrace_max_range,
        double raytrace_min_range)
    : origin_(origin), cloud_(new commsgs::sensor_msgs::PointCloud2(cloud)),
        obstacle_max_range_(obstacle_max_range), obstacle_min_range_(obstacle_min_range),
        raytrace_max_range_(raytrace_max_range), raytrace_min_range_(
        raytrace_min_range)
    {
    }

    /**
     * @brief  Copy constructor
     * @param obs The observation to copy
     */
    Observation(const Observation & obs)
    : origin_(obs.origin_), cloud_(new commsgs::sensor_msgs::PointCloud2(*(obs.cloud_))),
        obstacle_max_range_(obs.obstacle_max_range_), obstacle_min_range_(obs.obstacle_min_range_),
        raytrace_max_range_(obs.raytrace_max_range_),
        raytrace_min_range_(obs.raytrace_min_range_)
    {
    }

    /**
     * @brief  Creates an observation from a point cloud
     * @param cloud The point cloud of the observation
     * @param obstacle_max_range The range out to which an observation should be able to insert obstacles
     * @param obstacle_min_range The range from which an observation should be able to insert obstacles
     */
    Observation(
        const commsgs::sensor_msgs::PointCloud2& cloud, double obstacle_max_range,
        double obstacle_min_range)
    : cloud_(new commsgs::sensor_msgs::PointCloud2(cloud)), obstacle_max_range_(obstacle_max_range),
        obstacle_min_range_(obstacle_min_range),
        raytrace_max_range_(0.0), raytrace_min_range_(0.0)
    {
    }

    commsgs::geometry_msgs::Point origin_;
    commsgs::sensor_msgs::PointCloud2* cloud_;
    double obstacle_max_range_, obstacle_min_range_, raytrace_max_range_, raytrace_min_range_;
};

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy