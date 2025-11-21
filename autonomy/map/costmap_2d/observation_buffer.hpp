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
#include <list>
#include <string>

#include "autonomy/transform/buffer.hpp"
#include "autonomy/commsgs/sensor_msgs.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/map/costmap_2d/observation.hpp"

namespace autonomy {
namespace map {
namespace costmap_2d {

/**
 * @class ObservationBuffer
 * @brief Takes in point clouds from sensors, transforms them to the desired frame, and stores them
 */
class ObservationBuffer
{
public:
    using TfBuffer = autonomy::transform::Buffer;

    /**
     * @brief  Constructs an observation buffer
     * @param  topic_name The topic of the observations, used as an identifier for error and warning messages
     * @param  observation_keep_time Defines the persistence of observations in seconds, 0 means only keep the latest
     * @param  expected_update_rate How often this buffer is expected to be updated, 0 means there is no limit
     * @param  min_obstacle_height The minimum height of a hitpoint to be considered legal
     * @param  max_obstacle_height The minimum height of a hitpoint to be considered legal
     * @param  obstacle_max_range The range to which the sensor should be trusted for inserting obstacles
     * @param  obstacle_min_range The range from which the sensor should be trusted for inserting obstacles
     * @param  raytrace_max_range The range to which the sensor should be trusted for raytracing to clear out space
     * @param  raytrace_min_range The range from which the sensor should be trusted for raytracing to clear out space
     * @param  tf2_buffer A reference to a tf2 Buffer
     * @param  global_frame The frame to transform PointClouds into
     * @param  sensor_frame The frame of the origin of the sensor, can be left blank to be read from the messages
     * @param  tf_tolerance The amount of time to wait for a transform to be available when setting a new global frame
     */
    ObservationBuffer(
        std::string topic_name,
        double observation_keep_time,
        double expected_update_rate,
        double min_obstacle_height, double max_obstacle_height, double obstacle_max_range,
        double obstacle_min_range,
        double raytrace_max_range, double raytrace_min_range, TfBuffer& tf_buffer,
        std::string global_frame,
        std::string sensor_frame,
        transform::tf2::Duration tf_tolerance);

    /**
     * @brief  Destructor... cleans up
     */
    ~ObservationBuffer();

    /**
     * @brief  Transforms a PointCloud to the global frame and buffers it
     * <b>Note: The burden is on the user to make sure the transform is available... ie they should use a MessageNotifier</b>
     * @param  cloud The cloud to be buffered
     */
    void bufferCloud(const commsgs::sensor_msgs::PointCloud2 & cloud);

    /**
     * @brief  Pushes copies of all current observations onto the end of the vector passed in
     * @param  observations The vector to be filled
     */
    void getObservations(std::vector<Observation> & observations);

    /**
     * @brief  Check if the observation buffer is being update at its expected rate
     * @return True if it is being updated at the expected rate, false otherwise
     */
    bool isCurrent() const;

    /**
     * @brief  Lock the observation buffer
     */
    inline void lock()
    {
        lock_.lock();
    }

    /**
     * @brief  Lock the observation buffer
     */
    inline void unlock()
    {
        lock_.unlock();
    }

    /**
     * @brief Reset last updated timestamp
     */
    void resetLastUpdated();

private:
    /**
     * @brief  Removes any stale observations from the buffer list
     */
    void purgeStaleObservations();

    TfBuffer& tf_buffer_;
    // const rclcpp::Duration observation_keep_time_;
    // const rclcpp::Duration expected_update_rate_;
    commsgs::builtin_interfaces::Time last_updated_;
    std::string global_frame_;
    std::string sensor_frame_;
    std::list<Observation> observation_list_;
    std::string topic_name_;
    double min_obstacle_height_, max_obstacle_height_;
    std::recursive_mutex lock_;  ///< @brief A lock for accessing data in callbacks safely
    double obstacle_max_range_, obstacle_min_range_, raytrace_max_range_, raytrace_min_range_;
    transform::tf2::Duration tf_tolerance_;
};

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy