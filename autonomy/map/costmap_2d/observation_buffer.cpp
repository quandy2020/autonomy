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

#include "autonomy/map/costmap_2d/observation_buffer.hpp"

#include <algorithm>
#include <list>
#include <string>
#include <vector>
#include <chrono>

#include "autonomy/transform/tf2/convert.h"
// #include "sensor_msgs/point_cloud2_iterator.hpp"

using namespace std::chrono_literals;

namespace autonomy {
namespace map {
namespace costmap_2d {

ObservationBuffer::ObservationBuffer(
  std::string topic_name,
  double observation_keep_time,
  double expected_update_rate,
  double min_obstacle_height, double max_obstacle_height, double obstacle_max_range,
  double obstacle_min_range,
  double raytrace_max_range, double raytrace_min_range, TfBuffer& tf_buffer,
  std::string global_frame,
  std::string sensor_frame,
  transform::tf2::Duration tf_tolerance)
: tf_buffer_(tf_buffer),
//   observation_keep_time_(rclcpp::Duration::from_seconds(observation_keep_time)),
//   expected_update_rate_(rclcpp::Duration::from_seconds(expected_update_rate)),
  global_frame_(global_frame),
  sensor_frame_(sensor_frame),
  topic_name_(topic_name),
  min_obstacle_height_(min_obstacle_height), max_obstacle_height_(max_obstacle_height),
  obstacle_max_range_(obstacle_max_range), obstacle_min_range_(obstacle_min_range),
  raytrace_max_range_(raytrace_max_range), raytrace_min_range_(
    raytrace_min_range), tf_tolerance_(tf_tolerance)
{
    // last_updated_ = node->now();
}

ObservationBuffer::~ObservationBuffer()
{
}

void ObservationBuffer::bufferCloud(const commsgs::sensor_msgs::PointCloud2& cloud)
{
    commsgs::geometry_msgs::PointStamped global_origin;

    // create a new observation on the list to be populated
    observation_list_.push_front(Observation());

    // check whether the origin frame has been set explicitly
    // or whether we should get it from the cloud
    std::string origin_frame = sensor_frame_ == "" ? cloud.header.frame_id : sensor_frame_;

    try {
        // given these observations come from sensors...
        // we'll need to store the origin pt of the sensor
        commsgs::geometry_msgs::PointStamped local_origin;
        local_origin.header.stamp = cloud.header.stamp;
        local_origin.header.frame_id = origin_frame;
        local_origin.point.x = 0;
        local_origin.point.y = 0;
        local_origin.point.z = 0;
        // tf_buffer_.transform(local_origin, global_origin, global_frame_, tf_tolerance_);
        // tf2::convert(global_origin.point, observation_list_.front().origin_);

        // make sure to pass on the raytrace/obstacle range
        // of the observation buffer to the observations
        observation_list_.front().raytrace_max_range_ = raytrace_max_range_;
        observation_list_.front().raytrace_min_range_ = raytrace_min_range_;
        observation_list_.front().obstacle_max_range_ = obstacle_max_range_;
        observation_list_.front().obstacle_min_range_ = obstacle_min_range_;

        commsgs::sensor_msgs::PointCloud2 global_frame_cloud;

    //     // transform the point cloud
    //     tf2_buffer_.transform(cloud, global_frame_cloud, global_frame_, tf_tolerance_);
    //     global_frame_cloud.header.stamp = cloud.header.stamp;

        // now we need to remove observations from the cloud that are below
        // or above our height thresholds
        commsgs::sensor_msgs::PointCloud2 & observation_cloud = *(observation_list_.front().cloud_);
        observation_cloud.height = global_frame_cloud.height;
        observation_cloud.width = global_frame_cloud.width;
        observation_cloud.fields = global_frame_cloud.fields;
        observation_cloud.is_bigendian = global_frame_cloud.is_bigendian;
        observation_cloud.point_step = global_frame_cloud.point_step;
        observation_cloud.row_step = global_frame_cloud.row_step;
        observation_cloud.is_dense = global_frame_cloud.is_dense;

        // unsigned int cloud_size = global_frame_cloud.height * global_frame_cloud.width;
        // commsgs::sensor_msgs::PointCloud2Modifier modifier(observation_cloud);
        // modifier.resize(cloud_size);
        // unsigned int point_count = 0;

        // // copy over the points that are within our height bounds
        // commsgs::sensor_msgs::PointCloud2Iterator<float> iter_z(global_frame_cloud, "z");
        // std::vector<unsigned char>::const_iterator iter_global = global_frame_cloud.data.begin(),
        // iter_global_end = global_frame_cloud.data.end();
        // std::vector<unsigned char>::iterator iter_obs = observation_cloud.data.begin();
        // for (; iter_global != iter_global_end; ++iter_z, iter_global += global_frame_cloud.point_step) {
        //     if ((*iter_z) <= max_obstacle_height_ && (*iter_z) >= min_obstacle_height_) {
        //         std::copy(iter_global, iter_global + global_frame_cloud.point_step, iter_obs);
        //         iter_obs += global_frame_cloud.point_step;
        //         ++point_count;
        //     }
        // }

    //     // resize the cloud for the number of legal points
    //     modifier.resize(point_count);
    //     observation_cloud.header.stamp = cloud.header.stamp;
    //     observation_cloud.header.frame_id = global_frame_cloud.header.frame_id;
    } catch (transform::tf2::TransformException & ex) {
        // if an exception occurs, we need to remove the empty observation from the list
        observation_list_.pop_front();
        // RCLCPP_ERROR(
        //     logger_,
        //     "TF Exception that should never happen for sensor frame: %s, cloud frame: %s, %s",
        //     sensor_frame_.c_str(),
        //     cloud.header.frame_id.c_str(), ex.what());
        return;
    }

    // // if the update was successful, we want to update the last updated time
    // last_updated_ = clock_->now();

    // we'll also remove any stale observations from the list
    purgeStaleObservations();
}

// returns a copy of the observations
void ObservationBuffer::getObservations(std::vector<Observation>& observations)
{
    // first... let's make sure that we don't have any stale observations
    purgeStaleObservations();

    // now we'll just copy the observations for the caller
    std::list<Observation>::iterator obs_it;
    for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it) {
        observations.push_back(*obs_it);
    }
}

void ObservationBuffer::purgeStaleObservations()
{
    if (!observation_list_.empty()) {
        std::list<Observation>::iterator obs_it = observation_list_.begin();
        // // if we're keeping observations for no time... then we'll only keep one observation
        // if (observation_keep_time_ == rclcpp::Duration(0.0s)) {
        //     observation_list_.erase(++obs_it, observation_list_.end());
        //     return;
        // }

        // otherwise... we'll have to loop through the observations to see which ones are stale
        for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it) {
            Observation & obs = *obs_it;
            // // check if the observation is out of date... and if it is,
            // // remove it and those that follow from the list
            // if ((clock_->now() - obs.cloud_->header.stamp) > observation_keep_time_)
            // {
            //     observation_list_.erase(obs_it, observation_list_.end());
            //     return;
            // }
        }
    }
}

bool ObservationBuffer::isCurrent() const
{
    // if (expected_update_rate_ == rclcpp::Duration(0.0s)) {
    //     return true;
    // }

    // bool current = (clock_->now() - last_updated_) <=
    //     expected_update_rate_;
    // if (!current) {
    //     RCLCPP_WARN(
    //     logger_,
    //     "The %s observation buffer has not been updated for %.2f seconds, "
    //     "and it should be updated every %.2f seconds.",
    //     topic_name_.c_str(),
    //     (clock_->now() - last_updated_).seconds(),
    //     expected_update_rate_.seconds());
    // }
    // return current;
    return true;
}

void ObservationBuffer::resetLastUpdated()
{
    // last_updated_ = clock_->now();
}

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy