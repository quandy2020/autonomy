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

#include "autonomy/map/costmap_2d/footprint_subscriber.hpp"

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "autonomy/map/costmap_2d/footprint.hpp"

namespace autonomy {
namespace map {
namespace costmap_2d {

FootprintSubscriber::FootprintSubscriber(const std::string& topic_name, std::string robot_base_frame,
                                         double transform_tolerance)
    : robot_base_frame_(robot_base_frame), transform_tolerance_(transform_tolerance) {
  // footprint_sub_ =
  // node->create_subscription<commsgs::geometry_msgs::PolygonStamped>(
  //   topic_name, rclcpp::SystemDefaultsQoS(),
  //   std::bind(&FootprintSubscriber::footprint_callback, this,
  //   std::placeholders::_1));
}

bool FootprintSubscriber::getFootprintRaw(std::vector<commsgs::geometry_msgs::Point>& footprint,
                                          commsgs::std_msgs::Header& footprint_header) {
  if (!footprint_received_) {
    return false;
  }

  // auto current_footprint = std::atomic_load(&footprint_);
  // footprint = toPointVector(
  //   std::make_shared<commsgs::geometry_msgs::Polygon>(current_footprint->polygon));
  // footprint_header = current_footprint->header;

  return true;
}

bool FootprintSubscriber::getFootprintInRobotFrame(std::vector<commsgs::geometry_msgs::Point>& footprint,
                                                   commsgs::std_msgs::Header& footprint_header) {
  // if (!getFootprintRaw(footprint, footprint_header)) {
  //   return false;
  // }

  // commsgs::geometry_msgs::PoseStamped current_pose;
  // if (!nav2_util::getCurrentPose(
  //     current_pose, tf_, footprint_header.frame_id, robot_base_frame_,
  //     transform_tolerance_, footprint_header.stamp))
  // {
  //   return false;
  // }

  // double x = current_pose.pose.position.x;
  // double y = current_pose.pose.position.y;
  // double theta = tf2::getYaw(current_pose.pose.orientation);

  // std::vector<commsgs::geometry_msgs::Point> temp;
  // transformFootprint(-x, -y, 0, footprint, temp);
  // transformFootprint(0, 0, -theta, temp, footprint);

  // footprint_header.frame_id = robot_base_frame_;
  // footprint_header.stamp = current_pose.header.stamp;

  return true;
}

void FootprintSubscriber::footprint_callback(const commsgs::geometry_msgs::PolygonStamped::SharedPtr msg) {
  std::atomic_store(&footprint_, msg);
  if (!footprint_received_) {
    footprint_received_ = true;
  }
}

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy