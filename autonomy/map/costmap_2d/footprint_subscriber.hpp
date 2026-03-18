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
#include <vector>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/std_msgs.hpp"
#include "autonomy/map/costmap_2d/footprint.hpp"

namespace autonomy {
namespace map {
namespace costmap_2d {

/**
 * @class FootprintSubscriber
 * @brief Subscriber to the footprint topic to get current robot footprint
 * (if changing) for use in collision avoidance
 */
class FootprintSubscriber {
 public:
  /**
   * @brief A constructor
   */
  FootprintSubscriber(const std::string& topic_name, std::string robot_base_frame = "base_link",
                      double transform_tolerance = 0.1);

  /**
   * @brief A destructor
   */
  ~FootprintSubscriber() {}

  /**
   * @brief Returns the latest robot footprint, in the form as received from
   * topic (oriented).
   *
   * @param footprint Output param. Latest received footprint
   * @param footprint_header Output param. Header associated with the
   * footprint
   * @return False if no footprint has been received
   */
  bool getFootprintRaw(std::vector<commsgs::geometry_msgs::Point>& footprint,
                       commsgs::std_msgs::Header& footprint_header);

  /**
   * @brief Returns the latest robot footprint, transformed into robot base
   * frame (unoriented).
   *
   * @param footprint Output param. Latest received footprint, unoriented
   * @param footprint_header Output param. Header associated with the
   * footprint
   * @return False if no footprint has been received or if transformation
   * failed
   */
  bool getFootprintInRobotFrame(std::vector<commsgs::geometry_msgs::Point>& footprint,
                                commsgs::std_msgs::Header& footprint_header);

 protected:
  /**
   * @brief Callback to process new footprint updates.
   */
  void footprint_callback(const commsgs::geometry_msgs::PolygonStamped::SharedPtr msg);

  std::string robot_base_frame_;
  double transform_tolerance_;
  bool footprint_received_{false};
  commsgs::geometry_msgs::PolygonStamped::SharedPtr footprint_;
  //    rclcpp::Subscription<commsgs::geometry_msgs::PolygonStamped>::SharedPtr
  //    footprint_sub_;
};

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy