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

#include "autonomy/map/costmap_2d/costmap_filters/speed_filter.hpp"

#include "absl/strings/str_cat.h"

#include "autonomy/common/logging.hpp"
#include "autonomy/map/costmap_2d/costmap_filters/filter_values.hpp"

namespace autonomy {
namespace map {
namespace costmap_2d {

// SpeedFilter::SpeedFilter()
// : filter_info_sub_(nullptr), mask_sub_(nullptr),
//   speed_limit_pub_(nullptr), filter_mask_(nullptr), global_frame_(""),
//   speed_limit_(NO_SPEED_LIMIT), speed_limit_prev_(NO_SPEED_LIMIT)
// {
// }

void SpeedFilter::initializeFilter(const std::string& filter_info_topic)
{
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

    // rclcpp_lifecycle::LifecycleNode::SharedPtr node = node_.lock();
    // if (!node) {
    //     throw std::runtime_error{"Failed to lock node"};
    // }

    // // Declare "speed_limit_topic" parameter specific to SpeedFilter only
    // std::string speed_limit_topic;
    // declareParameter("speed_limit_topic", rclcpp::ParameterValue("speed_limit"));
    // node->get_parameter(name_ + "." + "speed_limit_topic", speed_limit_topic);

    // filter_info_topic_ = filter_info_topic;
    // // Setting new costmap filter info subscriber
    // RCLCPP_INFO(
    //     logger_,
    //     "SpeedFilter: Subscribing to \"%s\" topic for filter info...",
    //     filter_info_topic_.c_str());
    // filter_info_sub_ = node->create_subscription<nav2_msgs::msg::CostmapFilterInfo>(
    //     filter_info_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    //     std::bind(&SpeedFilter::filterInfoCallback, this, std::placeholders::_1));

    // // Get global frame required for speed limit publisher
    // global_frame_ = layered_costmap_->getGlobalFrameID();

    // // Create new speed limit publisher
    // speed_limit_pub_ = node->create_publisher<nav2_msgs::msg::SpeedLimit>(
    //     speed_limit_topic, rclcpp::QoS(10));
    // speed_limit_pub_->on_activate();

    // // Reset speed conversion states
    // base_ = BASE_DEFAULT;
    // multiplier_ = MULTIPLIER_DEFAULT;
    // percentage_ = false;
}

// void SpeedFilter::filterInfoCallback(const commmsgs::msg::CostmapFilterInfo::SharedPtr msg)
// {
//     std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

//     rclcpp_lifecycle::LifecycleNode::SharedPtr node = node_.lock();
//     if (!node) {
//         throw std::runtime_error{"Failed to lock node"};
//     }

//     if (!mask_sub_) {
//         RCLCPP_INFO(
//         logger_,
//         "SpeedFilter: Received filter info from %s topic.", filter_info_topic_.c_str());
//     } else {
//         RCLCPP_WARN(
//         logger_,
//         "SpeedFilter: New costmap filter info arrived from %s topic. Updating old filter info.",
//         filter_info_topic_.c_str());
//         // Resetting previous subscriber each time when new costmap filter information arrives
//         mask_sub_.reset();
//     }

//     // Set base_/multiplier_ or use speed limit in % of maximum speed
//     base_ = msg->base;
//     multiplier_ = msg->multiplier;
//     if (msg->type == SPEED_FILTER_PERCENT) {
//         // Using speed limit in % of maximum speed
//         percentage_ = true;
//         RCLCPP_INFO(
//         logger_,
//         "SpeedFilter: Using expressed in a percent from maximum speed"
//         "speed_limit = %f + filter_mask_data * %f",
//         base_, multiplier_);
//     } else if (msg->type == SPEED_FILTER_ABSOLUTE) {
//         // Using speed limit in m/s
//         percentage_ = false;
//         RCLCPP_INFO(
//         logger_,
//         "SpeedFilter: Using absolute speed_limit = %f + filter_mask_data * %f",
//         base_, multiplier_);
//     } else {
//         RCLCPP_ERROR(logger_, "SpeedFilter: Mode is not supported");
//         return;
//     }

//     mask_topic_ = msg->filter_mask_topic;

//     // Setting new filter mask subscriber
//     RCLCPP_INFO(
//         logger_,
//         "SpeedFilter: Subscribing to \"%s\" topic for filter mask...",
//         mask_topic_.c_str());
//     mask_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
//         mask_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
//         std::bind(&SpeedFilter::maskCallback, this, std::placeholders::_1));
// }

void SpeedFilter::maskCallback(const commsgs::map_msgs::OccupancyGrid::SharedPtr msg)
{
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

    if (!filter_mask_) {
        LOG(INFO) << absl::StrCat("SpeedFilter: Received filter mask from ", mask_topic_, " topic.");
    } else {
        LOG(WARNING) << absl::StrCat("SpeedFilter: New filter mask arrived from ", mask_topic_, "topic. Updating old filter mask.");
        filter_mask_.reset();
    }
    filter_mask_ = msg;
}

void SpeedFilter::process(Costmap2D& /*master_grid*/,
  int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/,
  const commsgs::geometry_msgs::Pose2D & pose)
{
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

    if (!filter_mask_) {
        // Show warning message every 2 seconds to not litter an output
        LOG(WARNING) << "SpeedFilter: Filter mask was not received";
        return;
    }

    commsgs::geometry_msgs::Pose2D mask_pose;  // robot coordinates in mask frame

    // Transforming robot pose from current layer frame to mask frame
    if (!transformPose(global_frame_, pose, filter_mask_->header.frame_id, mask_pose)) {
        return;
    }

    // Converting mask_pose robot position to filter_mask_ indexes (mask_robot_i, mask_robot_j)
    unsigned int mask_robot_i, mask_robot_j;
    if (!worldToMask(filter_mask_, mask_pose.x, mask_pose.y, mask_robot_i, mask_robot_j)) {
        return;
    }

    // Getting filter_mask data from cell where the robot placed and
    // calculating speed limit value
    int8_t speed_mask_data = getMaskData(filter_mask_, mask_robot_i, mask_robot_j);
    if (speed_mask_data == SPEED_MASK_NO_LIMIT) {
        // Corresponding filter mask cell is free.
        // Setting no speed limit there.
        speed_limit_ = NO_SPEED_LIMIT;
    } else if (speed_mask_data == SPEED_MASK_UNKNOWN) {
        // Corresponding filter mask cell is unknown.
        // Do nothing.
        LOG(ERROR) << absl::StrCat("SpeedFilter: Found unknown cell in filter_mask[", 
            mask_robot_i, mask_robot_j, "] which is invalid for this kind of filter");
        return;
    } else {
        // Normal case: speed_mask_data in range of [1..100]
        speed_limit_ = speed_mask_data * multiplier_ + base_;
        if (percentage_) {
            if (speed_limit_ < 0.0 || speed_limit_ > 100.0) {
                // RCLCPP_WARN(logger_,
                //     "SpeedFilter: Speed limit in filter_mask[%i, %i] is %f%%, "
                //     "out of bounds of [0, 100]. Setting it to no-limit value.",
                //     mask_robot_i, mask_robot_j, speed_limit_);
                speed_limit_ = NO_SPEED_LIMIT;
            }
        } else {
            if (speed_limit_ < 0.0) {
                // RCLCPP_WARN(logger_,
                // "SpeedFilter: Speed limit in filter_mask[%i, %i] is less than 0 m/s, "
                // "which can not be true. Setting it to no-limit value.",
                // mask_robot_i, mask_robot_j);
                speed_limit_ = NO_SPEED_LIMIT;
            }
        }
    }

    if (speed_limit_ != speed_limit_prev_) {
        if (speed_limit_ != NO_SPEED_LIMIT) {
            DLOG(INFO) << "SpeedFilter: Speed limit is set to " << speed_limit_;
        } else {
            DLOG(INFO) << "SpeedFilter: Speed limit is set to its default value";
        }

        // // Forming and publishing new SpeedLimit message
        // std::unique_ptr<nav2_msgs::msg::SpeedLimit> msg =
        // std::make_unique<nav2_msgs::msg::SpeedLimit>();
        // msg->header.frame_id = global_frame_;
        // msg->header.stamp = clock_->now();
        // msg->percentage = percentage_;
        // msg->speed_limit = speed_limit_;
        // speed_limit_pub_->publish(std::move(msg));

        speed_limit_prev_ = speed_limit_;
    }
}

void SpeedFilter::resetFilter()
{
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

    // filter_info_sub_.reset();
    // mask_sub_.reset();
    // if (speed_limit_pub_) {
    //     speed_limit_pub_->on_deactivate();
    //     speed_limit_pub_.reset();
    // }
}

bool SpeedFilter::isActive()
{
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

    if (filter_mask_) {
        return true;
    }
    return false;
}

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy
