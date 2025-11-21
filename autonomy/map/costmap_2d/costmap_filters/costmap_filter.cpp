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

#include "autonomy/map/costmap_2d/costmap_filters/costmap_filter.hpp"

#include <exception>

#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/utils/occ_grid_values.hpp"

namespace autonomy {
namespace map {
namespace costmap_2d {

CostmapFilter::CostmapFilter()
    : filter_info_topic_(""), mask_topic_("")
{
    access_ = new mutex_t();
}

CostmapFilter::~CostmapFilter()
{
    delete access_;
}

void CostmapFilter::onInitialize()
{
//   rclcpp_lifecycle::LifecycleNode::SharedPtr node = node_.lock();
//   if (!node) {
//     throw std::runtime_error{"Failed to lock node"};
//   }

//   try {
//     // Declare common for all costmap filters parameters
//     declareParameter("enabled", rclcpp::ParameterValue(true));
//     declareParameter("filter_info_topic", rclcpp::PARAMETER_STRING);
//     declareParameter("transform_tolerance", rclcpp::ParameterValue(0.1));

//     // Get parameters
//     node->get_parameter(name_ + "." + "enabled", enabled_);
//     filter_info_topic_ = node->get_parameter(name_ + "." + "filter_info_topic").as_string();
//     double transform_tolerance {};
//     node->get_parameter(name_ + "." + "transform_tolerance", transform_tolerance);
//     transform_tolerance_ = tf2::durationFromSec(transform_tolerance);

//     // Costmap Filter enabling service
//     enable_service_ = node->create_service<std_srvs::srv::SetBool>(
//       name_ + "/toggle_filter",
//       std::bind(
//         &CostmapFilter::enableCallback, this,
//         std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
//   } catch (const std::exception & ex) {
//     RCLCPP_ERROR(logger_, "Parameter problem: %s", ex.what());
//     throw ex;
//   }
}

void CostmapFilter::activate()
{
    initializeFilter(filter_info_topic_);
}

void CostmapFilter::deactivate()
{
    resetFilter();
}

void CostmapFilter::reset()
{
    resetFilter();
    initializeFilter(filter_info_topic_);
    current_ = false;
}

void CostmapFilter::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * /*min_x*/, double * /*min_y*/, double * /*max_x*/, double * /*max_y*/)
{
    if (!enabled_) {
        return;
    }

    latest_pose_.x = robot_x;
    latest_pose_.y = robot_y;
    latest_pose_.theta = robot_yaw;
}

void CostmapFilter::updateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
    if (!enabled_) {
        return;
    }

    process(master_grid, min_i, min_j, max_i, max_j, latest_pose_);
    current_ = true;
}

// void CostmapFilter::enableCallback(
//   const std::shared_ptr<rmw_request_id_t>/*request_header*/,
//   const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
//   std::shared_ptr<std_srvs::srv::SetBool::Response> response)
// {
//   enabled_ = request->data;
//   response->success = true;
//   if (enabled_) {
//     response->message = "Enabled";
//   } else {
//     response->message = "Disabled";
//   }
// }

bool CostmapFilter::transformPose(
  const std::string global_frame,
  const commsgs::geometry_msgs::Pose2D& global_pose,
  const std::string mask_frame,
  commsgs::geometry_msgs::Pose2D& mask_pose) const
{
    if (mask_frame != global_frame) {
        // Filter mask and current layer are in different frames:
        // Transform (global_pose.x, global_pose.y) point from current layer frame (global_frame)
        // to mask_pose point in mask_frame
        commsgs::geometry_msgs::TransformStamped transform;
        commsgs::geometry_msgs::PointStamped in, out;
        // in.header.stamp = clock_->now();
        in.header.frame_id = global_frame;
        in.point.x = global_pose.x;
        in.point.y = global_pose.y;
        in.point.z = 0;

        // try {
        //     tf_->transform(in, out, mask_frame, transform_tolerance_);
        // } catch (tf2::TransformException & ex) {
        //     RCLCPP_ERROR(
        //         logger_,
        //         "CostmapFilter: failed to get costmap frame (%s) "
        //         "transformation to mask frame (%s) with error: %s",
        //         global_frame.c_str(), mask_frame.c_str(), ex.what());
        //     return false;
        // }
        mask_pose.x = out.point.x;
        mask_pose.y = out.point.y;
    } else {
        // Filter mask and current layer are in the same frame:
        // Just use global_pose coordinates
        mask_pose = global_pose;
    }

    return true;
}

bool CostmapFilter::worldToMask(
  commsgs::map_msgs::OccupancyGrid::ConstSharedPtr filter_mask,
  double wx, double wy, unsigned int & mx, unsigned int & my) const
{
    const double origin_x = filter_mask->info.origin.position.x;
    const double origin_y = filter_mask->info.origin.position.y;
    const double resolution = filter_mask->info.resolution;
    const unsigned int size_x = filter_mask->info.width;
    const unsigned int size_y = filter_mask->info.height;

    if (wx < origin_x || wy < origin_y) {
        return false;
    }

    mx = static_cast<unsigned int>((wx - origin_x) / resolution);
    my = static_cast<unsigned int>((wy - origin_y) / resolution);
    if (mx >= size_x || my >= size_y) {
        return false;
    }

    return true;
}

unsigned char CostmapFilter::getMaskCost(
  commsgs::map_msgs::OccupancyGrid::ConstSharedPtr filter_mask,
  const unsigned int mx, const unsigned int & my) const
{
    const unsigned int index = my * filter_mask->info.width + mx;

    const char data = filter_mask->data[index];
    if (data == utils::OCC_GRID_UNKNOWN) {
        return NO_INFORMATION;
    } else {
        // Linear conversion from OccupancyGrid data range [OCC_GRID_FREE..OCC_GRID_OCCUPIED]
        // to costmap data range [FREE_SPACE..LETHAL_OBSTACLE]
        return std::round(static_cast<double>(data) * (LETHAL_OBSTACLE - FREE_SPACE) /
            (utils::OCC_GRID_OCCUPIED - utils::OCC_GRID_FREE));
    }
}

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy
