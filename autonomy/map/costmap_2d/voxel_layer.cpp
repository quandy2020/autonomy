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


#include "autonomy/map/costmap_2d/voxel_layer.hpp"

namespace autonomy {
namespace map {
namespace costmap_2d {

void VoxelLayer::onInitialize()
{
    ObstacleLayer::onInitialize();

    // declareParameter("enabled", rclcpp::ParameterValue(true));
    // declareParameter("footprint_clearing_enabled", rclcpp::ParameterValue(true));
    // declareParameter("max_obstacle_height", rclcpp::ParameterValue(2.0));
    // declareParameter("z_voxels", rclcpp::ParameterValue(10));
    // declareParameter("origin_z", rclcpp::ParameterValue(0.0));
    // declareParameter("z_resolution", rclcpp::ParameterValue(0.2));
    // declareParameter("unknown_threshold", rclcpp::ParameterValue(15));
    // declareParameter("mark_threshold", rclcpp::ParameterValue(0));
    // declareParameter("combination_method", rclcpp::ParameterValue(1));
    // declareParameter("publish_voxel_map", rclcpp::ParameterValue(false));

    // node->get_parameter(name_ + "." + "enabled", enabled_);
    // node->get_parameter(name_ + "." + "footprint_clearing_enabled", footprint_clearing_enabled_);
    // node->get_parameter(name_ + "." + "max_obstacle_height", max_obstacle_height_);
    // node->get_parameter(name_ + "." + "z_voxels", size_z_);
    // node->get_parameter(name_ + "." + "origin_z", origin_z_);
    // node->get_parameter(name_ + "." + "z_resolution", z_resolution_);
    // node->get_parameter(name_ + "." + "unknown_threshold", unknown_threshold_);
    // node->get_parameter(name_ + "." + "mark_threshold", mark_threshold_);
    // node->get_parameter(name_ + "." + "publish_voxel_map", publish_voxel_);

    // int combination_method_param{};
    // node->get_parameter(name_ + "." + "combination_method", combination_method_param);
    // combination_method_ = combination_method_from_int(combination_method_param);

    // auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

    // if (publish_voxel_) {
    //     voxel_pub_ = node->create_publisher<nav2_msgs::msg::VoxelGrid>(
    //     "voxel_grid", custom_qos);
    //     voxel_pub_->on_activate();
    // }

    // clearing_endpoints_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("clearing_endpoints", custom_qos);
    // clearing_endpoints_pub_->on_activate();

    // unknown_threshold_ += (VOXEL_BITS - size_z_);
    matchSize();
}

VoxelLayer::~VoxelLayer()
{
}

void VoxelLayer::matchSize()
{
    // std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
    // ObstacleLayer::matchSize();
    // voxel_grid_.resize(size_x_, size_y_, size_z_);
    // assert(voxel_grid_.sizeX() == size_x_ && voxel_grid_.sizeY() == size_y_);
}

void VoxelLayer::reset()
{
    // Call the base class method before adding our own functionality
    ObstacleLayer::reset();
    resetMaps();
}

void VoxelLayer::resetMaps()
{
    // Call the base class method before adding our own functionality
    // Note: at the time this was written, ObstacleLayer doesn't implement
    // resetMaps so this goes to the next layer down Costmap2DLayer which also
    // doesn't implement this, so it actually goes all the way to Costmap2D
    ObstacleLayer::resetMaps();
    // voxel_grid_.reset();
}

void VoxelLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

    if (rolling_window_) {
        updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
    }
    if (!enabled_) {
        return;
    }
    useExtraBounds(min_x, min_y, max_x, max_y);

    bool current = true;
    std::vector<Observation> observations, clearing_observations;

    // get the marking observations
    current = getMarkingObservations(observations) && current;

    // get the clearing observations
    current = getClearingObservations(clearing_observations) && current;

    // update the global current status
    current_ = current;

    // raytrace freespace
    for (unsigned int i = 0; i < clearing_observations.size(); ++i) {
        raytraceFreespace(clearing_observations[i], min_x, min_y, max_x, max_y);
    }

    // // place the new obstacles into a priority queue... each with a priority of zero to begin with
    // for (auto it = observations.begin(); it != observations.end(); ++it) {
    //     const Observation & obs = *it;

    //     const sensor_msgs::msg::PointCloud2 & cloud = *(obs.cloud_);

    //     double sq_obstacle_max_range = obs.obstacle_max_range_ * obs.obstacle_max_range_;
    //     double sq_obstacle_min_range = obs.obstacle_min_range_ * obs.obstacle_min_range_;

    //     sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    //     sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    //     sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    //     for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    //     // if the obstacle is too high or too far away from the robot we won't add it
    //     if (*iter_z > max_obstacle_height_) {
    //         continue;
    //     }

    //     // compute the squared distance from the hitpoint to the pointcloud's origin
    //     double sq_dist = (*iter_x - obs.origin_.x) * (*iter_x - obs.origin_.x) +
    //         (*iter_y - obs.origin_.y) * (*iter_y - obs.origin_.y) +
    //         (*iter_z - obs.origin_.z) * (*iter_z - obs.origin_.z);

    //     // if the point is far enough away... we won't consider it
    //     if (sq_dist >= sq_obstacle_max_range) {
    //         continue;
    //     }

    //     // If the point is too close, do not consider it
    //     if (sq_dist < sq_obstacle_min_range) {
    //         continue;
    //     }

    //     // now we need to compute the map coordinates for the observation
    //     unsigned int mx, my, mz;
    //     if (*iter_z < origin_z_) {
    //         if (!worldToMap3D(*iter_x, *iter_y, origin_z_, mx, my, mz)) {
    //         continue;
    //         }
    //     } else if (!worldToMap3D(*iter_x, *iter_y, *iter_z, mx, my, mz)) {
    //         continue;
    //     }

    //     // mark the cell in the voxel grid and check if we should also mark it in the costmap
    //     if (voxel_grid_.markVoxelInMap(mx, my, mz, mark_threshold_)) {
    //         unsigned int index = getIndex(mx, my);

    //         costmap_[index] = LETHAL_OBSTACLE;
    //         touch(
    //         static_cast<double>(*iter_x), static_cast<double>(*iter_y),
    //         min_x, min_y, max_x, max_y);
    //     }
    //     }
    // }

    // if (publish_voxel_) {
    //     auto grid_msg = std::make_unique<nav2_msgs::msg::VoxelGrid>();
    //     unsigned int size = voxel_grid_.sizeX() * voxel_grid_.sizeY();
    //     grid_msg->size_x = voxel_grid_.sizeX();
    //     grid_msg->size_y = voxel_grid_.sizeY();
    //     grid_msg->size_z = voxel_grid_.sizeZ();
    //     grid_msg->data.resize(size);
    //     memcpy(&grid_msg->data[0], voxel_grid_.getData(), size * sizeof(unsigned int));

    //     grid_msg->origin.x = origin_x_;
    //     grid_msg->origin.y = origin_y_;
    //     grid_msg->origin.z = origin_z_;

    //     grid_msg->resolutions.x = resolution_;
    //     grid_msg->resolutions.y = resolution_;
    //     grid_msg->resolutions.z = z_resolution_;
    //     grid_msg->header.frame_id = global_frame_;
    //     grid_msg->header.stamp = clock_->now();

    //     voxel_pub_->publish(std::move(grid_msg));
    // }

    updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void VoxelLayer::raytraceFreespace(
  const Observation& clearing_observation, double * min_x,
  double * min_y,
  double * max_x,
  double * max_y)
{
    auto clearing_endpoints_ = std::make_unique<commsgs::sensor_msgs::PointCloud2>();

    if (clearing_observation.cloud_->height == 0 || clearing_observation.cloud_->width == 0) {
        return;
    }

    double sensor_x, sensor_y, sensor_z;
    double ox = clearing_observation.origin_.x;
    double oy = clearing_observation.origin_.y;
    double oz = clearing_observation.origin_.z;

    if (!worldToMap3DFloat(ox, oy, oz, sensor_x, sensor_y, sensor_z)) {
        // RCLCPP_WARN(
        //     logger_,
        //     "Sensor origin at (%.2f, %.2f %.2f) is out of map bounds "
        //     "(%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f). "
        //     "The costmap cannot raytrace for it.",
        //     ox, oy, oz,
        //     origin_x_, origin_y_, origin_z_,
        //     origin_x_ + getSizeInMetersX(), origin_y_ + getSizeInMetersY(),
        //     origin_z_ + getSizeInMetersZ());

        return;
    }

    // bool publish_clearing_points;
    // {
    //     auto node = node_.lock();
    //     if (!node) {
    //         throw std::runtime_error{"Failed to lock node"};
    //     }
    //     publish_clearing_points = (node->count_subscribers("clearing_endpoints") > 0);
    // }

    // clearing_endpoints_->data.clear();
    // clearing_endpoints_->width = clearing_observation.cloud_->width;
    // clearing_endpoints_->height = clearing_observation.cloud_->height;
    // clearing_endpoints_->is_dense = true;
    // clearing_endpoints_->is_bigendian = false;

    // sensor_msgs::PointCloud2Modifier modifier(*clearing_endpoints_);
    // modifier.setPointCloud2Fields(
    //     3, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    //     "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    //     "z", 1, sensor_msgs::msg::PointField::FLOAT32);

    // sensor_msgs::PointCloud2Iterator<float> clearing_endpoints_iter_x(*clearing_endpoints_, "x");
    // sensor_msgs::PointCloud2Iterator<float> clearing_endpoints_iter_y(*clearing_endpoints_, "y");
    // sensor_msgs::PointCloud2Iterator<float> clearing_endpoints_iter_z(*clearing_endpoints_, "z");

    // // we can pre-compute the enpoints of the map outside of the inner loop... we'll need these later
    // double map_end_x = origin_x_ + getSizeInMetersX();
    // double map_end_y = origin_y_ + getSizeInMetersY();
    // double map_end_z = origin_z_ + getSizeInMetersZ();

    // sensor_msgs::PointCloud2ConstIterator<float> iter_x(*(clearing_observation.cloud_), "x");
    // sensor_msgs::PointCloud2ConstIterator<float> iter_y(*(clearing_observation.cloud_), "y");
    // sensor_msgs::PointCloud2ConstIterator<float> iter_z(*(clearing_observation.cloud_), "z");

    // for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    //     double wpx = *iter_x;
    //     double wpy = *iter_y;
    //     double wpz = *iter_z;

    //     double distance = dist(ox, oy, oz, wpx, wpy, wpz);
    //     double scaling_fact = 1.0;
    //     scaling_fact = std::max(std::min(scaling_fact, (distance - 2 * resolution_) / distance), 0.0);
    //     wpx = scaling_fact * (wpx - ox) + ox;
    //     wpy = scaling_fact * (wpy - oy) + oy;
    //     wpz = scaling_fact * (wpz - oz) + oz;

    //     double a = wpx - ox;
    //     double b = wpy - oy;
    //     double c = wpz - oz;
    //     double t = 1.0;

    //     // we can only raytrace to a maximum z height
    //     if (wpz > map_end_z) {
    //     // we know we want the vector's z value to be max_z
    //     t = std::max(0.0, std::min(t, (map_end_z - 0.01 - oz) / c));
    //     } else if (wpz < origin_z_) {
    //     // and we can only raytrace down to the floor
    //     // we know we want the vector's z value to be 0.0
    //     t = std::min(t, (origin_z_ - oz) / c);
    //     }

    //     // the minimum value to raytrace from is the origin
    //     if (wpx < origin_x_) {
    //     t = std::min(t, (origin_x_ - ox) / a);
    //     }
    //     if (wpy < origin_y_) {
    //     t = std::min(t, (origin_y_ - oy) / b);
    //     }

    //     // the maximum value to raytrace to is the end of the map
    //     if (wpx > map_end_x) {
    //     t = std::min(t, (map_end_x - ox) / a);
    //     }
    //     if (wpy > map_end_y) {
    //     t = std::min(t, (map_end_y - oy) / b);
    //     }

    //     wpx = ox + a * t;
    //     wpy = oy + b * t;
    //     wpz = oz + c * t;

    //     double point_x, point_y, point_z;
    //     if (worldToMap3DFloat(wpx, wpy, wpz, point_x, point_y, point_z)) {
    //     unsigned int cell_raytrace_max_range = cellDistance(clearing_observation.raytrace_max_range_);
    //     unsigned int cell_raytrace_min_range = cellDistance(clearing_observation.raytrace_min_range_);


    //     // voxel_grid_.markVoxelLine(sensor_x, sensor_y, sensor_z, point_x, point_y, point_z);
    //     voxel_grid_.clearVoxelLineInMap(
    //         sensor_x, sensor_y, sensor_z, point_x, point_y, point_z,
    //         costmap_,
    //         unknown_threshold_, mark_threshold_, FREE_SPACE, NO_INFORMATION,
    //         cell_raytrace_max_range, cell_raytrace_min_range);

    //     updateRaytraceBounds(
    //         ox, oy, wpx, wpy, clearing_observation.raytrace_max_range_,
    //         clearing_observation.raytrace_min_range_, min_x, min_y,
    //         max_x,
    //         max_y);

    //     if (publish_clearing_points) {
    //         *clearing_endpoints_iter_x = wpx;
    //         *clearing_endpoints_iter_y = wpy;
    //         *clearing_endpoints_iter_z = wpz;

    //         ++clearing_endpoints_iter_x;
    //         ++clearing_endpoints_iter_y;
    //         ++clearing_endpoints_iter_z;
    //     }
    //     }
    // }

    // if (publish_clearing_points) {
    //     clearing_endpoints_->header.frame_id = global_frame_;
    //     clearing_endpoints_->header.stamp = clearing_observation.cloud_->header.stamp;

    //     clearing_endpoints_pub_->publish(std::move(clearing_endpoints_));
    // }
}

void VoxelLayer::updateOrigin(double new_origin_x, double new_origin_y)
{
    // project the new origin into the grid
    int cell_ox, cell_oy;
    cell_ox = static_cast<int>((new_origin_x - origin_x_) / resolution_);
    cell_oy = static_cast<int>((new_origin_y - origin_y_) / resolution_);

    // compute the associated world coordinates for the origin cell
    // beacuase we want to keep things grid-aligned
    double new_grid_ox, new_grid_oy;
    new_grid_ox = origin_x_ + cell_ox * resolution_;
    new_grid_oy = origin_y_ + cell_oy * resolution_;

    // To save casting from unsigned int to int a bunch of times
    int size_x = size_x_;
    int size_y = size_y_;

    // we need to compute the overlap of the new and existing windows
    int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
    lower_left_x = std::min(std::max(cell_ox, 0), size_x);
    lower_left_y = std::min(std::max(cell_oy, 0), size_y);
    upper_right_x = std::min(std::max(cell_ox + size_x, 0), size_x);
    upper_right_y = std::min(std::max(cell_oy + size_y, 0), size_y);

    unsigned int cell_size_x = upper_right_x - lower_left_x;
    unsigned int cell_size_y = upper_right_y - lower_left_y;

    // we need a map to store the obstacles in the window temporarily
    unsigned char * local_map = new unsigned char[cell_size_x * cell_size_y];
    unsigned int * local_voxel_map = new unsigned int[cell_size_x * cell_size_y];
    // unsigned int * voxel_map = voxel_grid_.getData();

    // // copy the local window in the costmap to the local map
    // copyMapRegion(
    //     costmap_, lower_left_x, lower_left_y, size_x_, local_map, 0, 0, cell_size_x,
    //     cell_size_x,
    //     cell_size_y);
    // copyMapRegion(
    //     voxel_map, lower_left_x, lower_left_y, size_x_, local_voxel_map, 0, 0, cell_size_x,
    //     cell_size_x,
    //     cell_size_y);

    // // we'll reset our maps to unknown space if appropriate
    // resetMaps();

    // // update the origin with the appropriate world coordinates
    // origin_x_ = new_grid_ox;
    // origin_y_ = new_grid_oy;

    // // compute the starting cell location for copying data back in
    // int start_x = lower_left_x - cell_ox;
    // int start_y = lower_left_y - cell_oy;

    // // now we want to copy the overlapping information back into the map, but in its new location
    // copyMapRegion(
    //     local_map, 0, 0, cell_size_x, costmap_, start_x, start_y, size_x_, cell_size_x,
    //     cell_size_y);
    // copyMapRegion(
    //     local_voxel_map, 0, 0, cell_size_x, voxel_map, start_x, start_y, size_x_,
    //     cell_size_x,
    //     cell_size_y);

    // make sure to clean up
    delete[] local_map;
    delete[] local_voxel_map;
}


}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy