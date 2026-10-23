/*
 * Copyright 2026 The Openbot Authors
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

/**
 * @file voxel_slam_message_adapter.hpp
 * @brief AutoMsgs boundary for the ROS-free Voxel-SLAM system.
 */

#ifndef ATLAS_APP_VOXEL_SLAM_MESSAGE_ADAPTER_HPP_
#define ATLAS_APP_VOXEL_SLAM_MESSAGE_ADAPTER_HPP_

#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/sensor_msgs/imu.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

#include <string>

#include "voxel_slam_system.hpp"

namespace atlas {
namespace app {

/**
 * @brief Converts AutoMsgs sensor messages into Atlas types and publishes the
 * current estimate as AutoMsgs odometry.
 *
 * This adapter is the only Voxel-SLAM application boundary that depends on
 * generated messages. The estimator itself consumes PCL-free C++ values.
 */
class VoxelSlamMessageAdapter {
 public:
  explicit VoxelSlamMessageAdapter(VoxelSlamSystem* system);

  /** @brief Convert and enqueue one AutoMsgs IMU sample. */
  bool AddImu(const ::automsgs::msgs::sensor_msgs::Imu& message);

  /** @brief Convert and process one AutoMsgs PointCloud2 scan. */
  bool AddScan(const ::automsgs::msgs::sensor_msgs::PointCloud2& message);

  /**
   * @brief Serialize the latest estimator state as an AutoMsgs odometry.
   * @return False before initialization or when @p message is null.
   */
  bool GetOdometry(::automsgs::msgs::nav_msgs::Odometry* message,
                   const std::string& frame_id = "map",
                   const std::string& child_frame_id = "base_link") const;

  /** @brief Serialize the global-coordinate keyframe map; frame_id labels it.
   */
  bool GetMap(::automsgs::msgs::sensor_msgs::PointCloud2* message,
              const std::string& frame_id = "map") const;

 private:
  VoxelSlamSystem* system_ = nullptr;
};

}  // namespace app
}  // namespace atlas

#endif  // ATLAS_APP_VOXEL_SLAM_MESSAGE_ADAPTER_HPP_
