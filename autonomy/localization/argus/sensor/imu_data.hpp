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
 * @file imu_data.hpp
 * @brief IMU sample for FAST-LIVO2 (Eigen; I/O via automsgs).
 */

#ifndef ARGUS_SENSOR_IMU_DATA_HPP_
#define ARGUS_SENSOR_IMU_DATA_HPP_

#include <string>

#include "Eigen/Core"

#include <automsgs/msgs/sensor_msgs/imu.pb.h>

#include "argus/common/macros.hpp"

namespace argus {
namespace sensor {

/**
 * @brief Single IMU measurement used by Argus / LIVO pipelines.
 *
 * Timestamps are seconds. Accelerometer / gyro are in the IMU body frame
 * (m/s², rad/s), matching FAST-LIVO2 / automsgs `sensor_msgs/Imu` conventions.
 */
struct ImuData {
    ARGUS_SHARED_PTR_DEFINITIONS(ImuData);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double timestamp = 0.0; /**< @brief Sample time (seconds). */
    std::string frame_id;   /**< @brief IMU body frame id. */
    Eigen::Vector3d linear_acceleration =
        Eigen::Vector3d::Zero(); /**< @brief Specific force (m/s²), body frame.
                                  */
    Eigen::Vector3d angular_velocity =
        Eigen::Vector3d::Zero(); /**< @brief Angular rate (rad/s), body frame.
                                  */
};

/**
 * @brief Convert automsgs Imu to Argus ImuData.
 * @param msg Source protobuf IMU message.
 * @return Populated ImuData.
 */
ImuData FromProto(const ::automsgs::msgs::sensor_msgs::Imu& msg);

/**
 * @brief Convert Argus ImuData to automsgs Imu.
 * @param data Source sample.
 * @return Populated protobuf IMU message.
 */
::automsgs::msgs::sensor_msgs::Imu ToProto(const ImuData& data);

}  // namespace sensor
}  // namespace argus

#endif  // ARGUS_SENSOR_IMU_DATA_HPP_
