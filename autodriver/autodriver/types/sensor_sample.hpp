/*
 * Copyright 2026 Autodriver contributors
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

#ifndef AUTODRIVER_TYPES_SENSOR_SAMPLE_HPP_
#define AUTODRIVER_TYPES_SENSOR_SAMPLE_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "autodriver/sensor_id.hpp"
#include "autodriver/types/sensor_type.hpp"
#include "autolink/time/time.hpp"
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/sensor_msgs/imu.pb.h>
#include <automsgs/msgs/sensor_msgs/laser_scan.pb.h>
#include <automsgs/msgs/sensor_msgs/nav_sat_fix.pb.h>
#include <automsgs/msgs/sensor_msgs/nav_sat_status.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include <automsgs/msgs/sensor_msgs/range.pb.h>
#include <automsgs/msgs/std_msgs/header.pb.h>
#include <automsgs/msgs/time_utils.hpp>

namespace autodriver {

class SensorSample {
public:
    SensorSample(SensorId id, SensorType type, autolink::Time device_time)
        : id_(std::move(id)), type_(type), device_time_(device_time),
          host_time_(device_time) {}

    virtual ~SensorSample() = default;

    virtual std::unique_ptr<SensorSample> Clone() const = 0;

    SensorType type() const { return type_; }
    const SensorId& id() const { return id_; }
    autolink::Time device_time() const { return device_time_; }
    autolink::Time host_time() const { return host_time_; }
    void set_host_time(autolink::Time host_time) { host_time_ = host_time; }

    void Stamp(automsgs::msgs::std_msgs::Header* header) const {
        *header->mutable_stamp() =
            automsgs::msgs::builtin_interfaces::TimeFromNanoseconds(
                host_time_.ToNanosecond());
        header->set_frame_id(id_);
    }

protected:
    SensorId id_;
    SensorType type_;
    autolink::Time device_time_;
    autolink::Time host_time_;
};

template <SensorType kType, typename Msg>
class TypedSample : public SensorSample {
public:
    using Message = Msg;

    TypedSample(SensorId id, autolink::Time device_time, Msg msg)
        : SensorSample(std::move(id), kType, device_time),
          msg(std::move(msg)) {}

    std::unique_ptr<SensorSample> Clone() const override {
        return std::make_unique<TypedSample>(*this);
    }

    Message& StampInPlace() {
        Stamp(msg.mutable_header());
        return msg;
    }

    Msg msg;
};

using ImuSample =
    TypedSample<SensorType::kImu, automsgs::msgs::sensor_msgs::Imu>;
using GpsSample =
    TypedSample<SensorType::kGps, automsgs::msgs::sensor_msgs::NavSatFix>;

class CameraFrame
    : public TypedSample<SensorType::kCamera,
                         automsgs::msgs::sensor_msgs::Image> {
 public:
    using Base =
        TypedSample<SensorType::kCamera, automsgs::msgs::sensor_msgs::Image>;
    using Base::Base;

    automsgs::msgs::sensor_msgs::CameraInfo camera_info;
    bool has_camera_info{false};
    std::string frame_id;

    Message& StampInPlace() {
        Stamp(msg.mutable_header());
        if (!frame_id.empty()) {
            msg.mutable_header()->set_frame_id(frame_id);
        }
        return msg;
    }
};

using LidarScan =
    TypedSample<SensorType::kLidar2d, automsgs::msgs::sensor_msgs::LaserScan>;

class LidarCloud
    : public TypedSample<SensorType::kLidar3d,
                         automsgs::msgs::sensor_msgs::PointCloud2> {
 public:
    using Base = TypedSample<SensorType::kLidar3d,
                             automsgs::msgs::sensor_msgs::PointCloud2>;
    using Base::Base;

    std::string frame_id;

    Message& StampInPlace() {
        Stamp(msg.mutable_header());
        if (!frame_id.empty()) {
            msg.mutable_header()->set_frame_id(frame_id);
        }
        return msg;
    }
};

using RangeSample =
    TypedSample<SensorType::kRangeFinder, automsgs::msgs::sensor_msgs::Range>;
using WheelOdometrySample =
    TypedSample<SensorType::kWheelOdometry, automsgs::msgs::nav_msgs::Odometry>;

inline automsgs::msgs::sensor_msgs::Imu ImuMsg(
    const std::array<double, 3>& gyro, const std::array<double, 3>& accel,
    const std::string& frame_id = "camera_imu_optical_frame") {
    automsgs::msgs::sensor_msgs::Imu msg;
    msg.mutable_header()->set_frame_id(frame_id);
    msg.mutable_angular_velocity()->set_x(gyro[0]);
    msg.mutable_angular_velocity()->set_y(gyro[1]);
    msg.mutable_angular_velocity()->set_z(gyro[2]);
    msg.mutable_linear_acceleration()->set_x(accel[0]);
    msg.mutable_linear_acceleration()->set_y(accel[1]);
    msg.mutable_linear_acceleration()->set_z(accel[2]);
    for (int i = 0; i < 9; ++i) {
        msg.add_angular_velocity_covariance(0.0);
        msg.add_linear_acceleration_covariance(0.0);
        msg.add_orientation_covariance(i == 0 ? -1.0 : 0.0);
    }
    return msg;
}

inline automsgs::msgs::sensor_msgs::NavSatFix GpsMsg(
    double latitude_deg, double longitude_deg, double altitude_m,
    automsgs::msgs::sensor_msgs::NavSatStatus::Status status) {
    automsgs::msgs::sensor_msgs::NavSatFix msg;
    msg.set_latitude(latitude_deg);
    msg.set_longitude(longitude_deg);
    msg.set_altitude(altitude_m);
    msg.mutable_status()->set_status(status);
    return msg;
}

inline automsgs::msgs::sensor_msgs::Image ImageMsg(
    std::uint32_t width, std::uint32_t height, std::string encoding,
    const std::vector<std::uint8_t>& data) {
    automsgs::msgs::sensor_msgs::Image msg;
    msg.set_width(width);
    msg.set_height(height);
    msg.set_encoding(std::move(encoding));
    msg.set_is_bigendian(false);
    if (height > 0) {
        msg.set_step(static_cast<std::uint32_t>(data.size() / height));
    }
    msg.set_data(reinterpret_cast<const char*>(data.data()), data.size());
    return msg;
}

}  // namespace autodriver

#endif  // AUTODRIVER_TYPES_SENSOR_SAMPLE_HPP_
