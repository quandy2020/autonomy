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

/**
 * @file
 * @brief Timestamped sensor samples and automsgs message helpers.
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

/**
 * @class autodriver::SensorSample
 * @brief Base type for all captured sensor readings with device and host time.
 */
class SensorSample {
public:
    /**
     * @brief Construct a sample with device time initially equal to host time.
     * @param id Stable sensor instance identifier.
     * @param type Sensor modality of this reading.
     * @param device_time Timestamp reported by the device.
     */
    SensorSample(SensorId id, SensorType type, autolink::Time device_time)
        : id_(std::move(id)), type_(type), device_time_(device_time),
          host_time_(device_time) {}

    /**
     * @brief Virtual destructor for polymorphic samples.
     */
    virtual ~SensorSample() = default;

    /**
     * @brief Create a deep copy of this sample.
     * @return Unique pointer to a cloned sample instance.
     */
    virtual std::unique_ptr<SensorSample> Clone() const = 0;

    /**
     * @brief Sensor modality of this reading.
     * @return Configured sensor type.
     */
    SensorType type() const { return type_; }

    /**
     * @brief Stable sensor instance identifier.
     * @return Configured sensor id.
     */
    const SensorId& id() const { return id_; }

    /**
     * @brief Timestamp reported by the device.
     * @return Device time in autolink::Time.
     */
    autolink::Time device_time() const { return device_time_; }

    /**
     * @brief Mapped host receive time for this sample.
     * @return Host time in autolink::Time.
     */
    autolink::Time host_time() const { return host_time_; }

    /**
     * @brief Set the mapped host receive time.
     * @param host_time Host time after clock synchronization.
     */
    void set_host_time(autolink::Time host_time) { host_time_ = host_time; }

    /**
     * @brief Fill a protobuf header with host time and sensor id.
     * @param header Output header to stamp; must not be null.
     */
    void Stamp(automsgs::msgs::std_msgs::Header* header) const {
        *header->mutable_stamp() =
            automsgs::msgs::builtin_interfaces::TimeFromNanoseconds(
                host_time_.ToNanosecond());
        header->set_frame_id(id_);
    }

protected:
    // Stable sensor instance identifier.
    SensorId id_;

    // Sensor modality of this reading.
    SensorType type_;

    // Timestamp reported by the device.
    autolink::Time device_time_;

    // Mapped host receive time for this sample.
    autolink::Time host_time_;
};

/**
 * @class autodriver::TypedSample
 * @brief SensorSample bound to a concrete automsgs protobuf message.
 * @tparam kType Sensor modality handled by this sample.
 * @tparam Msg Underlying automsgs protobuf message type.
 */
template <SensorType kType, typename Msg>
class TypedSample : public SensorSample {
public:
    // Underlying automsgs protobuf message type.
    using Message = Msg;

    /**
     * @brief Construct a typed sample wrapping a protobuf message.
     * @param id Stable sensor instance identifier.
     * @param device_time Timestamp reported by the device.
     * @param msg Populated protobuf payload.
     */
    TypedSample(SensorId id, autolink::Time device_time, Msg msg)
        : SensorSample(std::move(id), kType, device_time),
          msg(std::move(msg)) {}

    /**
     * @brief Create a deep copy of this typed sample.
     * @return Unique pointer to a cloned sample instance.
     */
    std::unique_ptr<SensorSample> Clone() const override {
        return std::make_unique<TypedSample>(*this);
    }

    /**
     * @brief Stamp the embedded message header in place and return it.
     * @return Reference to the stamped protobuf message.
     */
    Message& StampInPlace() {
        Stamp(msg.mutable_header());
        return msg;
    }

    // Embedded automsgs protobuf payload.
    Msg msg;
};

// IMU sample using sensor_msgs/Imu.
using ImuSample =
    TypedSample<SensorType::kImu, automsgs::msgs::sensor_msgs::Imu>;

// GPS sample using sensor_msgs/NavSatFix.
using GpsSample =
    TypedSample<SensorType::kGps, automsgs::msgs::sensor_msgs::NavSatFix>;

/**
 * @class autodriver::CameraFrame
 * @brief Image sample with optional CameraInfo and custom frame_id override.
 */
class CameraFrame
    : public TypedSample<SensorType::kCamera,
                         automsgs::msgs::sensor_msgs::Image> {
 public:
    // Base TypedSample alias for sensor_msgs/Image.
    using Base =
        TypedSample<SensorType::kCamera, automsgs::msgs::sensor_msgs::Image>;
    using Base::Base;

    // Optional camera calibration metadata.
    automsgs::msgs::sensor_msgs::CameraInfo camera_info;

    // True when camera_info has been populated.
    bool has_camera_info{false};

    // Optional frame_id override for the image header.
    std::string frame_id;

    /**
     * @brief Stamp the image header and apply frame_id override when set.
     * @return Reference to the stamped image message.
     */
    Message& StampInPlace() {
        Stamp(msg.mutable_header());
        if (!frame_id.empty()) {
            msg.mutable_header()->set_frame_id(frame_id);
        }
        return msg;
    }
};

// Two-dimensional lidar scan using sensor_msgs/LaserScan.
using LidarScan =
    TypedSample<SensorType::kLidar2d, automsgs::msgs::sensor_msgs::LaserScan>;

/**
 * @class autodriver::LidarCloud
 * @brief PointCloud2 sample with optional frame_id override.
 */
class LidarCloud
    : public TypedSample<SensorType::kLidar3d,
                         automsgs::msgs::sensor_msgs::PointCloud2> {
 public:
    // Base TypedSample alias for sensor_msgs/PointCloud2.
    using Base = TypedSample<SensorType::kLidar3d,
                             automsgs::msgs::sensor_msgs::PointCloud2>;
    using Base::Base;

    // Optional frame_id override for the cloud header.
    std::string frame_id;

    /**
     * @brief Stamp the cloud header and apply frame_id override when set.
     * @return Reference to the stamped point cloud message.
     */
    Message& StampInPlace() {
        Stamp(msg.mutable_header());
        if (!frame_id.empty()) {
            msg.mutable_header()->set_frame_id(frame_id);
        }
        return msg;
    }
};

/**
 * @class autodriver::LidarPacketScan
 * @brief Aggregated raw lidar firing packets (Scan).
 * Uses kLidar3d modality; Publisher may ignore until a typed scan channel exists.
 */
class LidarPacketScan : public SensorSample {
public:
    LidarPacketScan(SensorId id, autolink::Time device_time,
                    std::vector<std::uint8_t> payload, std::size_t packet_bytes)
        : SensorSample(std::move(id), SensorType::kLidar3d, device_time),
          payload_(std::move(payload)),
          packet_bytes_(packet_bytes) {}

    std::unique_ptr<SensorSample> Clone() const override {
        return std::make_unique<LidarPacketScan>(id(), device_time(), payload_,
                                                 packet_bytes_);
    }

    const std::vector<std::uint8_t>& payload() const { return payload_; }
    std::size_t packet_bytes() const { return packet_bytes_; }
    std::size_t packet_count() const {
        return packet_bytes_ == 0 ? 0 : payload_.size() / packet_bytes_;
    }

    std::string frame_id;
    std::string channel;  // optional scan_channel hint

private:
    std::vector<std::uint8_t> payload_;
    std::size_t packet_bytes_ = 0;
};

// Range sample using sensor_msgs/Range.
using RangeSample =
    TypedSample<SensorType::kRangeFinder, automsgs::msgs::sensor_msgs::Range>;

// Wheel odometry sample using nav_msgs/Odometry.
using WheelOdometrySample =
    TypedSample<SensorType::kWheelOdometry, automsgs::msgs::nav_msgs::Odometry>;

// Radar: temporary PointCloud2 placeholder until dedicated radar msgs land.
using RadarSample =
    TypedSample<SensorType::kRadar, automsgs::msgs::sensor_msgs::PointCloud2>;

// Microphone: Image used as raw PCM byte bag (encoding e.g. audio/pcm16).
using MicrophoneSample =
    TypedSample<SensorType::kMicrophone, automsgs::msgs::sensor_msgs::Image>;

/**
 * @brief Build a sensor_msgs/Imu message from gyro and accel arrays.
 * @param gyro Angular velocity in rad/s as [x, y, z].
 * @param accel Linear acceleration in m/s^2 as [x, y, z].
 * @param frame_id Header frame_id for the message.
 * @return Populated sensor_msgs/Imu protobuf message.
 */
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

/**
 * @brief Build a sensor_msgs/NavSatFix message from WGS84 coordinates.
 * @param latitude_deg Latitude in decimal degrees.
 * @param longitude_deg Longitude in decimal degrees.
 * @param altitude_m Altitude above the WGS84 ellipsoid in meters.
 * @param status Fix quality status for NavSatStatus.
 * @return Populated sensor_msgs/NavSatFix protobuf message.
 */
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

/**
 * @brief Build a sensor_msgs/Image message from raw pixel bytes.
 * @param width Image width in pixels.
 * @param height Image height in pixels.
 * @param encoding Pixel encoding string, e.g. "rgb8".
 * @param data Raw pixel bytes in row-major order.
 * @return Populated sensor_msgs/Image protobuf message.
 */
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
