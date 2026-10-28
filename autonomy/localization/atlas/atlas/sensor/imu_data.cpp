// Copyright 2026 The Openbot Authors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "atlas/sensor/imu_data.hpp"

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/std_msgs/header.pb.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>

namespace autonomy::localization::atlas {
namespace {

constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;

bool Fail(std::string* error, std::string message) {
    if (error != nullptr) {
        *error = std::move(message);
    }
    return false;
}

bool TimestampFromProto(const ::automsgs::msgs::builtin_interfaces::Time& stamp,
                        std::int64_t* timestamp_ns, std::string* error) {
    if (stamp.nanosec() >= kNanosecondsPerSecond) {
        return Fail(error, "header stamp nanosec must be less than 1e9");
    }
    *timestamp_ns =
        static_cast<std::int64_t>(stamp.sec()) * kNanosecondsPerSecond +
        static_cast<std::int64_t>(stamp.nanosec());
    return true;
}

bool TimestampToProto(std::int64_t timestamp_ns,
                      ::automsgs::msgs::builtin_interfaces::Time* stamp,
                      std::string* error) {
    std::int64_t sec = timestamp_ns / kNanosecondsPerSecond;
    std::int64_t nanosec = timestamp_ns % kNanosecondsPerSecond;
    if (nanosec < 0) {
        --sec;
        nanosec += kNanosecondsPerSecond;
    }
    if (sec < std::numeric_limits<std::int32_t>::min() ||
        sec > std::numeric_limits<std::int32_t>::max()) {
        return Fail(error, "timestamp seconds do not fit AutoMsgs int32 sec");
    }
    stamp->set_sec(static_cast<std::int32_t>(sec));
    stamp->set_nanosec(static_cast<std::uint32_t>(nanosec));
    return true;
}

bool IsFiniteVector(const std::array<double, 3>& value) {
    return std::isfinite(value[0]) && std::isfinite(value[1]) &&
           std::isfinite(value[2]);
}

template <std::size_t Size>
bool IsFiniteArray(const std::array<double, Size>& value) {
    for (const double element : value) {
        if (!std::isfinite(element)) {
            return false;
        }
    }
    return true;
}

template <typename RepeatedField>
bool ReadCovariance(const RepeatedField& source, bool* is_present,
                    std::array<double, 9>* destination, std::string* error,
                    const char* field_name) {
    if (source.empty()) {
        *is_present = false;
        destination->fill(0.0);
        return true;
    }
    if (static_cast<std::size_t>(source.size()) != destination->size()) {
        return Fail(error, std::string(field_name) +
                               " must contain exactly 9 values when present");
    }
    *is_present = true;
    for (int index = 0; index < source.size(); ++index) {
        (*destination)[static_cast<std::size_t>(index)] = source.Get(index);
    }
    if (!IsFiniteArray(*destination)) {
        return Fail(error, std::string(field_name) + " must be finite");
    }
    return true;
}

}  // namespace

bool FromProto(const ::automsgs::msgs::sensor_msgs::Imu& message, ImuData* data,
               std::string* error) {
    if (data == nullptr) {
        return Fail(error, "ImuData output must not be null");
    }
    if (!message.has_header() || !message.header().has_stamp()) {
        return Fail(error, "IMU message must contain header.stamp");
    }
    if (!message.has_linear_acceleration()) {
        return Fail(error, "IMU message must contain linear_acceleration");
    }
    if (!message.has_angular_velocity()) {
        return Fail(error, "IMU message must contain angular_velocity");
    }

    ImuData result;
    if (!TimestampFromProto(message.header().stamp(), &result.timestamp_ns,
                            error)) {
        return false;
    }
    result.frame_id = message.header().frame_id();
    result.has_orientation = message.has_orientation();
    if (result.has_orientation) {
        result.orientation_xyzw = {
            message.orientation().x(), message.orientation().y(),
            message.orientation().z(), message.orientation().w()};
        if (!IsFiniteArray(result.orientation_xyzw)) {
            return Fail(error, "IMU orientation must be finite");
        }
    }
    result.linear_acceleration_mps2 = {message.linear_acceleration().x(),
                                       message.linear_acceleration().y(),
                                       message.linear_acceleration().z()};
    result.angular_velocity_radps = {message.angular_velocity().x(),
                                     message.angular_velocity().y(),
                                     message.angular_velocity().z()};
    if (!IsFiniteVector(result.linear_acceleration_mps2) ||
        !IsFiniteVector(result.angular_velocity_radps)) {
        return Fail(error,
                    "IMU acceleration and angular velocity must be finite");
    }
    if (!ReadCovariance(message.orientation_covariance(),
                        &result.has_orientation_covariance,
                        &result.orientation_covariance, error,
                        "orientation_covariance") ||
        !ReadCovariance(message.angular_velocity_covariance(),
                        &result.has_angular_velocity_covariance,
                        &result.angular_velocity_covariance, error,
                        "angular_velocity_covariance") ||
        !ReadCovariance(message.linear_acceleration_covariance(),
                        &result.has_linear_acceleration_covariance,
                        &result.linear_acceleration_covariance, error,
                        "linear_acceleration_covariance")) {
        return false;
    }
    *data = std::move(result);
    return true;
}

bool ToProto(const ImuData& data, ::automsgs::msgs::sensor_msgs::Imu* message,
             std::string* error) {
    if (message == nullptr) {
        return Fail(error, "IMU message output must not be null");
    }
    if (!IsFiniteVector(data.linear_acceleration_mps2) ||
        !IsFiniteVector(data.angular_velocity_radps)) {
        return Fail(error,
                    "IMU acceleration and angular velocity must be finite");
    }
    if ((data.has_orientation && !IsFiniteArray(data.orientation_xyzw)) ||
        (data.has_orientation_covariance &&
         !IsFiniteArray(data.orientation_covariance)) ||
        (data.has_angular_velocity_covariance &&
         !IsFiniteArray(data.angular_velocity_covariance)) ||
        (data.has_linear_acceleration_covariance &&
         !IsFiniteArray(data.linear_acceleration_covariance))) {
        return Fail(error,
                    "IMU orientation and covariance values must be finite");
    }

    ::automsgs::msgs::sensor_msgs::Imu result;
    auto* header = result.mutable_header();
    if (!TimestampToProto(data.timestamp_ns, header->mutable_stamp(), error)) {
        return false;
    }
    header->set_frame_id(data.frame_id);

    if (data.has_orientation) {
        auto* orientation = result.mutable_orientation();
        orientation->set_x(data.orientation_xyzw[0]);
        orientation->set_y(data.orientation_xyzw[1]);
        orientation->set_z(data.orientation_xyzw[2]);
        orientation->set_w(data.orientation_xyzw[3]);
    }

    auto* acceleration = result.mutable_linear_acceleration();
    acceleration->set_x(data.linear_acceleration_mps2[0]);
    acceleration->set_y(data.linear_acceleration_mps2[1]);
    acceleration->set_z(data.linear_acceleration_mps2[2]);
    auto* angular_velocity = result.mutable_angular_velocity();
    angular_velocity->set_x(data.angular_velocity_radps[0]);
    angular_velocity->set_y(data.angular_velocity_radps[1]);
    angular_velocity->set_z(data.angular_velocity_radps[2]);
    if (data.has_orientation_covariance) {
        for (const double value : data.orientation_covariance) {
            result.add_orientation_covariance(value);
        }
    }
    if (data.has_angular_velocity_covariance) {
        for (const double value : data.angular_velocity_covariance) {
            result.add_angular_velocity_covariance(value);
        }
    }
    if (data.has_linear_acceleration_covariance) {
        for (const double value : data.linear_acceleration_covariance) {
            result.add_linear_acceleration_covariance(value);
        }
    }
    message->Swap(&result);
    return true;
}

}  // namespace autonomy::localization::atlas
