#pragma once
#include <automsgs/msgs/geometry_msgs/vector3.pb.h>
#include <automsgs/msgs/std_msgs/header.pb.h>

namespace automsgs {
namespace msgs {
namespace sensor_msgs {

struct Imu {
    std_msgs::Header header_;
    bool has_header_ = false;
    geometry_msgs::Vector3 linear_acceleration_;
    geometry_msgs::Vector3 angular_velocity_;
    bool has_linear_acceleration_ = false;
    bool has_angular_velocity_ = false;

    bool has_header() const {
        return has_header_;
    }
    const std_msgs::Header& header() const {
        return header_;
    }
    std_msgs::Header* mutable_header() {
        has_header_ = true;
        return &header_;
    }
    bool has_linear_acceleration() const {
        return has_linear_acceleration_;
    }
    bool has_angular_velocity() const {
        return has_angular_velocity_;
    }
    const geometry_msgs::Vector3& linear_acceleration() const {
        return linear_acceleration_;
    }
    const geometry_msgs::Vector3& angular_velocity() const {
        return angular_velocity_;
    }
    geometry_msgs::Vector3* mutable_linear_acceleration() {
        has_linear_acceleration_ = true;
        return &linear_acceleration_;
    }
    geometry_msgs::Vector3* mutable_angular_velocity() {
        has_angular_velocity_ = true;
        return &angular_velocity_;
    }
};

}  // namespace sensor_msgs
}  // namespace msgs
}  // namespace automsgs
